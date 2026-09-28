/*
 * app.c - StarCrawler como nodo micro-ROS
 * ====================================================================
 * El ESP32 deja de ser un esclavo con protocolo serie propio y pasa a
 * ser un nodo ROS 2 nativo, como el de Donatello (TFG_MARIA_JOSE).
 * Desaparecen proto.c y el nodo puente starcrawler_driver.
 *
 *   Suscripciones
 *     /cmd_vel           geometry_msgs/Twist          traccion
 *     /crawler/command   starcrawler_msgs/CrawlerCommand   elevacion
 *
 *   Publicaciones
 *     /starcrawler/state starcrawler_msgs/RobotState  a 50 Hz
 *     /joint_states      sensor_msgs/JointState       a 50 Hz
 *
 * Reparto de tareas (como en Donatello):
 *   nucleo 0 - micro-ROS (executor + publicacion) y watchdog
 *   nucleo 1 - lazo de control a 100 Hz, que es lo que no puede jitter
 *
 * Compila con micro_ros_setup (ESP-IDF 4.1) y funciona en el banco con el
 * ESP32 solo; con el robot, sin probar. Ver README.md.
 */

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw/qos_profiles.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <starcrawler_msgs/msg/crawler_command.h>
#include <starcrawler_msgs/msg/robot_state.h>
#include <rosidl_runtime_c/string_functions.h>

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "idf_compat.h"

#include "config.h"
#include "control_core.h"
#include "hw.h"

/* --- Geometria del robot -------------------------------------------- */

/* Sacados del CAD (star-uma/SimulacionOrugas, Ejecutables/DefinicionParametros.m):
 *   R = 0.15279/2  radio de la polea activa
 *   L = 0.524      distancia lateral entre orugas
 * El radio EFECTIVO con la banda tensada no es el geometrico: hay que
 * refinarlo rodando (issue #13). */
#define RADIO_POLEA_M       0.0764f
#define SEPARACION_VIAS_M   0.524f

#define RAD_A_GRADOS        57.29577951f
#define GRADOS_A_RAD        0.01745329252f

/* Periodos de las tareas */
#define PERIODO_CONTROL_MS  CICLO_CONTROL_MS   /* 10 ms -> 100 Hz */
#define PERIODO_ROS_MS      20                 /* 50 Hz */
#define PERIODO_WDT_MS      50

/* Si el executor deja de girar, el nodo esta muerto: reiniciar */
#define WDT_EXECUTOR_MS     2000

/* El cliente XRCE no reenvia CREATE_CLIENT si el agente se reinicia: sin
 * esto habria que apagar y encender la placa. Ping cada segundo y, tras
 * PING_FALLOS_MAX seguidos sin respuesta, parada segura y reinicio. */
#define PING_CADA_CICLOS    50
#define PING_TIMEOUT_MS     20
#define PING_FALLOS_MAX     5

/* Sin hora del agente los mensajes salen con sello 0 y robot_state_publisher
 * descarta todos los /joint_states: sin TF de las orugas. La hora se
 * resincroniza en cada ping: acota el error aunque el reloj del PC derive. */

#define RCCHECK(fn) { rcl_ret_t rc_ = fn; if (rc_ != RCL_RET_OK) { \
    vTaskDelay(pdMS_TO_TICKS(100)); esp_restart(); } }
#define RCNOCHECK(fn) { rcl_ret_t rc_ = fn; (void)rc_; }

/* --- Entidades ROS --------------------------------------------------- */

static rcl_node_t          nodo;
static rclc_support_t      soporte;
static rcl_allocator_t     allocator;
static rclc_executor_t     executor;

static rcl_subscription_t  sub_cmd_vel;
static rcl_subscription_t  sub_crawler;
static rcl_publisher_t     pub_estado;
static rcl_publisher_t     pub_joints;

static geometry_msgs__msg__Twist              msg_cmd_vel;
static starcrawler_msgs__msg__CrawlerCommand  msg_crawler;
static starcrawler_msgs__msg__RobotState      msg_estado;
static sensor_msgs__msg__JointState           msg_joints;

/* --- Estado compartido entre tareas ---------------------------------- */

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/* Consignas que escribe micro-ROS y lee el lazo de control */
static float    consignaIzqDps = 0.0f;
static float    consignaDerDps = 0.0f;
static int8_t   consignaCmd[CC_NUM_ORUGAS]  = {0, 0, 0, 0};
static float    consignaObjDeg[CC_NUM_ORUGAS] = {180.0f, 180.0f, 180.0f, 180.0f};
static bool     usarPosicion = false;
static bool     emergencia   = false;

/* Estado que escribe el lazo de control y lee micro-ROS */
static float    anguloDeg[CC_NUM_ORUGAS] = {180.0f, 180.0f, 180.0f, 180.0f};
static bool     encoderOk[CC_NUM_ORUGAS] = {false, false, false, false};
static float    velIzqReal = 0.0f;
static float    velDerReal = 0.0f;
static bool     canOk      = false;
static bool     enSeguridad = true;
static uint16_t bitsError  = 0;

static volatile int64_t tickExecutor = 0;

/* Ultimo mensaje valido de cada topico, en hw_millis(). /crawler/command
 * es el latido de la parada: sin el, estado seguro. Sin /cmd_vel, la
 * traccion baja a 0 con par. */
static volatile uint32_t msCmdVel  = 0;
static volatile uint32_t msCrawler = 0;

/* Nombres de las articulaciones, en el orden {FR, FL, RR, RL} */
static const char *NOMBRE_JOINT[CC_NUM_ORUGAS] = {
    "crawler_fr_joint", "crawler_fl_joint",
    "crawler_rr_joint", "crawler_rl_joint"
};

static const uint32_t CAN_ID[CC_NUM_ORUGAS] = {
    CAN_ID_FR, CAN_ID_FL, CAN_ID_RR, CAN_ID_RL
};

static const float SIGNO_COMPENSACION[CC_NUM_ORUGAS] = TABLA_SIGNO_COMPENSACION;

/* --- Conversion de angulos ------------------------------------------- */

/* Dos espacios y una unica traduccion, igual que hacia protocol.py:
 *   - Grados de encoder: 180 = oruga horizontal. FL y RR van espejadas.
 *   - Radianes de elevacion: positivo = brazo levantado, 0 = horizontal,
 *     igual en las cuatro orugas.
 * Vertical arriba = +90 grados de elevacion = {90, 270, 270, 90} en
 * encoder para {FR, FL, RR, RL}, que es lo que documenta el TFG. */

static bool esEspejada(int i) { return (i == 1 || i == 2); }  /* FL, RR */

static float elevacionAEncoderDeg(int i, float elevacionRad) {
    const float d = elevacionRad * RAD_A_GRADOS;
    return esEspejada(i) ? (180.0f + d) : (180.0f - d);
}

static float encoderAElevacionRad(int i, float encoderDeg) {
    const float d = esEspejada(i) ? (encoderDeg - 180.0f) : (180.0f - encoderDeg);
    return d * GRADOS_A_RAD;
}

/* --- Callbacks de las suscripciones ---------------------------------- */

/* Un NaN pasa por cc_saturar y por la rampa: esos mensajes se descartan
 * sin refrescar la marca, como si no hubieran llegado. */
static bool vectorFinito(const geometry_msgs__msg__Vector3 *v) {
    return isfinite(v->x) && isfinite(v->y) && isfinite(v->z);
}

static void cb_cmd_vel(const void *msgin) {
    const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
    if (!vectorFinito(&m->linear) || !vectorFinito(&m->angular)) return;

    /* Cinematica inversa de un diferencial: cada via a su velocidad
     * lineal, y de ahi a grados por segundo del eje del RMD. */
    const float v = (float)m->linear.x;
    const float w = (float)m->angular.z;

    const float vIzq = v - (SEPARACION_VIAS_M / 2.0f) * w;
    const float vDer = v + (SEPARACION_VIAS_M / 2.0f) * w;

    const float dpsIzq = (vIzq / RADIO_POLEA_M) * RAD_A_GRADOS;
    const float dpsDer = (vDer / RADIO_POLEA_M) * RAD_A_GRADOS;
    /* Un double finito enorme desborda el float */
    if (!isfinite(dpsIzq) || !isfinite(dpsDer)) return;

    const uint32_t ahora = hw_millis();
    portENTER_CRITICAL(&mux);
    consignaIzqDps = cc_saturar(dpsIzq, VEL_MAX_DPS);
    consignaDerDps = cc_saturar(dpsDer, VEL_MAX_DPS);
    msCmdVel = ahora;
    portEXIT_CRITICAL(&mux);
}

static void cb_crawler(const void *msgin) {
    const starcrawler_msgs__msg__CrawlerCommand *m =
        (const starcrawler_msgs__msg__CrawlerCommand *)msgin;
    /* Con el cast cae tambien el double que no cabe en un float */
    for (int i = 0; i < CC_NUM_ORUGAS; i++) {
        if (!isfinite((float)m->target[i])) return;
    }

    const uint32_t ahora = hw_millis();
    portENTER_CRITICAL(&mux);
    emergencia   = m->emergency_stop;
    usarPosicion = m->use_position;
    for (int i = 0; i < CC_NUM_ORUGAS; i++) {
        /* increment viene en convenio de elevacion (+1 sube el brazo).
         * El firmware trabaja en grados de encoder, donde subir el brazo
         * puede significar aumentar o disminuir segun el espejado. */
        const int8_t inc = m->increment[i];
        consignaCmd[i] = esEspejada(i) ? inc : (int8_t)(-inc);
        consignaObjDeg[i] = elevacionAEncoderDeg(i, (float)m->target[i]);
    }
    msCrawler = ahora;
    portEXIT_CRITICAL(&mux);
}

/* --- Traccion --------------------------------------------------------- */

static bool enviarVelocidadRMD(uint32_t id, float dps) {
    uint8_t trama[8];
    cc_tramaVelocidadRMD(dps, trama);
    return canbus_enviar(id, trama);
}

static void liberarTraccion(void) {
    uint8_t trama[8];
    cc_tramaLiberarRMD(trama);
    for (int i = 0; i < CC_NUM_ORUGAS; i++) {
        canbus_enviar(CAN_ID[i], trama);
        esp_rom_delay_us(CAN_INTER_FRAME_US);
    }
}

/* --- Lazo de control (nucleo 1, 100 Hz) ------------------------------ */

static void TaskControl(void *arg) {
    (void)arg;

    float velIzqActual = 0.0f, velDerActual = 0.0f;
    bool  enMarcha[CC_NUM_ORUGAS] = {false, false, false, false};
    int8_t ultimoCmd[CC_NUM_ORUGAS] = {0, 0, 0, 0};
    /* Ultimo angulo bueno de cada encoder, como en la version Arduino */
    float ang[CC_NUM_ORUGAS] = {180.0f, 180.0f, 180.0f, 180.0f};
    /* Lo ultimo que dijo cada RMD, y la compensacion que llevaba su trama:
     * se descuenta para quedarse con la velocidad de la banda */
    float    velMotor[CC_NUM_ORUGAS]    = {0.0f, 0.0f, 0.0f, 0.0f};
    float    compEnviada[CC_NUM_ORUGAS] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint32_t msMotor[CC_NUM_ORUGAS]     = {0, 0, 0, 0};
    bool     respondio[CC_NUM_ORUGAS]   = {false, false, false, false};
    uint32_t ciclo = 0;

    TickType_t ultimoDespertar = xTaskGetTickCount();
    const TickType_t periodo = pdMS_TO_TICKS(PERIODO_CONTROL_MS);

    for (;;) {
        ciclo++;
        canbus_atender();
        /* Las marcas antes que la hora: una marca posterior a ahora daria
         * la vuelta en la resta sin signo y el watchdog saltaria. */
        const uint32_t marcaCmdVel  = msCmdVel;
        const uint32_t marcaCrawler = msCrawler;
        const uint32_t ahora = hw_millis();

        /* 0. Respuestas de los RMD a las tramas anteriores */
        uint32_t idRx;
        uint8_t  rx[8];
        cc_RespuestaRMD resp;
        while (canbus_recibir(&idRx, rx)) {
            const int m = cc_leerRespuestaRMD(idRx, rx, CAN_ID, &resp);
            if (m < 0) continue;
            respondio[m] = true;
            msMotor[m]   = ahora;
            /* La de 0x80 (liberado) no trae medida */
            velMotor[m]  = (resp.comando == 0xA2) ? (float)resp.velocidad_dps : 0.0f;
        }

        /* 1. Leer los cuatro encoders */
        bool  ok[CC_NUM_ORUGAS];
        uint16_t errores = 0;
        for (int i = 0; i < CC_NUM_ORUGAS; i++) {
            float leido;
            ok[i] = encoders_leer(i, &leido);
            if (ok[i]) ang[i] = leido;
            else errores |= (1u << i);
        }

        /* 2. Tomar una foto de las consignas */
        portENTER_CRITICAL(&mux);
        const float objIzq = consignaIzqDps;
        const float objDer = consignaDerDps;
        const bool  emer   = emergencia;
        const bool  porPos = usarPosicion;
        int8_t cmdPedido[CC_NUM_ORUGAS];
        float  objDeg[CC_NUM_ORUGAS];
        for (int i = 0; i < CC_NUM_ORUGAS; i++) {
            cmdPedido[i] = consignaCmd[i];
            objDeg[i]    = consignaObjDeg[i];
        }
        portEXIT_CRITICAL(&mux);

        /* 3. Watchdog por topico: sin /crawler/command, estado seguro; sin
         * /cmd_vel, traccion a 0 por la rampa y con par */
        const bool vencido = cc_watchdogExpirado(
            ahora, marcaCrawler, WATCHDOG_TIMEOUT_MS);
        const bool sinCmdVel = cc_watchdogExpirado(
            ahora, marcaCmdVel, WATCHDOG_TIMEOUT_MS);

        if (emer || vencido) {
            liberarTraccion();
            steppers_pararTodos();
            velIzqActual = 0.0f;
            velDerActual = 0.0f;
            for (int i = 0; i < CC_NUM_ORUGAS; i++) {
                enMarcha[i] = false;
                ultimoCmd[i] = 0;
                compEnviada[i] = 0.0f;
            }
            if (vencido) errores |= CC_ERR_WATCHDOG;
        } else {
            /* 4. Elevacion: posicion en lazo cerrado o incremental */
            for (int i = 0; i < CC_NUM_ORUGAS; i++) {
                int8_t cmd;
                if (porPos) {
                    /* Sin encoder valido no hay lazo cerrado: brazo parado */
                    cmd = ok[i] ? cc_controlPosicion(ang[i], objDeg[i],
                                                     enMarcha[i],
                                                     UMBRAL_ARRANQUE_DEG,
                                                     UMBRAL_PARADA_DEG)
                                : 0;
                } else {
                    cmd = cmdPedido[i];
                }
                cmd = cc_aplicarLimites(cmd, ang[i], ok[i],
                                        ANGULO_MIN_DEG, ANGULO_MAX_DEG);
                enMarcha[i] = (cmd != 0);
                ultimoCmd[i] = cmd;
                steppers_comando(i, cmd);
            }

            /* 5. Traccion, con rampa igual que en la version Arduino */
            velIzqActual = cc_rateLimiter(velIzqActual,
                                          sinCmdVel ? 0.0f : objIzq,
                                          RATE_LIMIT_DPS_CICLO);
            velDerActual = cc_rateLimiter(velDerActual,
                                          sinCmdVel ? 0.0f : objDer,
                                          RATE_LIMIT_DPS_CICLO);

            if (ciclo % ENVIO_CAN_CADA_N_CICLOS == 0) {
                /* Refleja el ultimo envio: tras recuperar el bus, vuelve */
                bool envioOk = true;
                for (int i = 0; i < CC_NUM_ORUGAS; i++) {
                    /* Lado izquierdo invertido, como en el firmware original */
                    const bool izquierda = (i == 1 || i == 3);  /* FL, RL */
                    const float v = izquierda ? -velIzqActual : velDerActual;

                    float comp = 0.0f;
#if COMPENSACION_TRACCION
                    /* Mientras la oruga bascula, su RMD gira para que la
                     * banda no arrastre. En este esquema se suma. */
                    if (ultimoCmd[i] != 0) {
                        comp = ((ultimoCmd[i] > 0) ? -COMPENSACION_DPS
                                                   : COMPENSACION_DPS)
                             * SIGNO_COMPENSACION[i];
                    }
#endif
                    compEnviada[i] = comp;
                    if (!enviarVelocidadRMD(CAN_ID[i], v + comp)) envioOk = false;
                    esp_rom_delay_us(CAN_INTER_FRAME_US);
                }
                canOk = envioOk;
            }
        }

        /* 6. Velocidad de cada lado segun sus RMD, sin la compensacion. El
         * que no ha respondido en RMD_TIMEOUT_MS no cuenta y lo marca su bit */
        float suma[2] = {0.0f, 0.0f};     /* [0] derecha, [1] izquierda */
        int   vivos[2] = {0, 0};
        for (int i = 0; i < CC_NUM_ORUGAS; i++) {
            const int izquierda = (i == 1 || i == 3);
            if (!respondio[i] ||
                (uint32_t)(ahora - msMotor[i]) > RMD_TIMEOUT_MS) {
                errores |= CC_ERR_RMD(i);
                continue;
            }
            const float banda = velMotor[i] - compEnviada[i];
            suma[izquierda] += izquierda ? -banda : banda;
            vivos[izquierda]++;
        }

        /* 7. Publicar el estado para la tarea de micro-ROS */
        portENTER_CRITICAL(&mux);
        for (int i = 0; i < CC_NUM_ORUGAS; i++) {
            anguloDeg[i] = ang[i];
            encoderOk[i] = ok[i];
        }
        velIzqReal  = vivos[1] ? suma[1] / vivos[1] : 0.0f;
        velDerReal  = vivos[0] ? suma[0] / vivos[0] : 0.0f;
        enSeguridad = (emer || vencido);
        bitsError   = errores | (canOk ? 0 : CC_ERR_CAN)
                    | (HW_SIMULADO ? HW_ERR_SIMULADO : 0);
        portEXIT_CRITICAL(&mux);

        vTaskDelayUntil(&ultimoDespertar, periodo);
    }
}

/* --- micro-ROS (nucleo 0, 50 Hz) ------------------------------------- */

static void TaskMicroROS(void *arg) {
    (void)arg;
    TickType_t ultimoDespertar = xTaskGetTickCount();
    uint32_t ciclos = 0;
    int fallosPing = 0;

    for (;;) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

        if (++ciclos % PING_CADA_CICLOS == 0) {
            if (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
                fallosPing = 0;
                RCNOCHECK(rmw_uros_sync_session(PING_TIMEOUT_MS));
            } else if (++fallosPing >= PING_FALLOS_MAX) {
                liberarTraccion();
                steppers_pararTodos();
                vTaskDelay(pdMS_TO_TICKS(20));
                esp_restart();
            }
        }

        portENTER_CRITICAL(&mux);
        float ang[CC_NUM_ORUGAS];
        bool  ok[CC_NUM_ORUGAS];
        for (int i = 0; i < CC_NUM_ORUGAS; i++) {
            ang[i] = anguloDeg[i];
            ok[i]  = encoderOk[i];
        }
        const float vIzq = velIzqReal;
        const float vDer = velDerReal;
        const bool  seg  = enSeguridad;
        const uint16_t err = bitsError;
        portEXIT_CRITICAL(&mux);

        for (int i = 0; i < CC_NUM_ORUGAS; i++) {
            const double elev = (double)encoderAElevacionRad(i, ang[i]);
            msg_estado.crawler_angle[i] = elev;
            msg_estado.encoder_ok[i]    = ok[i];
            msg_joints.position.data[i]      = elev;
        }
        msg_estado.track_speed_left  = (double)(vIzq * GRADOS_A_RAD);
        msg_estado.track_speed_right = (double)(vDer * GRADOS_A_RAD);
        msg_estado.can_ok         = canOk;
        msg_estado.imu_ok         = false;   /* esta variante no lleva IMU */
        msg_estado.safety_active  = seg;
        msg_estado.error_bits     = err;
        msg_estado.frames_ok      = 0;       /* sin protocolo serie propio */
        msg_estado.frames_crc_error = 0;

        const int64_t ns = rmw_uros_epoch_nanos();
        msg_estado.header.stamp.sec     = (int32_t)(ns / 1000000000LL);
        msg_estado.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
        msg_joints.header.stamp = msg_estado.header.stamp;

        RCNOCHECK(rcl_publish(&pub_estado, &msg_estado, NULL));
        RCNOCHECK(rcl_publish(&pub_joints, &msg_joints, NULL));

        tickExecutor = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
        vTaskDelayUntil(&ultimoDespertar, pdMS_TO_TICKS(PERIODO_ROS_MS));
    }
}

/* --- Watchdog (nucleo 0) --------------------------------------------- */

/* Segunda capa, independiente del PC: si el executor se queda colgado el
 * nodo esta muerto y nadie va a mandar la parada, asi que se reinicia. */
static void TaskWatchdog(void *arg) {
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(PERIODO_WDT_MS));
        const int64_t ahora = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
        if ((ahora - tickExecutor) > WDT_EXECUTOR_MS) {
            liberarTraccion();
            steppers_pararTodos();
            vTaskDelay(pdMS_TO_TICKS(20));
            esp_restart();
        }
    }
}

/* --- Reserva de memoria de los mensajes ------------------------------ */

/* micro-ROS no usa asignacion dinamica: las secuencias de JointState hay
 * que respaldarlas con memoria estatica antes de publicar. Los arrays
 * fijos de los mensajes propios ([4] en el .msg) van dentro del struct. */
static double  buf_joint_pos[CC_NUM_ORUGAS];
static rosidl_runtime_c__String buf_nombres[CC_NUM_ORUGAS];

static void prepararMensajes(void) {
    starcrawler_msgs__msg__RobotState__init(&msg_estado);

    sensor_msgs__msg__JointState__init(&msg_joints);
    msg_joints.position.data     = buf_joint_pos;
    msg_joints.position.size     = CC_NUM_ORUGAS;
    msg_joints.position.capacity = CC_NUM_ORUGAS;
    for (int i = 0; i < CC_NUM_ORUGAS; i++) {
        rosidl_runtime_c__String__init(&buf_nombres[i]);
        rosidl_runtime_c__String__assign(&buf_nombres[i], NOMBRE_JOINT[i]);
    }
    msg_joints.name.data     = buf_nombres;
    msg_joints.name.size     = CC_NUM_ORUGAS;
    msg_joints.name.capacity = CC_NUM_ORUGAS;

    starcrawler_msgs__msg__CrawlerCommand__init(&msg_crawler);

    geometry_msgs__msg__Twist__init(&msg_cmd_vel);
}

/* --- Transporte serie ------------------------------------------------ */

/* Las funciones del transporte de freertos_apps (microros_transports.c).
 * Se reutilizan tal cual salvo la apertura, que deja el UART a 115200. */
extern bool   esp32_serial_open(struct uxrCustomTransport *t);
extern bool   esp32_serial_close(struct uxrCustomTransport *t);
extern size_t esp32_serial_write(struct uxrCustomTransport *t,
                                 const uint8_t *buf, size_t len, uint8_t *err);
extern size_t esp32_serial_read(struct uxrCustomTransport *t, uint8_t *buf,
                                size_t len, int timeout, uint8_t *err);

static size_t puertoSerie = UART_NUM_0;

static bool abrirSerie(struct uxrCustomTransport *t) {
    if (!esp32_serial_open(t)) return false;
    return uart_set_baudrate((uart_port_t)*(size_t *)t->args,
                             SERIE_BAUDIOS) == ESP_OK;
}

/* --- Punto de entrada ------------------------------------------------ */

void appMain(void *argument) {
    (void)argument;

    /* Hardware primero y en estado seguro: si micro-ROS no llega a
     * levantar, el robot no puede quedarse con los motores excitados. */
    steppers_init();
    steppers_pararTodos();
    encoders_init();
    canOk = canbus_init();
    if (canOk) liberarTraccion();

    prepararMensajes();

    /* Sustituye al transporte que registra main.c: mismo UART, mas baudio */
    rmw_uros_set_custom_transport(true, (void *)&puertoSerie, abrirSerie,
                                  esp32_serial_close, esp32_serial_write,
                                  esp32_serial_read);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&soporte, 0, NULL, &allocator));
    RCNOCHECK(rmw_uros_sync_session(1000));
    RCCHECK(rclc_node_init_default(&nodo, "starcrawler_esp32", "", &soporte));

    /* Telemetria best-effort, como las suscripciones: sobre serie a
     * 115200 el stream fiable descartaba la mitad de los mensajes. */
    RCCHECK(rclc_publisher_init_best_effort(
        &pub_estado, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(starcrawler_msgs, msg, RobotState),
        "starcrawler/state"));

    RCCHECK(rclc_publisher_init_best_effort(
        &pub_joints, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states"));

    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;

    RCCHECK(rclc_subscription_init(
        &sub_cmd_vel, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel", &qos));

    RCCHECK(rclc_subscription_init(
        &sub_crawler, &nodo,
        ROSIDL_GET_MSG_TYPE_SUPPORT(starcrawler_msgs, msg, CrawlerCommand),
        "crawler/command", &qos));

    RCCHECK(rclc_executor_init(&executor, &soporte.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel,
            &msg_cmd_vel, &cb_cmd_vel, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_crawler,
            &msg_crawler, &cb_crawler, ON_NEW_DATA));

    tickExecutor = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    msCmdVel  = hw_millis();
    msCrawler = msCmdVel;

    xTaskCreatePinnedToCore(TaskWatchdog, "wdt",       2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(TaskMicroROS, "micro_ros", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(TaskControl,  "control",   8192, NULL, 5, NULL, 1);

    vTaskDelete(NULL);
}
