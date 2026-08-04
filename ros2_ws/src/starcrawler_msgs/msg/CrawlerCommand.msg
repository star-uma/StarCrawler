# Consigna de las cuatro orugas articuladas.
# Orden de los vectores SIEMPRE: [FR, FL, RR, RL]
#
# Convencion de signo (identica al URDF): POSITIVO = brazo LEVANTADO.
# El driver traduce a los grados de encoder del firmware.

std_msgs/Header header

# Movimiento incremental por oruga: +1 sube, -1 baja, 0 quieta.
# Se ignora si use_position es true.
int8[4] increment

# Control de posicion en lazo cerrado (necesita encoder valido).
bool use_position
float64[4] target        # rad de elevacion; 0 = oruga horizontal

# Parada de emergencia: libera traccion y detiene elevacion.
# Tiene prioridad sobre todo lo demas.
bool emergency_stop
