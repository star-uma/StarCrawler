# Estado completo del robot, publicado por starcrawler_driver a partir de la
# telemetria del ESP32. Orden de los vectores SIEMPRE: [FR, FL, RR, RL]

std_msgs/Header header

# Angulo de elevacion de cada oruga (rad, positivo = brazo levantado).
# Solo es valido donde encoder_ok es true.
float64[4] crawler_angle
bool[4] encoder_ok

# Velocidad de cada tren de traccion (rad/s en el eje de salida del RMD).
float64 track_speed_left
float64 track_speed_right

# Diagnostico del hardware de bajo nivel
bool can_ok
bool imu_ok              # reservado: el firmware ROS 2 no usa IMU
bool safety_active       # true = el ESP32 esta en estado seguro

# Bits de error tal cual los reporta el firmware (ver control_core.h):
# 0..3 encoder FR/FL/RR/RL, 4 IMU, 5 CAN, 6 watchdog
uint16 error_bits

# Salud del enlace serie PC <-> ESP32
uint32 frames_ok
uint32 frames_crc_error
