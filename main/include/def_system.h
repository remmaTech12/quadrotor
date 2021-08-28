// PIN allccation
#define MOTOR_PWM1 12
#define MOTOR_PWM2 13
#define MOTOR_PWM3 14
#define MOTOR_PWM4 15

#define MOTOR_DIR1 18
#define MOTOR_DIR2 19
#define MOTOR_DIR3 32
#define MOTOR_DIR4 33

#define BUILTIN_LED 2

#define EMERGENCY_SWITCH 4


// System setting
#define SAMPLING_TIME_MS 14
#define RECEIVE_DATA_SIZE 7
#define PID_MAX 2000
#define LIMIT_MOTOR 255

// Bias removal time
#define IMU_CNT_START_NUM 100
#define IMU_CNT_TOTAL_NUM 100
#define CTL_CNT_START_NUM 150
#define CTL_CNT_TOTAL_NUM 100

// Debug setting
//#define DEBUG_RECV_SWITCH
//#define DEBUG_RECV_JOYSTICK
//#define DEBUG_PID_COMMAND
//#define DEBUG_RECV_COMMAND
//#define DEBUG_MOTOR_COMMAND