#include <iostream>

/*              STATUS 					*/

#define POWER_ON        0x0001
#define MOVING          0x0002
#define FAULT           0x0004
#define EMG_STOP        0x0008
#define POSITION_CTL    0x0010
#define VELOCITY_CTL    0x0020
#define CURRENT_CTL     0x0040
#define CALIBRATION     0x0080
#define AI_NOT_CENTER   0x0100
#define PI_NOT_CENTER   0x0200
#define AI_LOSS         0x0400
#define PI_LOSS         0x0800
#define DI_STOP         0x1000
#define AT_LIMIT        0x2000
#define SERIAL_GO       0x4000
#define ANALOG_GO       0x8000


/*              VI_FAULT 			    */

#define NO_FAULT          0x0000
#define OVER_CURRENT      0x0001
#define OVER_VOLTAGE      0x0002
#define UNDER_VOLTAGE     0x0004
#define OVER_TEMPERATURE  0x0008
#define SHORT_CIRCUIT     0x0010
#define STALL_DETECT      0x0020
#define VEL_ERROR_DETECT  0x0040
#define POS_ERROR_DETECT  0x0080

enum direction
{
    Not_Change,
    Reverse_Direction
};

enum Comaand_select
{
    _position,
    _velocity,
    _current,
    _voltage
};
enum Motor_select
{
    channel_1,
    channel_2
};
enum Motor_function
{
    ON,
    OFF,
    CLEAR_FAULT,
    SLOW_STOP,
    QUICK_STOP
};
enum system_fucntion
{
    SAVE_CONFIG_FLASH,
    LOAD_CONFIG_FLASH,
    FACTORY_DEFAULT,
    RESET_CONTROLLE
};
enum dual_m_command_select
{
    mp,
    position,
    velocity,
    current,
    voltage,
    m_lav
};

struct MotorControlMonitoring
{
    float battery_voltage;
    float battery_current;
    float temperature;

    long Status[2];
    long Fault[2];
};

struct Motor_Configuration_value
{
    // 위치제어 범위
    long min_position[2];
    long max_position[2];

    // 모터사양 참고
    long encoder_ppr[2];   // (ppr)
    float Max_current[2];  // (A)
    float Max_voltage[2];  // (V)
    float Max_velocity[2]; // (RPM)
    float Acceleration[2];  // (RPM)
    float Deceleration[2]; // (RPM)

    // 제어기 안전
    float overvoltage_limit[2];  // (V)
    float undervoltage_limit[2]; // (V)
    float overcurrent_limit[2];  // (A)

    // 하드웨어 참고
    long direction[2];
    float velocity_p_gain[2];
    float velocity_i_gain[2];
};

struct MotorControlProductInfomaions
{
    long device_id;
    long prouct_id;
    float software_version;
    float hardware_version;
    long RS232_Baudrate;
    long USB_Baudrate;
    long CAN_Bitlate;
};

struct MonitoringCommand_value
{
    long position[2];
    float current[2];
    float velocity[2];
    float voltage[2];
};

struct dual_m_command_value
{
    long position[2];
    float current[2];
};

struct Differential_Mobile_Robot_value
{
    float wheel_radius; //이동로봇의 바퀴 반지름 (m)
    float axle_length;  //이동로봇의 좌우 바퀴간 거리 (m)
    float gear_ratio;   //모터와 바퀴간 감속비율 . 모터 회전수 / 바퀴회전수
};

extern struct Motor_Configuration_value MyMotorConfiguration;
extern struct MonitoringCommand_value MyMotorCommandReadValue;
extern struct dual_m_command_value MyMotorDualReadValue;
extern struct MotorControlProductInfomaions ProductInfomaions;
extern struct MotorControlMonitoring MyMotorControl;
extern struct Differential_Mobile_Robot_value Differential_MobileRobot;