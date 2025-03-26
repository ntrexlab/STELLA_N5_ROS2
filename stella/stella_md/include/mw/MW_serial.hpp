#include <iostream>

int MW_Serial_Connect(char *portname_, uint32_t baudrate_);
int MW_Serial_DisConnect();

int ControllerMonitoring();
int GetProductInformations();

int Status();
int Fault();

int Motor_MonitoringCommand(int channel_sel, int select);           // channel_sel = channel_1, channel_2 / select = _position, _velocity, _current, _voltage
int Motor_MoveCommand(int channel_sel, int select, float value);    // channel_sel = channel_1, channel_2 / select = _position, _velocity, _current, _voltage

int FunctionMotorControl(int channel_sel, int value);               // value = POWER_ON, POWER__OFF, CLEAR_FAULT, SLOW_STOP, QUICK_STOP};
int FunctionMotorControlSystem(int value);                          // value = SAVE_CONFIG_FLASH, LOAD_CONFIG_FLASH, FACTORY_DEFAULT, RESET_CONTROLLE};

int dual_m_command(int select, float Motor1, float Motor2);         //select = _position, _velocity, _current, _voltage / 

int Get_MotorConfiguration(int channel_sel);
int Set_MotorConfiguration(int channel_sel, struct Motor_Configuration_value *SetValue);
int Get_Differential_MobileRobot_value();
int Set_Differential_MobileRobot_value(struct Differential_Mobile_Robot_value *SetValue);
void InformationsPrintf();