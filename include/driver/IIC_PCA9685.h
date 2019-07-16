#ifndef I2C_PCA9685_H
#define I2C_PCA9685_H

#include "user_interface.h"
#include <os_type.h>
#include "driver/my_i2c.h"
#include "math.h"
#include "user_config.h"

os_timer_t PCA9685_timer;

void i2c_PCA9685_Init(void);
#define i2c_PCA9685_Address 0x82
#define PCA9685_Data_Conveyor_Lenght 32
#define PCA9685_First_Address 0x80
#define PCA9685_First_LED_Address 0x06

#define i2c_PCA9685_Conveyor_Task_Work_Delay 2
#define i2c_PCA9685_Conveyor_Task_Idle_Delay 20

uint8 i2c_PCA9685_CBF; 				// ����� ���������� ������,
uint8 i2c_PCA9685_Init_State;			// �� ���� ����������� PCA9685 - ����� ����� �����������.

#define i2c_PCA9685_CBF_Start 7		// ���� 1 - �������� ������ � PCA9685 �������
#define i2c_PCA9685_CBF_Empty 6		// ���� - ����� ����, i2c_PCA9685_CBRP=i2c_PCA9685_CBWP
#define i2c_PCA9685_CBF_Full 5		// ���� - ����� �����, i2c_PCA9685_CBRP=i2c_PCA9685_CBWP, ������
#define i2c_PCA9685_Initialized 4


#define i2c_PCA9685_CBL 32		// ����� ���������� ������ - ����� ������ + ����� �����.
uint8 i2c_PCA9685_CBD[i2c_PCA9685_CBL][6];
//	0 - PCF address offset from 0x80 (0x80 + PCA9685_Data_Conveyor [][0]*2
//	1 - LED Address offset from 0x06 (0x06 + PCA9685_Data_Conveyor [][1]*4
//	2,3,4,5 - LED0_ON_L, LED0_ON_H, LED0_OFF_L, LED0_OFF_H
uint8 i2c_PCA9685_CBRP;				// ����������, ���������� ������ ������ �� ���������� ������, �������� �� PCA9685, ��������� �� !!_���������_!! ������
uint8 i2c_PCA9685_CBWP;				// ����������, ���������� ������ ������ � ��������� �����




void ICACHE_FLASH_ATTR i2c_PCA9685_Conveyor_Task (void);
void  ICACHE_FLASH_ATTR i2c_PCA9685_Push_CB (uint8 AddrOffset, uint8 LED_N, uint8 LED0_ON_L, uint8 LED0_ON_H, uint8 LED0_OFF_L, uint8 LED0_OFF_H);
void  ICACHE_FLASH_ATTR i2c_PCA9685_POP_CB(void);


#define Servo_1ms 205
#define Servo_2ms 410
void  ICACHE_FLASH_ATTR i2c_PCA9685_Servo_Simple(uint8 AddrOffset, uint8 LED_N, uint8 Position);









#endif
