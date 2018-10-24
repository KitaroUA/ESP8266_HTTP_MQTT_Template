#ifndef I2C_SHT3X_H
#define I2C_SHT3X_H


#include "user_interface.h"
#include <os_type.h>
#include "driver/my_i2c.h"
#include "math.h"
#include "user_config.h"



// extern uint8 i2c_DS1621_WriteByte(uint8 SAddr, uint8 Byte[], uint8 ByteNumbers, IIC_F WhatDo);
// extern uint8 i2c_SHT3X_ReadByte(uint8 SAddr, IIC_F WhatDo);


void StartWrite2SHT3X(void);
void Writed2SHT3X(void);
// void i2c_init_SHT3X (void);	//	��������� ����������� �������, ��� 4 �����������
#define i2c_SHT3X_Init_Restart 100
#define i2c_SHT3X_Init_Error_Restart 50

// void i2c_SHT3X_Init_stage1 (void);
// void i2c_SHT3X_Init_stage2 (void);
// void i2c_SHT3X_Inited_Stage1 (void);
// void i2c_SHT3X_Inited_Stage2 (void);
void i2c_SHT3X_Read(void);	//	��������� ������� ����������� � �������

#define i2c_SHT3X_First_Start 975	//	�������� ����� ������� �������
#define i2c_SHT3X_Read_Restart 100	//	�������� �� �������� ������, ���� ���� �������
#define i2c_SHT3X_Read_Reread 5000	//	�������� �� �������� �������, ������� �������, 10 ��� (5 ��� ��������, 5��� �����������, 5 ��� ��������....)
#define i2c_SHT3X_Rescan 10000	//	�������� �� �������� �������, ������� �������, 10 ��� (5 ��� ��������, 5��� �����������, 5 ��� ��������....)

//	�� ���� 
#define i2c_Buffer_Lenght 3
uint8 i2c_send_buffer[i2c_Buffer_Lenght];
//extern uint8 i2c_SHT3X_Temp[2];	//	����� � ��� ���� ������� ���� ������ �����������
//uint8 i2c_SHT3X_count;	// ʳ������ ������ SHT3X
//uint8 i2c_SHT3X_Addresses[8];	// ����� ������ ������ SHT3X, ����������� ���� ���� 8 ������

uint8 i2c_SHT3X_Flags;	// 0bxxx1xxxxx - ���������� ����������� ������ ������
							// 0bx1xxxxxxx - ��� �������
#define i2c_SHT3X_Address_Mask 0b00001111
#define i2c_SHT3X_Init_Mask 0b00100000
#define i2c_SHT3X_Read_Complete 0b01000000
#define i2c_SHT3X_Phase 0b00000001	//	���� ������� ������� � ������.





#define i2c_SHT3X_Address 0x8A
#define i2c_SHT3X_Command_MSB 0x24
#define i2c_SHT3X_Command_LSB 0x00

void i2c_SHT3X_Start_Measuring (void);
void i2c_SHT3X_Start_Measuring_End (void);


void i2c_SHT3X_Start_Reading (void);
void i2c_SHT3X_Readed (void);


void i2c_SHT3X_Test(void);

float i2c_SHT3X_Temp;
float i2c_SHT3X_Hum;

uint8 i2c_SHT3X_errors;	//	��� ������ ��������� ������� � ��������; ����� ��� ��� - �������� ������� i2c_ERR_NA, ��������� �� - ��������� - ������� ��������, ��� ��������.
#define i2c_SHT3X_errors_NA	0b00001000

os_timer_t SHT3X_timer_read;
os_timer_t SHT3X_timer_write;



#endif
