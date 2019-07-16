/*
 * my_i2c.h
 *
 *  Created on: 6 ���. 2017 �.
 *      Author: Kitaro
 */

#ifndef INCLUDE_DRIVER_MY_I2C_H_
#define INCLUDE_DRIVER_MY_I2C_H_

#include "user_config.h"
#include "ets_sys.h"
#include "osapi.h"

uint8 i2c_Do;
#define i2c_Busy 0b01000000 // Trans is Busy �������� ����� "���������� �����, ������ �� �������".
#define i2c_Free 0b10111111 // Trans is Free �������� ������ ����� ���������.

#define i2c_Err_msk 0b00110011 // ����� ���� ������
#define i2c_Err_NO 0b00000000 // All Right! -- ��� ����, �������� �������.
#define i2c_ERR_NA 0b00010000 // Device No Answer -- ����� �� ��������. �.�. ���� �����, ���� ��� ��� �� �����.
#define i2c_ERR_LP 0b00100000 // Low Priority -- ��� ����������� ����������� �������, ���� �� ��������� ��������
#define i2c_ERR_NK 0b00000010 // Received NACK. End Transmittion. -- ��� ������� NACK. ������ � ���.
#define i2c_ERR_BF 0b00000001 // BUS FAIL -- ������� ��������. � ���� ��� �������. ����� ����������� ������� ����������������� ����

#define i2c_global_MaxBuffer 48	//	�������� ������ ��� ��� ������, �� �������������� I2C
#define i2c_MaxBuffer i2c_global_MaxBuffer // �������� ������ Master ������ RX-TX. ������� �� ���� ����� ����� ������ �� ����� ������

#define i2c_sarp 0b00000000 // Start-Addr_R-Read-Stop ��� ����� �������� ������. �������� �� ������ ��� �� ������ � �������� ������
#define i2c_sawp 0b00000100 // Start-Addr_W-Write-Stop ��� ����� ������� ������. � ��� ����� � ������ � ������� ��������.
#define i2c_sawsarp 0b00001000 // Start-Addr_W-WrPageAdr-rStart-Addr_R-Read-Stop ��� ����� � ��������������� ������� �������� ������ ��������

#define MACRO_i2c_WhatDo_MasterOut (MasterOutFunc)(); // ������� ��� ������ ������. ���� ��� �������, �� ����� ���� ��� ������
#define MACRO_i2c_WhatDo_ErrorOut (ErrorOutFunc)();
#define i2c_packet_lenght 8


typedef void (*IIC_F)(void);	// ��� ��������� �� �������
extern IIC_F MasterOutFunc;	// ���������� � �������.
extern IIC_F ErrorOutFunc;
void DoNothing(void);

uint8 i2c_Buffer[i2c_MaxBuffer];
uint8 i2c_index;
extern uint8 i2c_ByteCount;


os_timer_t i2c_timer;


uint8 i2c_Write_Addr_Buffer(uint8 SAddr, uint8 Addr, uint8 Byte[], uint8 ByteNumbers, IIC_F WhatDo);
uint8 i2c_Write_Buffer(uint8 SAddr,  uint8 Byte[], uint8 ByteNumbers, IIC_F WhatDo);
uint8 i2c_Read_Addr_Buffer(uint8 SAddr, uint8 Addr, uint8 ByteNumbers, IIC_F WhatDo);	// ��������� ��� � ������ Byte[] �������� ByteNumbers
uint8 i2c_Read_Buffer(uint8 SAddr, uint8 ByteNumbers, IIC_F WhatDo);
uint8 i2c_Test_Write(uint8 SAddr);	// ��������, �� � ����� ����� �� ���

uint8 i2c_Read_Scan(uint8 SAddr_first, uint8 SAddr_last);

#endif /* INCLUDE_DRIVER_MY_I2C_H_ */
