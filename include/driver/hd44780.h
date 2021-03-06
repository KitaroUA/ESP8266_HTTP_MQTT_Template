/*
 * hd44780.h
 *
 *  Created on: 8 ���. 2017 �.
 *      Author: Kitaro
 */

#ifndef INCLUDE_DRIVER_HD44780_H_
#define INCLUDE_DRIVER_HD44780_H_

#include "user_config.h"
#include "ets_sys.h"
#include "osapi.h"

#define i2c_LCD_Columns 20
#define i2c_LCD_Rows 4

uint8 i2c_LCD_CBF; 				// ����� ���������� ������,
uint8 i2c_LCD_Init_State;			// �� ���� ������������ �ʲ - ����� ����� ������������.

#define i2c_LCD_CBF_Start 7		// ���� 1 - �������� ������ � ��� �������
#define i2c_LCD_CBF_Empty 6		// ���� - ����� ����, i2c_LCD_CBRP=i2c_LCD_CBWP
#define i2c_LCD_CBF_Full 5		// ���� - ����� �����, i2c_LCD_CBRP=i2c_LCD_CBWP, ������
#define i2c_LCD_Initialized 4


#define i2c_LCD_CBL 150		// ����� ���������� ������ - ����� ������ + ����� �����.
uint8 i2c_LCD_CBD[i2c_LCD_CBL];
uint8 i2c_LCD_CBRP;				// ����������, ���������� ������ ������ �� ���������� ������, �������� �� ���, ��������� �� !!_���������_!! ������
uint8 i2c_LCD_CBWP;				// ����������, ���������� ������ ������ � ��������� �����

#define i2c_LCD_CBComm 0x0F		// ���������, �����������, ��� ��������� ���� � ������ - ��������
#define i2c_LCD_CBData 0x1F		// ���������, �����������, ��� ��������� ���� � ������ - ������

// ������ ������ � ������ ���������. ��� ������ ������ ������ ����� +1 �� ������� ������
// �� ������ ���� � ������ �� �������� ��� ���� ���������� � ������ ���� ��������.
uint8 i2c_LCD_CBExch;				// ��������������� ���������� ��� ������ ������� ����� �����������
uint8 i2c_LCD_CBData_Out;			// ���������� ��� ������ � ����
//uint8 i2c_LCD_CB_CGRAM_Counter;    // ������� ��� ��������� ������ �������� � ���

//#define i2c_LCD_CB_CGRAM_Count 4		// ���-�� ��������, ������� ���� ������� � ���



// ����������� � ���������


#define i2c_LCD_RS 0
#define i2c_LCD_RW 1
#define i2c_LCD_E 2
#define i2c_LCD_Light 3
#define i2c_LCD_Data_Mask 0b00001111	//	����� ��� ��������� �����, ���� ��� �� "0"
#define i2c_LCD_Port_Offset 4	// ���� �� ����� �� �������� ����, ���� ���������� ����� ��������� � �������� ���� - ����� 0




char ICACHE_FLASH_ATTR i2c_LCD_Code_converter(char buffer);	//	���������� ���� �������� � ����, ��������� ��� ���������.
static void ICACHE_FLASH_ATTR i2c_POP_LCD_CB(void);					//	������� � ��, ����������� ����� � ��������� ������ �� �����.
static void ICACHE_FLASH_ATTR i2c_LCD_Code_display(uint8 row, uint8 col, char buffer);	//������� ��������� - ���������� ������ � ���� ���, ������ ���� - ���������, �� �������� ���������, ������
static void ICACHE_FLASH_ATTR i2c_LCD_Write_Task (void);		//	�������� ���������, ������ ��� RTOS, �������� ��������� ����� � ��, ������� ��� ������ � ���������, ���������� ���� ���� ��� �� ����������
//void  ICACHE_FLASH_ATTR i2c_LCD_Send_byte (uint8 buffer, uint8 flag);
static void  ICACHE_FLASH_ATTR i2c_LCD_Send_byte (void);
static void  ICACHE_FLASH_ATTR i2c_LCD_byte_sent (void);
//====================================================================================================
#define i2c_LCD_Write_Task_Idle	50			//	��������� ��� �������� ���������, ��� ���������� ��������, ���� �� ��������
#define i2c_LCD_Write_Task_Work 5			//	��� ���������� ������ ��������, ���� � �� � ����.
//====================================================================================================

void ICACHE_FLASH_ATTR i2c_LCD_init(void);				//	��������� ������������ ���������.
//====================================================================================================
#define i2c_LCD_init_Preinit 300			//	�������� ����� �������� ������������
#define i2c_LCD_init_Delay 50				//	�������� �� ������� ������������
//====================================================================================================

void ICACHE_FLASH_ATTR i2c_LCD_Send_String(uint8 row, uint8 col, char str_out[]);
#define i2c_LCD_Send_String_FirstRow	0				//	������ � ���� � ���`�� ����������� �����.
#define i2c_LCD_Send_String_SecondRow	0x40
#define i2c_LCD_Send_String_ThirdRow	0x14
#define i2c_LCD_Send_String_FourthRow	0x54
#define i2c_LCD_Send_String_Set_DDRAM_Address	0x80	//	��� ���� �����, �� � ����� ����� ��� ������ � ���`��, ���� ������
uint8 LCD2PCF_send_buffer [4];

uint8 i2c_LCD_light_control;


os_timer_t hd44780_timer;



#endif /* INCLUDE_DRIVER_HD44780_H_ */
