/*
 * my_i2c.c
 *
 *  Created on: 6 ���. 2017 �.
 *      Author: Kitaro
 */

#include "driver/my_i2c.h"
#include "stdlib.h"

uint8 i2c_Buffer[i2c_MaxBuffer];
IIC_F MasterOutFunc = &DoNothing;
IIC_F ErrorOutFunc = &DoNothing;	// � ���������� ������ � ������ Master

uint8 i2c_ByteCount;

void ICACHE_FLASH_ATTR DoNothing(void)
{

}
void i2c_send (void);
void i2c_recive (void);




uint8 i2c_pointer; //	��������� ������� �� 8 ����, �� ��������� �����, ��� ������������ ��� � �����.

uint8  ICACHE_FLASH_ATTR i2c_Write_Addr_Buffer(uint8 SAddr, uint8 Addr, uint8 Byte[], uint8 ByteNumbers, IIC_F WhatDo)	// ��������� ��� � ������ Byte[] �������� ByteNumbers
//	�������� � ������� SAddr �� �������� ������ Addr
{
uint8 i;
//	�������� ��������  �� ��������� ��



if (i2c_Do & i2c_Busy )	return 0; //	���� �������, ������� �����.
i2c_Do |= i2c_Busy; //	������������ ��������� - ���� �������


MasterOutFunc = WhatDo;
ErrorOutFunc = WhatDo;

i2c_ByteCount = ByteNumbers;

for (i=0;i<ByteNumbers;i++)
{
	i2c_Buffer[i]=Byte[i];
}	//	���������� �����, ���� ������ ����� � i2c

//	�������� �������� �����, ��������� �������, ������ �������� ������
i2c_start();
i2c_writeByte(SAddr);
if (!i2c_check_ack()) {
  os_printf("0x%x-%s-%s slave not ack... return \r\n", SAddr, __FILE__, __func__);
  i2c_stop();
  i2c_Do &= i2c_Free;			// ����������� ����
  return(0);
}
i2c_writeByte(Addr);
if (!i2c_check_ack()) {
  //os_printf("-%s-%s slave not ack... return \r\n", __FILE__, __func__);
  i2c_stop();
  i2c_Do &= i2c_Free;			// ����������� ����
  return(0);
}
//	���� ������� ����������, �� ����� ����� ���, ��������� ���������, ��� ���� ����� ������� �� 8 ����



i2c_pointer = 0;
os_timer_disarm (&i2c_timer);
os_timer_setfn (&i2c_timer, (os_timer_func_t *)i2c_send, NULL);
os_timer_arm(&i2c_timer, 1, 0);




return 1;
}




uint8  ICACHE_FLASH_ATTR i2c_Write_Buffer(uint8 SAddr,  uint8 Byte[], uint8 ByteNumbers, IIC_F WhatDo)	// ��������� ��� � ������ Byte[] �������� ByteNumbers
//	�������� � ������� SAddr �� �������� ������ Addr
{
uint8 i;
//	�������� ��������  �� ��������� ��




if (i2c_Do & i2c_Busy )	return 0; //	���� �������, ������� �����.
i2c_Do |= i2c_Busy; //	������������ ��������� - ���� �������


MasterOutFunc = WhatDo;
ErrorOutFunc = WhatDo;

i2c_ByteCount = ByteNumbers;

for (i=0;i<ByteNumbers;i++)
{
	i2c_Buffer[i]=Byte[i];
}	//	���������� �����, ���� ������ ����� � i2c

//	�������� �������� �����, ��������� �������, ������ �������� ������
i2c_start();
i2c_writeByte(SAddr);
if (!i2c_check_ack()) {
//  os_printf("Addr:%X-%s-%s slave not ack... return \r\n", SAddr, __FILE__, __func__);
  i2c_stop();
  i2c_Do &= i2c_Free;			// ����������� ����
  return(0);
}

//	���� ������� ����������, �� ����� ����� ���, ��������� ���������, ��� ���� ����� ������� �� 8 ����



i2c_pointer = 0;

os_timer_disarm (&i2c_timer);
os_timer_setfn (&i2c_timer, (os_timer_func_t *)i2c_send, NULL);
/*os_timer_arm(&i2c_timer, 1, 0);*/
i2c_send();
return 1;
}
















void  ICACHE_FLASH_ATTR i2c_send (void)
{
uint8 i;
for (i = 0; i < i2c_packet_lenght; i++)
	{
		if (i2c_pointer < i2c_ByteCount)	//	���� � ��� � �����
		{
			i2c_writeByte(i2c_Buffer[i2c_pointer]);
			if (!i2c_check_ack())
			{
				//os_printf("-%s-%s slave not ack... return \r\n", __FILE__, __func__);
				i2c_stop();
			    MACRO_i2c_WhatDo_ErrorOut // ������������ ������� ������;

			}
			i2c_pointer++;
		}
		else	//	����� ��������� ����� :)
		{
/*			i2c_stop();
			LogC("send end 1\r\n");
			MACRO_i2c_WhatDo_MasterOut; */
			break;

		}
	}


	if (i2c_pointer < i2c_ByteCount)	//	���� ��� �� � ��� � ����� - �� ������� ������ �� ������
	{
		os_timer_arm(&i2c_timer, 1, 0);
	}
	if (i2c_pointer == i2c_ByteCount)// ���� � - �� �������� �����, �������
	{
		i2c_stop();
		MACRO_i2c_WhatDo_MasterOut;
	}



}









uint8  ICACHE_FLASH_ATTR i2c_Read_Addr_Buffer(uint8 SAddr, uint8 Addr, uint8 ByteNumbers, IIC_F WhatDo)	// ��������� ��� � ������ Byte[] �������� ByteNumbers
{
	if (i2c_Do & i2c_Busy)
		{
//		INFO ("\r\n i2c busy \r\n");
		return 0;
		}

	i2c_Do |= i2c_Busy; //	������������ ��������� - ���� �������

	MasterOutFunc = WhatDo;
	ErrorOutFunc = WhatDo;

	i2c_ByteCount = ByteNumbers;

	i2c_pointer = 0;

	i2c_start();
	i2c_writeByte(SAddr);
	if (!i2c_check_ack()) {
//	  os_printf("-%s-%s slave not ack... return \r\n", __FILE__, __func__);
	  i2c_stop();
	  return(0);
	}
	i2c_writeByte(Addr);
	if (!i2c_check_ack()) {
//	  os_printf("-%s-%s slave not ack... return \r\n", __FILE__, __func__);
	  i2c_stop();
	  return(0);
	}
	i2c_stop();
	i2c_start();
	i2c_writeByte(SAddr+1);
	if (!i2c_check_ack()) {
//	  os_printf("-%s-%s slave not ack... return \r\n", __FILE__, __func__);
	  i2c_stop();
	  return(0);
	}








	os_timer_disarm (&i2c_timer);
	os_timer_setfn (&i2c_timer, (os_timer_func_t *)i2c_recive, NULL);
	os_timer_arm(&i2c_timer, 1, 0);


	return 1;
}



uint8  ICACHE_FLASH_ATTR i2c_Read_Buffer(uint8 SAddr, uint8 ByteNumbers, IIC_F WhatDo)	// ��������� ��� � ������ Byte[] �������� ByteNumbers
{
	if (i2c_Do & i2c_Busy) return 0;

	i2c_Do |= i2c_Busy; //	������������ ��������� - ���� �������

	MasterOutFunc = WhatDo;
	ErrorOutFunc = WhatDo;

	i2c_ByteCount = ByteNumbers;

	i2c_pointer = 0;

	i2c_start();
	i2c_writeByte(SAddr+1);
	if (!i2c_check_ack()) {
	  os_printf("-%s-%s slave not ack... return \r\n", __FILE__, __func__);
	  i2c_stop();
	  return(0);
	}

	os_timer_disarm (&i2c_timer);
	os_timer_setfn (&i2c_timer, (os_timer_func_t *)i2c_recive, NULL);
	os_timer_arm(&i2c_timer, 1, 0);


	return 1;
}



void  ICACHE_FLASH_ATTR i2c_recive (void)
{
	uint8 i;
	for (i = 0; i < i2c_packet_lenght; i++)
	{
		if (i2c_pointer < i2c_ByteCount)	//	���� � ��� � �����
		{
			i2c_Buffer[i2c_pointer] = i2c_readByte();
			if (i2c_pointer == i2c_ByteCount-1)
			{
				i2c_send_ack(0);
			}
			else
			{
				i2c_send_ack(1);
			}
			i2c_pointer++;
		}
		else	//	����� ��������� ����� :)
		{
			i2c_stop();
			MACRO_i2c_WhatDo_MasterOut;
			break;
		}
	}
	if (i2c_pointer < i2c_ByteCount)	//	���� ��� �� � ��� � ����� - �� ������� ������ �� ������
	{
		os_timer_arm(&i2c_timer, 1, 0);
	}
	if (i2c_pointer == i2c_ByteCount)	// ���� � - �� �������� �����, �������
	{
		i2c_stop();
		MACRO_i2c_WhatDo_MasterOut;
	}



}




uint8  ICACHE_FLASH_ATTR i2c_Test_Write(uint8 SAddr)	// ��������, �� � ����� ����� �� ���
{
	if (i2c_Do & i2c_Busy) return 0;

	i2c_Do |= i2c_Busy; //	������������ ��������� - ���� �������

	i2c_start();
	i2c_writeByte(SAddr);
	if (!i2c_check_ack()) {
	  os_printf("-%s-%s slave not ack... return \r\n", __FILE__, __func__);
	  i2c_stop();
	  return(255);
	}
	i2c_stop();
	return 1;
}








uint8  ICACHE_FLASH_ATTR i2c_Read_Scan(uint8 SAddr_first, uint8 SAddr_last)	// ��������� ��� � ������ Byte[] �������� ByteNumbers
{

	if (i2c_Do & i2c_Busy) return 0;

	i2c_Do |= i2c_Busy; //	������������ ��������� - ���� �������

	INFO("\r\n WiFi Scan \r\n");

	i2c_pointer = 0;

	if (SAddr_first > SAddr_last) return 0;
	uint8 SAddr;
	SAddr = SAddr_first;
	do 	{
		i2c_start();
		i2c_writeByte(SAddr+1);
		if (!i2c_check_ack())
			{
			INFO("Addr:%X-%s-%s slave not ack... return \r\n", SAddr, __FILE__, __func__);

//			return(0);
			}
		else
			{
			INFO("\r\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Found Addr:%X \r\n", SAddr);
			}
		SAddr = SAddr >> 1;
		SAddr++;
		SAddr = SAddr << 1;
		i2c_stop();
		} while (SAddr <= SAddr_last);

	i2c_Do &= i2c_Free;
	return 1;
}






