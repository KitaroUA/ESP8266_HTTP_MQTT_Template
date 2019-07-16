/*
 * IIC_PCA9685.c
 *
 *  Created on: 22 ���. 2017 �.
 *      Author: Kitaro
 */

#include "driver/IIC_PCA9685.h"

#define HI(X) (X>>8)
#define LO(X) (X & 0xFF)






void ICACHE_FLASH_ATTR i2c_PCA9685_Test_End(void)
{
	i2c_Do &= i2c_Free;
}


void ICACHE_FLASH_ATTR i2c_PCA9685_Test(void)
{
	uint8 ii[4];
	uint16 i16on, i16off;
	//110 - 0.54 ms
	//205 - 1ms
	//410 - 2ms
	//480 - 2.35 ms
	i16off = 308;
	i16on = 4095 - i16off;
	ii[0] = LO(i16on);
	ii[1] = HI(i16on);
//	ii[2] = LO(i16off);
//	ii[3] = HI(i16off);
	ii[2] = LO(4095);
	ii[3] = HI(4095);
	if (!i2c_Write_Addr_Buffer (i2c_PCA9685_Address, 0x06, ii, 4, i2c_PCA9685_Test_End))
	{
		SetTimerTask(PCA9685_timer,i2c_PCA9685_Test, 1, 0);
	}
}


void ICACHE_FLASH_ATTR i2c_PCA9685_Init_End(void)
{
i2c_Do &= i2c_Free;
if(i2c_Do & (i2c_ERR_NA|i2c_ERR_BF))	// ���� ������ �� �������
{

}
SetTimerTask(PCA9685_timer,i2c_PCA9685_Conveyor_Task, 1, 0);
}

void ICACHE_FLASH_ATTR i2c_PCA9685_Init2(void)
{
i2c_Do &= i2c_Free;
uint8 ii[1];
ii[0]=0b00100001;
if (!i2c_Write_Addr_Buffer (i2c_PCA9685_Address, 0x00, ii, 1, i2c_PCA9685_Init_End))
{
	SetTimerTask(PCA9685_timer,i2c_PCA9685_Init2, 1, 0);
}
}



void ICACHE_FLASH_ATTR i2c_PCA9685_Init1(void)
{
i2c_Do &= i2c_Free;
uint8 ii[1];
ii[0]=126;
if (!i2c_Write_Addr_Buffer (i2c_PCA9685_Address, 0xFE, ii, 1, i2c_PCA9685_Init2))
{
	SetTimerTask(PCA9685_timer,i2c_PCA9685_Init1, 1, 0);
}
}


void ICACHE_FLASH_ATTR i2c_PCA9685_Init(void)
{
	i2c_PCA9685_CBF |= (1<<i2c_PCA9685_CBF_Empty);              // ������������ ����� "�������� ����� ������"
	i2c_PCA9685_CBRP = 0;									// �������� ������� ������� �� ������
	i2c_PCA9685_CBWP = 0;
	i2c_PCA9685_CBF |= (1<<i2c_PCA9685_Initialized);

uint8 ii[1];
ii[0]=0b00110001;
if (!i2c_Write_Addr_Buffer (i2c_PCA9685_Address, 0x00, ii, 1, i2c_PCA9685_Init1))
{
	SetTimerTask(PCA9685_timer,i2c_PCA9685_Init, 1, 0);
}
}






























































void ICACHE_FLASH_ATTR i2c_PCA9685_Conveyor_Task (void)	//	�������� ���������, ������ ��� RTOS, �������� �������� ����� � ��, ������� ��� ������ � ���������, ���������� ���� ���� ��� �� ����������
{

	if (((i2c_PCA9685_CBF)& ((1<<i2c_PCA9685_CBF_Start)|(1<<i2c_PCA9685_Initialized))) == ((1<<i2c_PCA9685_CBF_Start)|(1<<i2c_PCA9685_Initialized))) // ���� �������� ����� �������
	{
		i2c_PCA9685_POP_CB();
//		SetTimerTask(PCA9685_timer,i2c_PCA9685_Conveyor_Task,i2c_PCA9685_Conveyor_Task_Work_Delay,0);
	}
	else
	{
		SetTimerTask(PCA9685_timer,i2c_PCA9685_Conveyor_Task,i2c_PCA9685_Conveyor_Task_Idle_Delay,0);
	}
}











void  ICACHE_FLASH_ATTR i2c_PCA9685_Push_CB (uint8 AddrOffset, uint8 LED_N, uint8 LED0_ON_L, uint8 LED0_ON_H, uint8 LED0_OFF_L, uint8 LED0_OFF_H)
{
if ((i2c_PCA9685_CBWP!=i2c_PCA9685_CBRP)||(i2c_PCA9685_CBF&(1<<i2c_PCA9685_CBF_Empty)))
	{
		i2c_PCA9685_CBF &= ~(1<<i2c_PCA9685_CBF_Empty);				//	������� ���� - �� �������
		i2c_PCA9685_CBD[i2c_PCA9685_CBWP][0]=AddrOffset;
		i2c_PCA9685_CBD[i2c_PCA9685_CBWP][1]=LED_N;
		i2c_PCA9685_CBD[i2c_PCA9685_CBWP][2]=LED0_ON_L;
		i2c_PCA9685_CBD[i2c_PCA9685_CBWP][3]=LED0_ON_H;
		i2c_PCA9685_CBD[i2c_PCA9685_CBWP][4]=LED0_OFF_L;
		i2c_PCA9685_CBD[i2c_PCA9685_CBWP][5]=LED0_OFF_H;
		i2c_PCA9685_CBWP ++;
		if (i2c_PCA9685_CBWP == i2c_PCA9685_CBL) {i2c_PCA9685_CBWP=0;}	//	�������� � "0" ������� �������, ���� ������� ���� ��
	}
else
	{
			i2c_PCA9685_CBF |= (1<<i2c_PCA9685_CBF_Full); // ������ ���� - ����� �����
			//cli(); //; ��� ����� ����� ���������, ������� ������� �� ������ ������������ ���������� ������, ������ �������� ���������
			uint8 i;
			os_printf("��� ����� ����� ���������, ������� ������� �� ������ ������������ ���������� ������ PCA9685\r\n");
			while (1)
			{
				i++;
			}
//		}
	}
		i2c_PCA9685_CBF |= (1<<i2c_PCA9685_CBF_Start);							//	������� ����, �� � �� � ���.

}


void  ICACHE_FLASH_ATTR i2c_PCA9685_POP_CB_writed(void)
{
	i2c_Do &= i2c_Free;

	if(i2c_Do & (i2c_ERR_NA|i2c_ERR_BF))	// ���� ������ �� �������
	{
		SetTimerTask(PCA9685_timer,i2c_PCA9685_Conveyor_Task,i2c_PCA9685_Conveyor_Task_Work_Delay,0);
	}
	else
	{
		i2c_PCA9685_CBRP ++;
		if (i2c_PCA9685_CBRP == i2c_PCA9685_CBL) {i2c_PCA9685_CBRP=0;}	//	�������� � "0" ������� �������, ���� ������� ���� ��



	if (i2c_PCA9685_CBWP == i2c_PCA9685_CBRP)
	{
		i2c_PCA9685_CBF &=(~(1<<i2c_PCA9685_CBF_Start)); // ����� ��������� - �������� �������
		i2c_PCA9685_CBF |=(1<<i2c_PCA9685_CBF_Empty);	 // ������ ��������� - ����� �������
	}

		SetTimerTask(PCA9685_timer,i2c_PCA9685_Conveyor_Task,i2c_PCA9685_Conveyor_Task_Work_Delay,0);
	}


}

void  ICACHE_FLASH_ATTR i2c_PCA9685_POP_CB(void)
{
	uint8 buffer[4];
	buffer[0] = i2c_PCA9685_CBD[i2c_PCA9685_CBRP][2];
	buffer[1] = i2c_PCA9685_CBD[i2c_PCA9685_CBRP][3];
	buffer[2] = i2c_PCA9685_CBD[i2c_PCA9685_CBRP][4];
	buffer[3] = i2c_PCA9685_CBD[i2c_PCA9685_CBRP][5];

				if(!i2c_Write_Addr_Buffer  (PCA9685_First_Address + i2c_PCA9685_CBD[i2c_PCA9685_CBRP][0]*2, \
											PCA9685_First_LED_Address + i2c_PCA9685_CBD[i2c_PCA9685_CBRP][1]*4, \
											buffer, 4, i2c_PCA9685_POP_CB_writed) )
				{
					SetTimerTask(PCA9685_timer,i2c_PCA9685_Conveyor_Task,i2c_PCA9685_Conveyor_Task_Work_Delay,0);
				}

/*
				i2c_PCA9685_CBRP ++;
				if (i2c_PCA9685_CBRP == i2c_PCA9685_CBL) {i2c_PCA9685_CBRP=0;}	//	�������� � "0" ������� �������, ���� ������� ���� ��



			if (i2c_PCA9685_CBWP == i2c_PCA9685_CBRP)
			{
				i2c_PCA9685_CBF &=(~(1<<i2c_PCA9685_CBF_Start)); // ����� ��������� - �������� �������
				i2c_PCA9685_CBF |=(1<<i2c_PCA9685_CBF_Empty);	 // ������ ��������� - ����� �������
			}
			*/
}





void  ICACHE_FLASH_ATTR i2c_PCA9685_Servo_Simple(uint8 AddrOffset, uint8 LED_N, uint8 Position)
{
	/*
	 * 110 - 0.54 ms
	 * 205 - 1ms
	 * 410 - 2ms
	 * 480 - 2.35 ms
	 * Position limit 0 ... 100
	 * 0 -> 1 ms
	 * 100 -> 2 ms
	 */
	uint16 Servo_on;
	if (Position < 0)
	{
		Position = 0;
	}
	if (Position > 100)
	{
		Position = 100;
	}
	Servo_on = 4095 - (int)(Servo_1ms + (Position/100.0) * Servo_1ms);
	if (i2c_PCA9685_CBF & (1<<i2c_PCA9685_Initialized))
	{
	i2c_PCA9685_Push_CB (AddrOffset, LED_N, LO(Servo_on), HI(Servo_on), 0xFF, 0x0F);
	}
}


