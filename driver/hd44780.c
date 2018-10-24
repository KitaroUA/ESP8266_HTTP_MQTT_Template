/*
 * hd44780.c
 *
 *  Created on: 8 ���. 2017 �.
 *      Author: Kitaro
 */


#include "driver/hd44780.h"
// #include "asm2c.h"

uint8 i2c_LCD_CBF=0; 				// ����� ���������� ������, �� ���� ����������� �ʲ - ����� ����� �����������.
uint8 i2c_LCD_Init_State=0;

uint8 i2c_LCD_CBD[i2c_LCD_CBL];
uint8 i2c_LCD_CBRP;				// ����������, ���������� ������ ������ �� ���������� ������, �������� �� ���, ��������� �� !!_���������_!! ������
uint8 i2c_LCD_CBWP;				// ����������, ���������� ������ ������ � ��������� �����

uint8 i2c_LCD_CBExch;				// ��������������� ���������� ��� ������ ������� ����� �����������
uint8 i2c_LCD_CBData_Out;			// ���������� ��� ������ � ����
//uint8 i2c_LCD_CB_CGRAM_Counter;    // ������� ��� ��������� ������ �������� � ���
//uint8 LCD2PCF_send_buffer[4];

uint8 i2c_LCD_light_control=0;
uint8 LCD2PCF_send_buffer [4];


#define HI2PCA(X) (X&0b11110000)
#define LO2PCA(X) ((X&0b00001111)<<4)



void ICACHE_FLASH_ATTR i2c_LCD_Write_Task (void)	//	�������� ���������, ������ ��� RTOS, �������� �������� ����� � ��, ������� ��� ������ � ���������, ���������� ���� ���� ��� �� ����������
{

	if (((i2c_LCD_CBF)& ((1<<i2c_LCD_CBF_Start)|(1<<i2c_LCD_Initialized))) == ((1<<i2c_LCD_CBF_Start)|(1<<i2c_LCD_Initialized))) // ���� �������� ����� �������
	{
		i2c_POP_LCD_CB();
//		SetTimerTask(hd44780_timer,i2c_LCD_Write_Task,i2c_LCD_Write_Task_Work,0);
	}
	else
	{
		SetTimerTask(hd44780_timer,i2c_LCD_Write_Task,i2c_LCD_Write_Task_Idle,0);
	}
}

//===============================================================
//========				i2c_LCD

void  ICACHE_FLASH_ATTR i2c_LCD_byte_sent (void)
{
	i2c_Do &= i2c_Free;                  		// ����������� ����

	if(i2c_Do & (i2c_ERR_NA|i2c_ERR_BF))   		// ������ ��� ������ ����?
	{	//	�������� ������� ����� � ��
		SetTimerTask(hd44780_timer, i2c_LCD_Send_byte, 2, 0);
	}
	else
	{
		i2c_LCD_CBRP ++;
		if (i2c_LCD_CBRP == i2c_LCD_CBL) {i2c_LCD_CBRP=0;}	//	�������� � "0" ������� �������, ���� ������� ���� ��

		if (i2c_LCD_CBWP == i2c_LCD_CBRP)
		{
			i2c_LCD_CBF &=(~(1<<i2c_LCD_CBF_Start)); // ����� ��������� - �������� �������
			i2c_LCD_CBF |=(1<<i2c_LCD_CBF_Empty);	 // ������ ��������� - ����� �������
		}
		SetTimerTask(hd44780_timer,i2c_LCD_Write_Task,i2c_LCD_Write_Task_Work,0);
			//	���������� ������� �� ������ ���.
	}

}
//void  ICACHE_FLASH_ATTR i2c_LCD_Send_byte (uint8 buffer, uint8 flag)
void  ICACHE_FLASH_ATTR i2c_LCD_Send_byte (void)
{

//uint8 t1;
	/*
LCD2PCF_send_buffer[1]=0;
LCD2PCF_send_buffer[3]=0;

if (flag == 1)
{		// Data
	 	LCD2PCF_send_buffer[1]=(1<<i2c_LCD_RS);
	 	LCD2PCF_send_buffer[3]=(1<<i2c_LCD_RS);
}
if (i2c_LCD_light_control != 0)
{
		LCD2PCF_send_buffer[1] |= (1<<i2c_LCD_Light);
		LCD2PCF_send_buffer[3] |= (1<<i2c_LCD_Light);
}
	LCD2PCF_send_buffer[1] |= HI2PCA(buffer);
	LCD2PCF_send_buffer[0] = LCD2PCF_send_buffer[1]|(1<<i2c_LCD_E);

	LCD2PCF_send_buffer[3] |= LO2PCA(buffer);
	LCD2PCF_send_buffer[2] = LCD2PCF_send_buffer[3]|(1<<i2c_LCD_E);
*/

//	i2c_PCF8574_Write(0x4e,LCD2PCF_send_buffer,4);
	if (!i2c_Write_Buffer(0x4e, LCD2PCF_send_buffer, 4, i2c_LCD_byte_sent) )      // ������
	{
//		SetTimerTask(hd44780_timer, i2c_POP_LCD_CB, 2, 0);
		SetTimerTask(hd44780_timer, i2c_LCD_Send_byte, 2, 0);
	}

}



void  ICACHE_FLASH_ATTR i2c_LCD_init(void)
{
	LCD2PCF_send_buffer[1]=0;
	LCD2PCF_send_buffer[3]=0;

switch (i2c_LCD_Init_State)
{
	case 0:
	{																	// ������� ����, ������������ ����� � ����� ������, �������� �������� ���� ������
																		// �������� �� ��������
																		// �������� > 40 ����.
					i2c_LCD_CBF |= (1<<i2c_LCD_CBF_Empty);              // ������������ ����� "�������� ����� ������"
					i2c_LCD_CBRP = 0;									// �������� ������� ������� �� ������
					i2c_LCD_CBWP = 0;

	i2c_LCD_Init_State ++;
	SetTimerTask(hd44780_timer,i2c_LCD_init,i2c_LCD_init_Preinit,0);
	break;
	}

	case 1:									// ����� ���� �����������, ������ ����
	{										// ������� ��������� � ����� 4-bit
			/*CBI(i2c_LCD_Contr_Port,i2c_LCD_RS)*/;
			LCD2PCF_send_buffer[1] &= (~(1<<i2c_LCD_RS));
			LCD2PCF_send_buffer[1] &=i2c_LCD_Data_Mask;
			LCD2PCF_send_buffer[1] |=0b00110000;
			LCD2PCF_send_buffer[0] = LCD2PCF_send_buffer[1]|(1<<i2c_LCD_E);
			i2c_PCF8574_Write(0x4e,LCD2PCF_send_buffer,2);
// 			Strob_E();
	i2c_LCD_Init_State ++;
	SetTimerTask(hd44780_timer,i2c_LCD_init,i2c_LCD_init_Delay,0);
	break;
	}
	case 2:									// ����� ���� �����������, ������ ����
	{
			LCD2PCF_send_buffer[1] &=i2c_LCD_Data_Mask;
			LCD2PCF_send_buffer[1] |=0b00110000;
			LCD2PCF_send_buffer[0] = LCD2PCF_send_buffer[1]|(1<<i2c_LCD_E);
					i2c_PCF8574_Write(0x4e,LCD2PCF_send_buffer,2);
	i2c_LCD_Init_State ++;
	SetTimerTask(hd44780_timer,i2c_LCD_init,i2c_LCD_init_Delay,0);
	break;
	}
	case 3:									// ����� ���� �����������, ������ ���� 001 DL N F, DL=0 ���������� ���� ������ 4
	{										// ������� ��������� � ����� 4-bit
			LCD2PCF_send_buffer[1] &=i2c_LCD_Data_Mask;
			LCD2PCF_send_buffer[1] |=0b00100000;
			LCD2PCF_send_buffer[0] = LCD2PCF_send_buffer[1]|(1<<i2c_LCD_E);
			i2c_PCF8574_Write(0x4e,LCD2PCF_send_buffer,2);
	i2c_LCD_Init_State ++;
	SetTimerTask(hd44780_timer,i2c_LCD_init,i2c_LCD_init_Delay,0);
	break;
	}
	case 4:									// ����� ���� �����������, ������ ����
	{
			LCD2PCF_send_buffer[1] &=i2c_LCD_Data_Mask;
			LCD2PCF_send_buffer[1] |=0b00100000;
			LCD2PCF_send_buffer[0] = LCD2PCF_send_buffer[1]|(1<<i2c_LCD_E);
			i2c_PCF8574_Write(0x4e,LCD2PCF_send_buffer,2);
	i2c_LCD_Init_State ++;
	SetTimerTask(hd44780_timer,i2c_LCD_init,i2c_LCD_init_Delay,0);
	break;
	}
	case 5:
	{										// ����� ���� �����������, ����� ����
			LCD2PCF_send_buffer[1] &=i2c_LCD_Data_Mask;		// ������������ ��� N,F, N=1 (2-Line), F=0 font type 5x8 dots
			LCD2PCF_send_buffer[1] |=0b10000000;
			LCD2PCF_send_buffer[0] = LCD2PCF_send_buffer[1]|(1<<i2c_LCD_E);
			i2c_PCF8574_Write(0x4e,LCD2PCF_send_buffer,2);
	i2c_LCD_Init_State ++;
	SetTimerTask(hd44780_timer,i2c_LCD_init,i2c_LCD_init_Delay,0);
	break;
}
	case 6:									// ����� ���� �����������
			{										// 0b00001 D C B, D=1 (�������� ������), C=0 (������ �� �������), B=0 (������ �� �����)
			LCD2PCF_send_buffer[1] &=i2c_LCD_Data_Mask;
			LCD2PCF_send_buffer[0] = LCD2PCF_send_buffer[1]|(1<<i2c_LCD_E);
			LCD2PCF_send_buffer[3] &=i2c_LCD_Data_Mask;
			LCD2PCF_send_buffer[3] |=0b11000000;
			LCD2PCF_send_buffer[2] = LCD2PCF_send_buffer[3]|(1<<i2c_LCD_E);
			i2c_PCF8574_Write(0x4e,LCD2PCF_send_buffer,4);
			i2c_LCD_Init_State ++;
			SetTimerTask(hd44780_timer,i2c_LCD_init,i2c_LCD_init_Delay,0);
			break;
			}
	case 7:									// �������� ���� �����������
			{										// �������� �������
			LCD2PCF_send_buffer[1] &=i2c_LCD_Data_Mask;
			LCD2PCF_send_buffer[0] = LCD2PCF_send_buffer[1]|(1<<i2c_LCD_E);
			i2c_PCF8574_Write(0x4e,LCD2PCF_send_buffer,2);
			i2c_LCD_Init_State ++;
			SetTimerTask(hd44780_timer,i2c_LCD_init,i2c_LCD_init_Delay,0);
			break;
			}
	case 8:
			{
			LCD2PCF_send_buffer[1] &=i2c_LCD_Data_Mask;
			LCD2PCF_send_buffer[1] |=0b00010000;
			LCD2PCF_send_buffer[0] = LCD2PCF_send_buffer[1]|(1<<i2c_LCD_E);
			i2c_PCF8574_Write(0x4e,LCD2PCF_send_buffer,2);
			i2c_LCD_Init_State ++;
			SetTimerTask(hd44780_timer,i2c_LCD_init,i2c_LCD_init_Delay,0);
			break;
			}
	case 9:								// �'��� ���� �����������
			{									// 0b000001 I/D SH, I/D=1 (������ �������� ��������), SH=0 (������� �� ���������)
			LCD2PCF_send_buffer[1] &=i2c_LCD_Data_Mask;
			LCD2PCF_send_buffer[0] = LCD2PCF_send_buffer[1]|(1<<i2c_LCD_E);
			LCD2PCF_send_buffer[3] &=i2c_LCD_Data_Mask;
			LCD2PCF_send_buffer[3] |=0b01100000;
			LCD2PCF_send_buffer[2] = LCD2PCF_send_buffer[3]|(1<<i2c_LCD_E);
			i2c_PCF8574_Write(0x4e,LCD2PCF_send_buffer,4);
			i2c_LCD_Init_State ++;
			SetTimerTask(hd44780_timer,i2c_LCD_init,i2c_LCD_init_Delay,0);
			break;
			}
	case 10:
			{
				i2c_LCD_CBF |= (1<<i2c_LCD_Initialized);
				i2c_LCD_light_control=1;
				SetTimerTask(hd44780_timer,i2c_LCD_Write_Task,10,0);
				break;
			}
	}
}



void  ICACHE_FLASH_ATTR Push_i2c_LCD_CB (char buffer)
{
if ((i2c_LCD_CBWP!=i2c_LCD_CBRP)||(i2c_LCD_CBF&(1<<i2c_LCD_CBF_Empty)))
	{
		i2c_LCD_CBF &= ~(1<<i2c_LCD_CBF_Empty);				//	������� ���� - �� �������
		i2c_LCD_CBD[i2c_LCD_CBWP]=buffer;
		i2c_LCD_CBWP ++;
		if (i2c_LCD_CBWP == i2c_LCD_CBL) {i2c_LCD_CBWP=0;}	//	�������� � "0" ������� �������, ���� ������� ���� ��
	}
else
	{
			i2c_LCD_CBF |= (1<<i2c_LCD_CBF_Full); // ������ ���� - ����� �����
			//cli(); //; ��� ����� ����� ���������, ������� ������� �� ������ ������������ ���������� ������, ������ �������� ���������
			uint8 i;
			os_printf("��� ����� ����� ���������, ������� ������� �� ������ ������������ ���������� ������ HD44780\r\n");
			while (1)
			{
				i++;
			}
//		}
	}
}




void  ICACHE_FLASH_ATTR i2c_POP_LCD_CB(void)
{
//		uint8 t1;
//		t1=i2c_LCD_CBD[i2c_LCD_CBRP];
	uint8 flag = 0;
	//; ����������� � ����������� �� ���������� ������
		if (i2c_LCD_CBD[i2c_LCD_CBRP] == i2c_LCD_CBComm)   // ���������, ����� ��� ������ ������� � ����
		{							// ��������, ����� ����������� ���� ������������ �����, ������� ������, �������� ��������� �����
			i2c_LCD_CBRP ++;									//	���������� �������� ���������� � ��
			if (i2c_LCD_CBRP == i2c_LCD_CBL) {i2c_LCD_CBRP=0;}	//	�������� � "0" ������� �������, ���� ������� ���� ��
			flag = 0;
//			i2c_LCD_Send_byte(i2c_LCD_CBD[i2c_LCD_CBRP],0);
//			i2c_LCD_CBRP ++;
//			if (i2c_LCD_CBRP == i2c_LCD_CBL) {i2c_LCD_CBRP=0;}	//	�������� � "0" ������� �������, ���� ������� ���� ��
		}
		else
		{
			if (i2c_LCD_CBD[i2c_LCD_CBRP] == i2c_LCD_CBData)   // ���������, ����� ��� ������ ������� � ����
			{						  	// ��������, ����� ����������� ���� ������������ �����, ������� ������, �������� ��������� �����
				i2c_LCD_CBRP ++;									//	���������� �������� ���������� � ��
				if (i2c_LCD_CBRP == i2c_LCD_CBL) {i2c_LCD_CBRP=0;}	//	�������� � "0" ������� �������, ���� ������� ���� ��
				flag = 1;
//				i2c_LCD_Send_byte(i2c_LCD_CBD[i2c_LCD_CBRP],1);
//				i2c_LCD_CBRP ++;
//				if (i2c_LCD_CBRP == i2c_LCD_CBL) {i2c_LCD_CBRP=0;}	//	�������� � "0" ������� �������, ���� ������� ���� ��
			}
			else
			{
				flag = 1;
//				i2c_LCD_Send_byte(i2c_LCD_CBD[i2c_LCD_CBRP],1);
//				i2c_LCD_CBRP ++;
//				if (i2c_LCD_CBRP == i2c_LCD_CBL) {i2c_LCD_CBRP=0;}	//	�������� � "0" ������� �������, ���� ������� ���� ��
			}
		}
		LCD2PCF_send_buffer[1]=0;
		LCD2PCF_send_buffer[3]=0;

		if (flag == 1)
		{		// Data
			 	LCD2PCF_send_buffer[1]=(1<<i2c_LCD_RS);
			 	LCD2PCF_send_buffer[3]=(1<<i2c_LCD_RS);
		}
		if (i2c_LCD_light_control != 0)
		{
				LCD2PCF_send_buffer[1] |= (1<<i2c_LCD_Light);
				LCD2PCF_send_buffer[3] |= (1<<i2c_LCD_Light);
		}
			LCD2PCF_send_buffer[1] |= HI2PCA(i2c_LCD_CBD[i2c_LCD_CBRP]);
			LCD2PCF_send_buffer[0] = LCD2PCF_send_buffer[1]|(1<<i2c_LCD_E);

			LCD2PCF_send_buffer[3] |= LO2PCA(i2c_LCD_CBD[i2c_LCD_CBRP]);
			LCD2PCF_send_buffer[2] = LCD2PCF_send_buffer[3]|(1<<i2c_LCD_E);
			i2c_LCD_Send_byte();
/*
			i2c_LCD_CBRP ++;
			if (i2c_LCD_CBRP == i2c_LCD_CBL) {i2c_LCD_CBRP=0;}	//	�������� � "0" ������� �������, ���� ������� ���� ��

			if (i2c_LCD_CBWP == i2c_LCD_CBRP)
			{
				i2c_LCD_CBF &=(~(1<<i2c_LCD_CBF_Start)); // ����� ��������� - �������� �������
				i2c_LCD_CBF |=(1<<i2c_LCD_CBF_Empty);	 // ������ ��������� - ����� �������
			}
			*/
}


// ��������� ������������ ������� ���� ASCII ������� � ���� ��������� HD44780
char  ICACHE_FLASH_ATTR i2c_LCD_Code_converter(char buffer) // ��������� ������������ ������� ���� ASCII ������� � ���� ��������� HD44780
{
//��������/������������ ����� �������, ��� ����������� ����������� ��������
        if ((buffer == 0xB2)||(buffer == 0xB3)||(buffer == 0xA8)||(buffer >= 0xC0))
        {
                switch (buffer)
                {
                        case 0xB2: return(0x49); break;//�
						case 0xB3: return(0x69); break;//�


						case 0xC0: return(0x41); break;//�
                        case 0xC1: return(0xA0); break;//�
                        case 0xC2: return(0x42); break;//�
                        case 0xC3: return(0xA1); break;//�
                        case 0xC4: return(0xE0); break;//�
                        case 0xC5: return(0x45); break;//�
                        case 0xA8: return(0xA2); break;//�
                        case 0xC6: return(0xA3); break;//�
                        case 0xC7: return(0xA4); break;//�
                        case 0xC8: return(0xA5); break;//�
                        case 0xC9: return(0xA6); break;//�
                        case 0xCA: return(0x4B); break;//�
                        case 0xCB: return(0xA7); break;//�
                        case 0xCC: return(0x4D); break;//�
                        case 0xCD: return(0x48); break;//H
                        case 0xCE: return(0x4F); break;//O
                        case 0xCF: return(0xA8); break;//�
                        case 0xD0: return(0x50); break;//P
                        case 0xD1: return(0x43); break;//C
                        case 0xD2: return(0x54); break;//T
                        case 0xD3: return(0xA9); break;//�
                        case 0xD4: return(0xAA); break;//�
                        case 0xD5: return(0x58); break;//X
                        case 0xD6: return(0xE1); break;//�
                        case 0xD7: return(0xAB); break;//�
                        case 0xD8: return(0xAC); break;//�
                        case 0xD9: return(0xE2); break;//�
                        case 0xDA: return(0xAD); break;//�
                        case 0xDB: return(0xAE); break;//�
                        case 0xDC: return(0x62); break;//�
                        case 0xDD: return(0xAF); break;//�
                        case 0xDE: return(0xB0); break;//�
                        case 0xDF: return(0xB1); break;//�
                        case 0xE0: return(0x61); break;//�
                        case 0xE1: return(0xB2); break;//�
                        case 0xE2: return(0xB3); break;//�
                        case 0xE3: return(0xB4); break;//�
                        case 0xE4: return(0xE3); break;//�
                        case 0xE5: return(0x65); break;//�
                        case 0xB8: return(0xB5); break;//�
                        case 0xE6: return(0xB6); break;//�
                        case 0xE7: return(0xB7); break;//�
                        case 0xE8: return(0xB8); break;//�
                        case 0xE9: return(0xB9); break;//�
                        case 0xEA: return(0xBA); break;//�
                        case 0xEB: return(0xBB); break;//�
                        case 0xEC: return(0xBC); break;//�
                        case 0xED: return(0xBD); break;//�
                        case 0xEE: return(0x6F); break;//�
                        case 0xEF: return(0xBE); break;//�
                        case 0xF0: return(0x70); break;//�
                        case 0xF1: return(0x63); break;//�
                        case 0xF2: return(0xBF); break;//�
                        case 0xF3: return(0x79); break;//�
                        case 0xF4: return(0xE4); break;//�
                        case 0xF5: return(0x78); break;//�
                        case 0xF6: return(0xE5); break;//�
                        case 0xF7: return(0xC0); break;//�
                        case 0xF8: return(0xC1); break;//�
                        case 0xF9: return(0xE6); break;//�
                        case 0xFA: return(0xC2); break;//�
                        case 0xFB: return(0xC3); break;//�
                        case 0xFC: return(0xC4); break;//�
                        case 0xFD: return(0xC5); break;//�
                        case 0xFE: return(0xC6); break;//�
                        case 0xFF: return(0xC7); break;//�
                }

       }
				 return(buffer); // ���� �� ������ � if , �� ��������� ����� ����
}


void  ICACHE_FLASH_ATTR i2c_LCD_Send_String(uint8 row, uint8 col, char str_out[])
{
volatile uint8 adress = 0;
uint8 i = 0;
// cli();
	switch(row)														//	������� ������� � ������� ���������, ���� �������� ������ �����
	{
		case 0: adress = i2c_LCD_Send_String_FirstRow; break;
		case 1: adress = i2c_LCD_Send_String_SecondRow; break;
		case 2: adress = i2c_LCD_Send_String_ThirdRow; break;
		case 3: adress = i2c_LCD_Send_String_FourthRow; break;
		default: adress = i2c_LCD_Send_String_FirstRow; break;
	}
	if ((col >=0)&(col <=19))
	{
			adress += col;
	}
	else
	{
			adress += 0;
	}

	adress |= i2c_LCD_Send_String_Set_DDRAM_Address;

	Push_i2c_LCD_CB(i2c_LCD_CBComm);								//	�������� � �� ������� - ��� ����� �������.


	Push_i2c_LCD_CB(adress);

	Push_i2c_LCD_CB(i2c_LCD_CBData);								//	�������� � �� �������  - ��� ����� ���.
	while ((str_out[i] != '\0')&((col+i)<i2c_LCD_Columns))											//	����� � �� �� ���� �����, ��� ���� �� ������� �� ��� ������...
	{
		Push_i2c_LCD_CB(i2c_LCD_Code_converter(str_out[i]));
		i++;
	}
	i2c_LCD_CBF |= (1<<i2c_LCD_CBF_Start);							//	������� ����, �� � �� � ���.
// sei();
}


//������� ��������� - ���������� ������ � ���� ���
void  ICACHE_FLASH_ATTR i2c_LCD_Code_display(uint8 row, uint8 col, char buffer)  //������� ��������� - ���������� ������ � ���� ���
{
	char ci3[3]; // ����� �� �����
	char hulfbuffer;

	hulfbuffer = (buffer>>4)&0b00001111;


                switch (hulfbuffer)
                {
	                case 0x00: ci3[0]='0'; break;
	                case 0x01: ci3[0]='1'; break;
	                case 0x02: ci3[0]='2'; break;
	                case 0x03: ci3[0]='3'; break;
	                case 0x04: ci3[0]='4'; break;
	                case 0x05: ci3[0]='5'; break;
	                case 0x06: ci3[0]='6'; break;
	                case 0x07: ci3[0]='7'; break;
	                case 0x08: ci3[0]='8'; break;
	                case 0x09: ci3[0]='9'; break;
	                case 0x0A: ci3[0]='A'; break;
	                case 0x0B: ci3[0]='B'; break;
	                case 0x0C: ci3[0]='C'; break;
	                case 0x0D: ci3[0]='D'; break;
	                case 0x0E: ci3[0]='E'; break;
	                case 0x0F: ci3[0]='F'; break;

				}

	hulfbuffer = buffer&0b00001111;


	switch (hulfbuffer)
	{
		case 0x00: ci3[1]='0'; break;
		case 0x01: ci3[1]='1'; break;
		case 0x02: ci3[1]='2'; break;
		case 0x03: ci3[1]='3'; break;
		case 0x04: ci3[1]='4'; break;
		case 0x05: ci3[1]='5'; break;
		case 0x06: ci3[1]='6'; break;
		case 0x07: ci3[1]='7'; break;
		case 0x08: ci3[1]='8'; break;
		case 0x09: ci3[1]='9'; break;
		case 0x0A: ci3[1]='A'; break;
		case 0x0B: ci3[1]='B'; break;
		case 0x0C: ci3[1]='C'; break;
		case 0x0D: ci3[1]='D'; break;
		case 0x0E: ci3[1]='E'; break;
		case 0x0F: ci3[1]='F'; break;

	}

	ci3[2] = 0;
i2c_LCD_Send_String(row,col,ci3);
}
