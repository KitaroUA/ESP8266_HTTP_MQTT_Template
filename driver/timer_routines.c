/*
 * timer_routines.c
 *
 *  Created on: 10 трав. 2018 р.
 *      Author: Kitaro
 */

#include "driver/timer_routines.h"

uint8 one_sec_task_state = 0;

void every_second_task();
void every_5_second_task();
void every_10_second_task();




static uint8 ICACHE_FLASH_ATTR bcdToDec(uint8 bcd) {
  return(((bcd / 16) * 10) + (bcd % 16));
}
static uint8 ICACHE_FLASH_ATTR decToBcd(uint8 dec) {
  return(((dec / 10) * 16) + (dec % 10));
}






void ICACHE_FLASH_ATTR init_circular_timer_proc(void)
{
	circular_timer_proc();
}










void ICACHE_FLASH_ATTR circular_timer_proc(void)
{

		uint8 sec = bcdToDec(DS3231_Time[0]);
		uint8 min = bcdToDec(DS3231_Time[1]);
		uint8 hr  = bcdToDec(DS3231_Time[2]);

		every_second_task();

		if(sec%5 == 0)	//	Every 5 sec task
		{
			every_5_second_task();
		}
		if(sec%10 == 0)	//	Every 10 sec task
		{
			every_10_second_task();
		}


			if ( (temporary_light_off_timer != 0))
			{
				temporary_light_off_timer = temporary_light_off_timer-1;
	//			os_printf("                                        temp_off_timer = %d \r\n", temp_off_timer);
			}
			if ( (temporary_light_on_timer != 0))
			{
				temporary_light_on_timer = temporary_light_on_timer-1;
	//			os_printf("                                        temp_off_timer = %d \r\n", temp_off_timer);
			}


	/*
			   ets_intr_lock(); // These seem to be OK
			   for (uint8 pixel = 0; pixel < PIXEL_COUNT; pixel++)
					ws2812_sendPixel((one_sec_task_state*255)/10,((10-one_sec_task_state)*255)/10,0);
			   ets_intr_unlock(); // These seem to be OK
			   ws2812_show();  // latch the colors
	*/


	//		ws2812_sendPixel(0,128,0);
	//		ws2812_sendPixel(0,0,128);















		char ci[21];
	//	os_sprintf(ci, "%02x:%02x:%02x %02x.%02x.20%02x",  DS3231_Time[2], DS3231_Time[1], DS3231_Time[0], DS3231_Date[0], DS3231_Date[1], DS3231_Date[2]);
		os_sprintf(ci, "%02x:%02x:%02x",  DS3231_Time[2], DS3231_Time[1], DS3231_Time[0]);
	//	os_sprintf(ci, "1");
		ci[(sizeof(ci)-1)]=0;
		i2c_SSD1306_Send_GB_Addr_String( 0,0, SSDFONT_2X,ci);

		if (wifi_station_get_connect_status() == 5)
		{
			ssd1306_global_buffer[3*128+0] ^= 0b11000000;
			ssd1306_global_buffer[3*128+1] ^= 0b11000000;
//			uint8 ii[1];
//			ii[0]=one_sec_timer_xor1;
//			i2c_PCF8574_Write(0x4c,ii,1);
		}

		if (sec == 0)	//	Task on every minute start
		{

			SetTimerTask(i2c_BH1750FVI_timer, i2c_BH1750FVI_Write,150,0);

		// 	MQTT tasks
			MQTT_Client* client_publish = &mqttClient;
			os_printf("\r\n MQTT conn state %d\r\n",client_publish->connState);
			if ((client_publish->connState == 15)& (sysCfg.mqtt_task_enabled == 1))
			{
				INFO("\r\nSend MQTT\r\n");
					char tc[80];
					//	os_sprintf (tc, "connState -> %d \r\n", client_publish->connState);
	//				os_sprintf(tc,"%02d", (int)i2c_SHT3X_Temp);
					os_sprintf(tc, "%d",ds18b20_temperature_arrey[0]);
					//	os_printf(tc);
					MQTT_Publish(client_publish, "/esp1/Temp", tc, strlen(tc), 0, 0);
	//				os_sprintf(tc,"%02d", (int)i2c_SHT3X_Hum);
					//	os_printf(tc);
					MQTT_Publish(client_publish, "/esp1/Hum", tc, strlen(tc), 0, 0);

			}


		}


		// Перевірка кожну годину
		if ( (DS3231_Time[1] == 0) & (DS3231_Time[0] == 0))
	//	if ( (DS3231_Time[1] != 0) & (DS3231_Time[0] != 0))
		{
			INFO ("\n ------------------an hour------------------------- \r\n");



			// Check is DST time

			if ( ((DS3231_Time[1] == 3)&(sysCfg.dst_active=0)) | ((DS3231_Time[0] == 4)&(sysCfg.dst_active=1)))
			{
			struct tm *dt;
			time_t timestamp = 1522921929;
	//		timestamp = 1522921929;
			dt = gmtime(&timestamp);
			dt->tm_sec  = bcdToDec(DS3231_Time[0]);
			dt->tm_min  = bcdToDec(DS3231_Time[1]);
			dt->tm_hour = bcdToDec(DS3231_Time[2]);
			dt->tm_mday = bcdToDec(DS3231_Date[0]);
			dt->tm_mon  = bcdToDec(DS3231_Date[1]&0b01111111) - 1;
			dt->tm_year = bcdToDec(DS3231_Date[2])+100;
			dt->tm_wday = DS3231_Date[3];
			if (dt->tm_wday == 7)
			{
				dt->tm_wday = 0; //� NTP ���� ����� �� 0 �� 6 ��������� � �����, ��������� � ����� �� 1 �� 7
			}


			char cj[80];
			check_dst(dt);
			os_sprintf(cj, "gmt: %d %02d:%02d:%02d %02d.%02d.%04d, %d",  dt->tm_isdst, dt->tm_hour, dt->tm_min, dt->tm_sec, dt->tm_mday, dt->tm_mon + 1, dt->tm_year + 1900, dt->tm_wday);
			os_printf(cj);
			os_printf("\r\n");
			if(dt->tm_isdst != sysCfg.dst_active)
			{
				if (dt->tm_isdst != 0)
				{
					INFO ("it`s summer time!!!! :)");
					dt->tm_hour += 1;
				}
				else
				{
					INFO ("it`s winter time :(");
					dt->tm_hour -= 1;

				}
				sysCfg.dst_active = dt->tm_isdst;
				SysCFG_Save();
				mktime(dt);


				os_sprintf(cj, "new time: %d %02d:%02d:%02d %02d.%02d.%04d, %d",  dt->tm_isdst, dt->tm_hour, dt->tm_min, dt->tm_sec, dt->tm_mday, dt->tm_mon + 1, dt->tm_year + 1900, dt->tm_wday);
				os_printf(cj);
				os_printf("\r\n");




				DS3231_Date_to_Write[0] = decToBcd(dt->tm_sec);
				DS3231_Date_to_Write[1] = decToBcd(dt->tm_min);
				DS3231_Date_to_Write[2] = decToBcd(dt->tm_hour);
				DS3231_Date_to_Write[3] = decToBcd(dt->tm_wday);
				DS3231_Date_to_Write[4] = decToBcd(dt->tm_mday);
				DS3231_Date_to_Write[5] = decToBcd(dt->tm_mon + 1);
				DS3231_Date_to_Write[6] = decToBcd(dt->tm_year - 100);
				DS3231_Set_Date();


			}
			}
			// End check is DST time



		}




		one_sec_task_state++;
		if (one_sec_task_state >=10)
		{
			one_sec_task_state = 0;

		}


	//	one_sec_timer_xor1 ^= 0xff;
		one_sec_timer_xor1 ^= 0b00000010;
		uptime++;
		SetTimerTask (circular_timer, circular_timer_proc, 1000, 0);
}






void ICACHE_FLASH_ATTR every_second_task()
{
	uint8 rand_time;
	rand_time = system_get_time()%100;
	SetTimerTask(SSD1306_CB_timer,i2c_SSD1306_LCD_Update,(rand_time),0);



















	// �������� ���� ���������
	// -------------------------
		uint16 cur_time = (bcdToDec(DS3231_Time[2]))*60+bcdToDec(DS3231_Time[1]);
//		os_printf("%X:%X=%d \r\n",DS3231_Time[2], DS3231_Time[1], (bcdToDec(DS3231_Time[2]))*60+bcdToDec(DS3231_Time[1]));
//		os_printf("%d %d %d\r\n",cur_time, atoi(sysCfg.on_time),atoi(sysCfg.off_time));

		if ( (atoi(sysCfg.on_time)) < (atoi(sysCfg.off_time)))
		{ // light control template - off-on-off
//			INFO ("          1          ");
			if (    (
					(cur_time >= atoi(sysCfg.on_time))
					&(cur_time<=atoi(sysCfg.off_time))
					&(temporary_light_off_timer == 0)
					&((light_avg <= sysCfg.minLight)|(sysCfg.minLight ==0))
					)
					|(temporary_light_on_timer != 0)
				)
			{
//				GPIO_OUTPUT_SET(RELAY_PIN, 1);
				global_light_status=1;
//				os_printf("  ON1 \r\n");
			}
			else
			{
//				GPIO_OUTPUT_SET(RELAY_PIN, 0);
				global_light_status=0;
//				os_printf("  OFF1 \r\n");
			}
		}
		else
		{// light control template - on-of-on
//			INFO ("          2          ");
			if (    (
					((cur_time >= atoi(sysCfg.on_time)|(cur_time < atoi(sysCfg.off_time))))
					&(temporary_light_off_timer == 0)
					&((light_avg <= sysCfg.minLight)|(sysCfg.minLight ==0))
					)
					|(temporary_light_on_timer != 0)
				)
			{
//				GPIO_OUTPUT_SET(RELAY_PIN, 1);
//				i2c_PCF8574_Write(0x4c,0xFF,1);
				global_light_status=1;
//				os_printf("  ON2 \r\n");
			}
			else
			{
//				GPIO_OUTPUT_SET(RELAY_PIN, 0);
//				i2c_PCF8574_Write(0x4c,0x00,1);
				global_light_status=0;
//				os_printf("  OFF2 \r\n");
			}

		}
		uint8 ii[1];
		ii[0] = bcdToDec(DS3231_Time[0]);
//		ii[0] = 0x01;
		i2c_PCF8574_Write(0x4c,ii,1);
//		BitBang_TLC5947_setPWM(0,(ii[0]*4096)/60);
//		BitBang_TLC5947_write();










}




void ICACHE_FLASH_ATTR every_5_second_task ()
{

#ifdef SCAN_IIC
		i2c_Read_Scan(SCAN_IIC_first_address,SCAN_IIC_last_address);
#endif




	static uint8 ds_screen_tick;
	// start reading devices...
		// ... i'm handing off with a timer, could have called directly
		if (ds18b20_amount_of_devices != 255) //	I Dds18b20 still not scaned
		{
		os_timer_disarm(&ds18b20_timer);
		os_timer_setfn(&ds18b20_timer, (os_timer_func_t *)ds18b20p1, (void *)0);
		os_timer_arm(&ds18b20_timer, 100, 0);

//			INFO("Temp%d=%d.%02d\n",ds_screen_tick,ds18b20_temperature_arrey[ds_screen_tick]/10000,abs(ds18b20_temperature_arrey[ds_screen_tick])%10000);
		char cj[21];


		os_sprintf(cj, "Temp%d=%d.%02d%c",ds_screen_tick,ds18b20_temperature_arrey[ds_screen_tick]/10000,(abs(ds18b20_temperature_arrey[ds_screen_tick])%10000)/100,0x80);
		cj[(sizeof(cj)-1)]=0;
		i2c_SSD1306_Send_GB_Addr_String( 50,3, SSDFONT_1X,cj);

		ds_screen_tick ^= 0b00000001;
		}



// 		os_printf("\r\n --Nextion Code-- \r\n");
		//		os_sprintf(cj, "Temp%d=%d.%02d%c",ds18b20_temperature_arrey[0]/10000,(abs(ds18b20_temperature_arrey[0])%10000)/100,0x80);
		//   0...180 - 22...25C (...3C)
		// 308...359 < 22C
		// 180...232 > 25C
		// ds18b20_temperature_arrey[0]/100 -> cur_temp*100 -> 2200...2500

//		os_printf("\r\n %d \r\n",ds18b20_temperature_arrey[0]/100);
//		os_printf("\r\n %d \r\n",(((ds18b20_temperature_arrey[0]/100)-2000)*284)/1000);

				if ( ( (ds18b20_temperature_arrey[0]/100)>=2000 ) & ( (ds18b20_temperature_arrey[0]/100)<=3000 ) )
				{
				nextion_z0 = (((ds18b20_temperature_arrey[0]/100)-2000)*284)/1000;
				}
				nextion_z0 = nextion_z0 - 52;
				if (nextion_z0 < 0) nextion_z0 = 360 - nextion_z0;


//				if ( ( (ds18b20_temperature_arrey[0]/100)<2113 ) ) nextion_z0 = 308;



//				if ( ( (ds18b20_temperature_arrey[0]/100)>2587 ) ) nextion_z0 = 232;

		//		  if (nextion_z0 >=360) nextion_z0 = 0;
		//		os_printf("/r/n %d /r/n",ds18b20_temperature_arrey[0]/100);

				os_printf("\r\n --Nextion Out-- \r\n");
				os_printf("%c%c%c",0xff,0xff,0xff);
				os_printf("z0.val=%d",nextion_z0);
				os_printf("%c%c%c",0xff,0xff,0xff);
				if (ds18b20_temperature_arrey[0] >=0)
				{
				os_printf("t0.txt=\"+%02d.%01d\"",ds18b20_temperature_arrey[0]/10000,(abs(ds18b20_temperature_arrey[0])%10000)/100);
				}
				else
				{
					os_printf("t0.txt=\"-%02d.%01d\"",ds18b20_temperature_arrey[0]/10000,(abs(ds18b20_temperature_arrey[0])%10000)/100);

				}
				os_printf("%c%c%c",0xff,0xff,0xff);
				os_printf("t1.txt=\"");
				os_printf("%02x:%02x:%02x",  DS3231_Time[2], DS3231_Time[1], DS3231_Time[0]);
				os_printf("\"");
				os_printf("%c%c%c",0xff,0xff,0xff);
				if ((ds18b20_temperature_arrey[0]/100)<2700)
				{
					os_printf("t0.pco=512");
					os_printf("%c%c%c",0xff,0xff,0xff);
				}
				else
				{
					os_printf("t0.pco=63488");
					os_printf("%c%c%c",0xff,0xff,0xff);

				}

//				os_printf("\r\n --Nextion-- \r\n");
/*				char cj[21];
				os_sprintf(cj, "UART0 send");

				uart0_tx_buffer(cj, sizeof(cj));*/





}





void ICACHE_FLASH_ATTR every_10_second_task ()
{
	ADS1015_REG_GAIN_selector = 3;
	ads1015_channel_to_read = 0;
	ads1015ReadADC_SingleRead();

	char cj[8];
	os_sprintf(cj, "    ");
	cj[(sizeof(cj)-1)]=0;
	i2c_SSD1306_Send_GB_Addr_String( 0,2, SSDFONT_1X,cj);
	os_sprintf(cj, "%d",2048 - ads1015_data[0]);
	cj[(sizeof(cj)-1)]=0;
	i2c_SSD1306_Send_GB_Addr_String( 0,2, SSDFONT_1X,cj);


}

