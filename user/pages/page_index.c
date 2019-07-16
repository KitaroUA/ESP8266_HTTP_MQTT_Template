/*
 * page_index.c
 *
 *  Created on: 15 лют. 2018 р.
 *      Author: Kitaro
 */


#include "pages/page_index.h"
#include <esp8266.h>
#include "io.h"
#include "user_config.h"








int ICACHE_FLASH_ATTR tpl_index(HttpdConnData *connData, char *token, void **arg)
{
	char cj[80];
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;

/*	if (os_strcmp(token, "curTime")==0)
	{
		if (DS3231_Date[0] == 0)
		{	//���� ������� ���� ������� - ������ �� ��������
			os_strcpy(buff, "");
		}
		else
		{
			os_sprintf(cj, "%02x:%02x:%02x %02x.%02x.20%02x",  DS3231_Time[2], DS3231_Time[1], DS3231_Time[0], DS3231_Date[0], DS3231_Date[1], DS3231_Date[2]);
			os_strcpy(buff, cj);
		}
	}
	memset(&cj[0], 0, sizeof(cj));
*/

/*	if (os_strcmp(token, "working_hr")==0)
	{
		uint16 n16 = atoi (sysCfg.on_time);
		uint8 nh = n16/60;
		uint8 nm = n16 - nh*60;
		uint16 f16 = atoi (sysCfg.off_time);
		uint8 fh = f16/60;
		uint8 fm = f16 - fh*60;

		if ( (atoi(sysCfg.on_time)) < (atoi(sysCfg.off_time)))
		{
			os_sprintf(cj,"Режим роботи: %02d:%02d - %02d:%02d",nh,nm,fh,fm);
			os_strcpy(buff, cj);
		}
		else
		{
			os_sprintf(cj,"Режим роботи: 00:00 - %02d:%02d, %02d:%02d - 23:59",fh,fm,nh,nm);
			os_strcpy(buff, cj);
		}


	}
	memset(&cj[0], 0, sizeof(cj));
*/



/*	if (os_strcmp(token, "Temperature")==0)
		{//Temperature_display_array[0]
			os_sprintf(cj, "Температура в акваріумі: %d.%02d°",ds18b20_temperature_arrey[sysCfg.Temperature_display_array[0]]/10000,(abs(ds18b20_temperature_arrey[sysCfg.Temperature_display_array[0]])%10000)/100);
			if (ds18b20_amount_of_devices > 1)
			{
				os_sprintf(cj,"Температура в акваріумі: %d.%02d°<br>Температура зовні: %d.%02d°",\
		ds18b20_temperature_arrey[sysCfg.Temperature_display_array[0]]/10000,(abs(ds18b20_temperature_arrey[sysCfg.Temperature_display_array[0]])%10000)/100,\
		ds18b20_temperature_arrey[sysCfg.Temperature_display_array[1]]/10000,(abs(ds18b20_temperature_arrey[sysCfg.Temperature_display_array[1]])%10000)/100 );
			}
			os_strcpy(buff,cj);
		}

	memset(&cj[0], 0, sizeof(cj));
//
	if (os_strcmp(token, "Humidity")==0) {
		if (v_tempOffMsg == "" )
		{
			os_strcpy(buff, "");
		}
		else
		{
			os_sprintf(cj, "%02d", (int)i2c_SHT3X_Hum);
			os_strcpy(buff,cj);
		}
	}
	memset(&cj[0], 0, sizeof(cj));
*/
	/*
	if (os_strcmp(token, "light_status")==0) {
		if (global_light_status == 1 )
		{
			os_strcpy(buff, "\"#00FF00\"");
		}
		else
		{
			os_strcpy(buff,"\"#FF0000\"");
		}
	}
	memset(&cj[0], 0, sizeof(cj));
*/
/*
	if (os_strcmp(token, "checker_1_status")==0) {
		if (temporary_light_off_timer == 0 )
		{
			os_strcpy(buff, "");
		}
		else
		{
			os_strcpy(buff,"checked");
		}
	}
	memset(&cj[0], 0, sizeof(cj));


	if (os_strcmp(token, "checker_2_status")==0) {
		if (temporary_light_on_timer == 0 )
		{
			os_strcpy(buff, "");
		}
		else
		{
			os_strcpy(buff,"checked");
		}
	}
	memset(&cj[0], 0, sizeof(cj));
*/

/*

	if (os_strcmp(token, "checker_1_text")==0) {
		if (temporary_light_off_timer == 0 )
		{
			os_sprintf(buff,"Тимчасово вимкнути світло на: %d хв.", atoi (sysCfg.tempOff_time));
//			os_strcpy(buff, "");
		}
		else
		{
			os_sprintf(buff,"Світло тимчасово вимкнено ще: %d хв. %d сек.", temporary_light_off_timer/60, temporary_light_off_timer%60);
//			os_strcpy(buff,"checked");
		}
	}
	memset(&cj[0], 0, sizeof(cj));



	if (os_strcmp(token, "checker_2_text")==0) {
		if (temporary_light_on_timer == 0 )
		{
			os_sprintf(buff,"Тимчасово увімкнути світло на: %d хв.", atoi (sysCfg.tempOn_time));
//			os_strcpy(buff, "");
		}
		else
		{
			os_sprintf(buff,"Світло тимчасово увімкнено ще: %dхв. %d сек.", temporary_light_on_timer/60, temporary_light_on_timer%60);
//			os_strcpy(buff,"checked");
		}
	}
	memset(&cj[0], 0, sizeof(cj));
	*/

/*	if (os_strcmp(token, "uptime")==0)
	{
		uint32 seconds, hours, minutes;
		int days;

		minutes = uptime / 60;
		hours = minutes / 60;
		days = hours /24;
		os_sprintf(cj,"uptime: %d days, %02d:%02d:%02d<br>try: %d", days, (int)(hours%24), (int)(minutes%60), (int)(uptime%60), sysCfg.try);
		INFO(cj);
//		os_sprintf(buff,"uptime: %d days, %02d:%02d:%02d", days, (int)(hours%24), (int)(minutes%60), (int)(uptime%60));
//		os_sprintf(buff,"uptime: ");
		os_strcpy(buff,cj);
	}
	memset(&cj[0], 0, sizeof(cj));
*/

	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}








int ICACHE_FLASH_ATTR cgi_index_get_data(HttpdConnData *connData)
{

		int len;			//length of user name
		char buff[128];		//Temporary buffer for name
		char output[256];	//Temporary buffer for HTML output

		//If the browser unexpectedly closes the connection, the CGI will be called
		//with connData->conn=NULL. We can use this to clean up any data. It's not really
		//used in this simple CGI function.
		if (connData->conn==NULL) {
			//Connection aborted. Clean up.
			return HTTPD_CGI_DONE;
		}

		if (connData->requestType==HTTPD_METHOD_GET)
		{
			//Sorry, we only accept GET requests.
//			httpdStartResponse(connData, 406);  //http error code 'unacceptable'
//			httpdEndHeaders(connData);
//			return HTTPD_CGI_DONE;
//==================================================
//============================Working on get request
//==================================================
				len=httpdFindArg(connData->getArgs, "sensor_data", buff, sizeof(buff));
				if (len!=0)
				{
					char cj[80];

					if(os_strcmp("DS_temperature",buff)==0)
					{
					os_sprintf(cj, "Температура в акваріумі: %d.%02d°",ds18b20_temperature_arrey[sysCfg.Temperature_display_array[0]]/10000,(abs(ds18b20_temperature_arrey[sysCfg.Temperature_display_array[0]])%10000)/100);
					if (ds18b20_amount_of_devices > 1)
					{
						os_sprintf(cj,"Температура в акваріумі: %d.%02d°<br>Температура зовні: %d.%02d°<br>Освітленність: %d лк",\
				ds18b20_temperature_arrey[sysCfg.Temperature_display_array[0]]/10000,(abs(ds18b20_temperature_arrey[sysCfg.Temperature_display_array[0]])%10000)/100,\
				ds18b20_temperature_arrey[sysCfg.Temperature_display_array[1]]/10000,(abs(ds18b20_temperature_arrey[sysCfg.Temperature_display_array[1]])%10000)/100,\
				i2c_BH1750FVI_Illuminance);
					}
					os_strcpy(buff,cj);
					}
					memset(&cj[0], 0, sizeof(cj));

					if(os_strcmp("current_time",buff)==0)
					{
					os_sprintf(cj, "%02x:%02x:%02x<br>%02x.%02x.20%02x",  DS3231_Time[2], DS3231_Time[1], DS3231_Time[0], DS3231_Date[0], DS3231_Date[1], DS3231_Date[2]);
					os_strcpy(buff, cj);
					}
					memset(&cj[0], 0, sizeof(cj));

					if(os_strcmp("working_time",buff)==0)
					{
						uint16 n16 = atoi (sysCfg.on_time);
						uint8 nh = n16/60;
						uint8 nm = n16 - nh*60;
						uint16 f16 = atoi (sysCfg.off_time);
						uint8 fh = f16/60;
						uint8 fm = f16 - fh*60;

						if ( (atoi(sysCfg.on_time)) < (atoi(sysCfg.off_time)))
						{
							os_sprintf(cj,"Режим роботи: %02d:%02d - %02d:%02d",nh,nm,fh,fm);
							os_strcpy(buff, cj);
						}
						else
						{
							os_sprintf(cj,"Режим роботи: 00:00 - %02d:%02d, %02d:%02d - 23:59",fh,fm,nh,nm);
							os_strcpy(buff, cj);
						}
					}
					memset(&cj[0], 0, sizeof(cj));

					if (os_strcmp("up_time",buff)==0)
					{
						uint32 seconds, hours, minutes;
						int days;

						minutes = uptime / 60;
						hours = minutes / 60;
						days = hours /24;
						os_sprintf(cj,"uptime: %d days, %02d:%02d:%02d<br>try: %d", days, (int)(hours%24), (int)(minutes%60), (int)(uptime%60), sysCfg.try);
//						INFO(cj);
				//		os_sprintf(buff,"uptime: %d days, %02d:%02d:%02d", days, (int)(hours%24), (int)(minutes%60), (int)(uptime%60));
				//		os_sprintf(buff,"uptime: ");
						os_strcpy(buff,cj);
					}
					memset(&cj[0], 0, sizeof(cj));











					if (os_strcmp("get_data",buff)==0)
					{
						//	time
						os_sprintf(cj, "%02x:%02x:%02x",  DS3231_Time[2], DS3231_Time[1], DS3231_Time[0]);
						os_strcpy(buff,cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));

						//	date
						os_sprintf(cj, "%02x.%02x.20%02x",   DS3231_Date[0], DS3231_Date[1], DS3231_Date[2]);
						os_strcat(buff, cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));

						//working_hr
						uint16 n16 = atoi (sysCfg.on_time);
						uint8 nh = n16/60;
						uint8 nm = n16 - nh*60;
						uint16 f16 = atoi (sysCfg.off_time);
						uint8 fh = f16/60;
						uint8 fm = f16 - fh*60;

						if ( (atoi(sysCfg.on_time)) < (atoi(sysCfg.off_time)))
						{
							os_sprintf(cj,"%02d:%02d - %02d:%02d",nh,nm,fh,fm);
							os_strcat(buff, cj);
							os_strcat(buff,";");
						}
						else
						{
							os_sprintf(cj,"00:00 - %02d:%02d, %02d:%02d - 23:59",fh,fm,nh,nm);
							os_strcat(buff, cj);
							os_strcat(buff,";");
						}
						memset(&cj[0], 0, sizeof(cj));

						//temper_DS_1
						os_sprintf(cj, "%d.%02d°",ds18b20_temperature_arrey[sysCfg.Temperature_display_array[0]]/10000,(abs(ds18b20_temperature_arrey[sysCfg.Temperature_display_array[0]])%10000)/100);
						os_strcat(buff, cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));

						//temper_DS_2
						os_sprintf(cj, "%d.%02d°",ds18b20_temperature_arrey[sysCfg.Temperature_display_array[1]]/10000,(abs(ds18b20_temperature_arrey[sysCfg.Temperature_display_array[1]])%10000)/100);
						os_strcat(buff, cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));

						//	temper_BM
						os_sprintf(cj,"%d.%02u",bme280_temperature / 100, bme280_temperature % 100);
						os_strcat(buff, cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));

						//	hum_BM
						os_sprintf(cj,"%u.%02u",bme280_humidity >> 10, ((bme280_humidity & 0x000003FF) * 100) >> 10);
						os_strcat(buff, cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));

						//	press_BM
						os_sprintf(cj,"%u.%02u",(bme280_pressure >> 8) / 100, (bme280_pressure >> 8) % 100);
						os_strcat(buff, cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));

						//	checker_1_state
						if (temporary_light_off_timer == 0 )
						{
							os_strcpy(cj, "0");
						}
						else
						{
							os_strcpy(cj,"1");
						}
						os_strcat(buff, cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));


						//	checker_1_time
						if (temporary_light_off_timer == 0 )
						{
							os_sprintf(cj,"%d", atoi (sysCfg.tempOff_time));
				//			os_strcpy(buff, "");
						}
						else
						{
							os_sprintf(cj,"%d хв. %d сек.", temporary_light_off_timer/60, temporary_light_off_timer%60);
				//			os_strcpy(buff,"checked");
						}
						os_strcat(buff, cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));



						//	checker_2_state
						if (temporary_light_on_timer == 0 )
						{
							os_strcpy(cj, "0");
						}
						else
						{
							os_strcpy(cj,"1");
						}
						os_strcat(buff, cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));



						//	checker_2_time
						if (temporary_light_on_timer == 0 )
						{
							os_sprintf(cj,"%d", atoi (sysCfg.tempOn_time));
				//			os_strcpy(buff, "");
						}
						else
						{
							os_sprintf(cj,"%dхв. %d сек.", temporary_light_on_timer/60, temporary_light_on_timer%60);
				//			os_strcpy(buff,"checked");
						}
						os_strcat(buff, cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));

						//	uptime
						uint32 seconds, hours, minutes;
						int days;

						minutes = uptime / 60;
						hours = minutes / 60;
						days = hours /24;
						os_sprintf(cj,"%d days, %02d:%02d:%02d", days, (int)(hours%24), (int)(minutes%60), (int)(uptime%60));
						os_strcat(buff,cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));


						//	Try
						os_sprintf(cj,"%d", sysCfg.try);
						os_strcat(buff,cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));


						// indicator_color
						if (global_light_status == 1 )
						{
							os_strcpy(cj, "#00FF00");
						}
						else
						{
							os_strcpy(cj,"#FF0000");
						}
						os_strcat(buff,cj);
						os_strcat(buff,";");
						memset(&cj[0], 0, sizeof(cj));

						//illuminance
						os_sprintf(cj,"%d", i2c_BH1750FVI_Illuminance);
						os_strcat(buff,cj);
						memset(&cj[0], 0, sizeof(cj));




					}
					memset(&cj[0], 0, sizeof(cj));













				}


		//Generate the header
		//We want the header to start with HTTP code 200, which means the document is found.
		httpdStartResponse(connData, 200);
		//We are going to send some HTML.
		httpdHeader(connData, "Content-Type", "text/html");
		//No more headers.
		httpdEndHeaders(connData);

		//We're going to send the HTML as two pieces: a head and a body. We could've also done
		//it in one go, but this demonstrates multiple ways of calling httpdSend.
		//Send the HTML head. Using -1 as the length will make httpdSend take the length
		//of the zero-terminated string it's passed as the amount of data to send.
//		httpdSend(connData, "<html><head><title>Page</title></head>", -1);
		//Generate the HTML body.
		len=sprintf(output, "%s", buff);
		//Send HTML body to webbrowser. We use the length as calculated by sprintf here.
		//Using -1 again would also have worked, but this is more efficient.
		httpdSend(connData, output, len);
		}
//====================================================
//============================End of get request proc.
//====================================================









		//All done.
		return HTTPD_CGI_DONE;

}


/*
int ICACHE_FLASH_ATTR tpl_index_js(HttpdConnData *connData, char *token, void **arg) {
	char cj[80];
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	if (os_strcmp(token, "light_status")==0) {
		if (global_light_status == 1 )
		{
			os_strcpy(buff, "\"#00FF00\"");
		}
		else
		{
			os_strcpy(buff,"\"#FF0000\"");
		}
	}
	memset(&cj[0], 0, sizeof(cj));

	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}
*/


int ICACHE_FLASH_ATTR cgi_index(HttpdConnData *connData)
{
	int len;
	char buff[1024];
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}
	len=httpdFindArg(connData->post->buff, "slider1", buff, sizeof(buff));
	if (len!=0)
	{
		INFO("\r\n slider 1   ");
		INFO(buff);
//		INFO ("     atoi = %d", atoi(buff) == 0);
		INFO("\r\n");

		if (os_strcmp("false",buff)==0)
		{
			temporary_light_off_timer=0;
		}
		if (os_strcmp("true",buff)==0)
		{
			temporary_light_off_timer=atoi (sysCfg.tempOff_time)*60;;
			temporary_light_on_timer=0;
		}
//		sysCfg.minLight = atoi (buff);
//		SysCFG_Save();
	}
	memset(&buff[0], 0, sizeof(buff));


	len=httpdFindArg(connData->post->buff, "slider2", buff, sizeof(buff));
	if (len!=0)
	{
		INFO("\r\n slider 2   ");
		INFO(buff);
//		INFO ("     atoi = %d", atoi(buff) == 0);
		INFO("\r\n");

		if (os_strcmp("false",buff)==0)
		{
			temporary_light_on_timer=0;
		}
		if (os_strcmp("true",buff)==0)
		{
			temporary_light_on_timer=atoi (sysCfg.tempOn_time)*60;
			temporary_light_off_timer=0;
		}
//		sysCfg.minLight = atoi (buff);
//		SysCFG_Save();
	}
	memset(&buff[0], 0, sizeof(buff));


	httpdStartResponse(connData, 200);
	//We are going to send some HTML.
	httpdHeader(connData, "Content-Type", "text/html");
	//No more headers.
	httpdEndHeaders(connData);

//	httpdRedirect(connData, "/index.tpl");
	return HTTPD_CGI_DONE;
}





