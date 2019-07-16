/*
 * temperature_selector.c
 *
 *  Created on: 13 вер. 2017 р.
 *      Author: Kitaro
 */


#include <esp8266.h>
#include "pages/temperature_selector.h"
#include "io.h"
#include "user_config.h"
//#include "string.h"

uint8 temperature_options_current;










int ICACHE_FLASH_ATTR tpl_temperature_selector(HttpdConnData *connData, char *token, void **arg)
{

	char cj[80];
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;

	if (os_strcmp(token, "s_opt")==0)
	{
		os_sprintf(buff,"%d",number_of_temperature_options_channels);
	}
	memset(&cj[0], 0, sizeof(cj));

	if (os_strcmp(token, "r_opt")==0)
	{
		os_sprintf(buff,"%d",number_of_relay_channels);
	}
	memset(&cj[0], 0, sizeof(cj));

	if (os_strcmp(token, "p_opt")==0)
	{
		os_sprintf(buff,"%d",number_of_PWM_channels);
	}
	memset(&cj[0], 0, sizeof(cj));

	if (os_strcmp(token, "t_opt")==0)
	{
		os_sprintf(buff,"%d",ds18b20_amount_of_devices);
	}
	memset(&cj[0], 0, sizeof(cj));



















	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}


int ICACHE_FLASH_ATTR cgi_temperature_selector_1(HttpdConnData *connData) {
	int len;
	char buff[1024];

	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}




INFO("\r\n\r\n");
	len=httpdFindArg(connData->post->buff, "data", buff, sizeof(buff));
	if (len!=0)
	{
		INFO(buff);
		INFO("\r\n\r\n");





		char * pch;
		pch = strtok (buff,",");
		temperature_options_current = atoi (pch);
		INFO(pch);
		INFO("\r\n\r\n");
		pch = strtok (NULL, ",");
		sysCfg.Temperature_selector_array[temperature_options_current].temparture_sensor = atoi (pch);
		INFO(pch);
		INFO("\r\n\r\n");
		pch = strtok (NULL, ",");
		sysCfg.Temperature_selector_array[temperature_options_current].control_channel = atoi (pch);
		INFO(pch);
		INFO("\r\n\r\n");
		pch = strtok (NULL, ",");
		sysCfg.Temperature_selector_array[temperature_options_current].on_temperature = atoi (pch);
		INFO(pch);
		INFO("\r\n\r\n");
		pch = strtok (NULL, ",");
		sysCfg.Temperature_selector_array[temperature_options_current].off_temperature = atoi (pch);
		INFO(pch);
		INFO("\r\n\r\n");
		pch = strtok (NULL, ",");
		sysCfg.Temperature_selector_array[temperature_options_current].lower_dimmer_value = atoi (pch);
		INFO(pch);
		INFO("\r\n\r\n");
		pch = strtok (NULL, ",");
		sysCfg.Temperature_selector_array[temperature_options_current].upper_dimmer_value = atoi (pch);
		INFO(pch);
		INFO("\r\n\r\n");
		SysCFG_Save();

//		i2c_PCF8574_Write(0x4c,currLedState,1);

//		ioLed(currLedState);
	}

//	httpdRedirect(connData, "/temperature_selector/temperature_selector.tpl");
	return HTTPD_CGI_DONE;
}








int ICACHE_FLASH_ATTR var_temperature_selector(HttpdConnData *connData, char *token, void **arg)
{
	char cj[80];
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	INFO("\r\n\r\n getArgs:   ");
	INFO(connData->getArgs);
	INFO("\r\n\r\n");
	os_strcpy(cj,connData->getArgs);
	char * pch;
	pch = strtok (cj,"val=");
	temperature_options_current = atoi (pch);
	if ((temperature_options_current < 0)| (temperature_options_current>number_of_temperature_options_channels))
	{//Error, reload page
		httpdRedirect(connData, "/temperature_selector/temperature_selector.tpl");
		return HTTPD_CGI_DONE;
	}
	INFO("ch:%d",temperature_options_current);
	INFO("\r\n\r\n");

	if (os_strcmp(token, "txt")==0)
	{//	fill temperature selector element
		os_sprintf(buff,"%d,%d,%d,%d,%d,%d",\
			sysCfg.Temperature_selector_array[temperature_options_current].temparture_sensor,\
			sysCfg.Temperature_selector_array[temperature_options_current].control_channel,\
			sysCfg.Temperature_selector_array[temperature_options_current].on_temperature,\
			sysCfg.Temperature_selector_array[temperature_options_current].off_temperature,\
			sysCfg.Temperature_selector_array[temperature_options_current].lower_dimmer_value,\
			sysCfg.Temperature_selector_array[temperature_options_current].upper_dimmer_value);
	}
	memset(&cj[0], 0, sizeof(cj));





















	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}

