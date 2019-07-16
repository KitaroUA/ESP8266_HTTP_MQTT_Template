/*
 * mqtt_settings.c
 *
 *  Created on: 30 жовт. 2017 р.
 *      Author: Kitaro
 */


#include <esp8266.h>
#include "pages/mqtt_settings.h"
#include "io.h"
#include "user_config.h"

int ICACHE_FLASH_ATTR tpl_set_mqtt(HttpdConnData *connData, char *token, void **arg)
{
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	char cj[80];

	if (os_strcmp(token,"v_mqtt")==0)
	{
//		os_sprintf(buff, "\"%s\"", sysCfg.ntp);
		os_strcpy(buff, sysCfg.mqtt_host);
	}
	memset(&cj[0], 0, sizeof(cj));

	if (os_strcmp(token,"mqtt_status")==0)
	{
		if (sysCfg.mqtt_task_enabled == 1)
		{
			os_strcpy(buff, "1");
		}
		else
		{
			os_strcpy(buff, "0");

		}
//		os_sprintf(buff, "\"%s\"", sysCfg.ntp);
	}
	memset(&cj[0], 0, sizeof(cj));

	SysCFG_Save();

	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}
int ICACHE_FLASH_ATTR cgi_set_mqtt(HttpdConnData *connData)
{
	int len;
	char buff[1024];
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}

	len=httpdFindArg(connData->post->buff, "mqttip", buff, sizeof(buff));
	if (len!=0)
	{
		os_strcpy(sysCfg.mqtt_host, buff);
//		os_strcpy(ntp, buff);
	}
	memset(&buff[0], 0, sizeof(buff));

	len=httpdFindArg(connData->post->buff, "mqtt_status", buff, sizeof(buff));
	if (len!=0)
	{
//		INFO("\r\n");
//		INFO(buff);
//		INFO("\r\n");
		sysCfg.mqtt_task_enabled = atoi(buff);
//		os_strcpy(ntp, buff);
	}
	memset(&buff[0], 0, sizeof(buff));




	SysCFG_Save();


	httpdRedirect(connData, "/set_mqtt/set_mqtt.tpl");
	return HTTPD_CGI_DONE;

}
