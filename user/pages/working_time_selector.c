/*
 * temperature_selector.c
 *
 *  Created on: 13 вер. 2017 р.
 *      Author: Kitaro
 */


#include <esp8266.h>
#include "pages/working_time_selector.h"
#include "io.h"
#include "user_config.h"












int ICACHE_FLASH_ATTR tpl_working_time_selector(HttpdConnData *connData, char *token, void **arg)
{
	char cj[80];
	char buff[128];

	if (token==NULL) return HTTPD_CGI_DONE;

	if (os_strcmp(token, "time_to_on_select")==0)
	{
		uint16 m16 = atoi (sysCfg.on_time);
		uint8 h = m16/60;
		uint8 m = m16 - h*60;
		os_sprintf(cj,"%02d:%02d", h, m);
		os_strcpy(buff,cj);

	}
	memset(&cj[0], 0, sizeof(cj));

/*	if (os_strcmp(token, "time_to_on_d")==0)
	{
//		os_strcpy(buff,sysCfg.on_time);
		uint16 m16 = atoi (sysCfg.on_time);
		uint8 h = m16/60;
		uint8 m = m16 - h*60;
		os_sprintf(cj,"%02d:%02d", h, m);
		os_strcpy(buff,cj);

	}
	memset(&cj[0], 0, sizeof(cj));*/


	if (os_strcmp(token, "time_to_off_select")==0)
	{
		uint16 m16 = atoi (sysCfg.off_time);
		uint8 h = m16/60;
		uint8 m = m16 - h*60;
		os_sprintf(cj,"%02d:%02d", h, m);
		os_strcpy(buff,cj);

	}
	memset(&cj[0], 0, sizeof(cj));
/*	if (os_strcmp(token, "time_to_off_d")==0)
	{
//		os_strcpy(buff,sysCfg.off_time);
		uint16 m16 = atoi (sysCfg.off_time);
		uint8 h = m16/60;
		uint8 m = m16 - h*60;
		os_sprintf(cj,"%02d:%02d", h, m);
		os_strcpy(buff,cj);
	}
	memset(&cj[0], 0, sizeof(cj));*/



	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}





int ICACHE_FLASH_ATTR cgi_working_time_selector_1(HttpdConnData *connData) {
	int len;
	char buff[1024];
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}
	len=httpdFindArg(connData->post->buff, "sh_on", buff, sizeof(buff));
	if (len!=0) {
		char ch[4];
		char cm[4];
		char ct[6];
		os_sprintf(ch,"%c%c", buff[0],buff[1]);
		os_sprintf(cm,"%c%c", buff[3],buff[4]);
		os_sprintf(ct,"%d",atoi(ch)*60+atoi(cm));
		INFO("\r\n\r\n");
		INFO("ON:");
		INFO(ct);
		INFO("\r\n\r\n");
		os_strcpy(sysCfg.on_time,ct);
		SysCFG_Save();
	}


	len=httpdFindArg(connData->post->buff, "sh_off", buff, sizeof(buff));
	if (len!=0) {
		char ch[4];
		char cm[4];
		char ct[6];
		os_sprintf(ch,"%c%c", buff[0],buff[1]);
		os_sprintf(cm,"%c%c", buff[3],buff[4]);
		os_sprintf(ct,"%d",atoi(ch)*60+atoi(cm));
		INFO("\r\n\r\n");
		INFO("OFF:");
		INFO(ct);
		INFO("\r\n\r\n");
		os_strcpy(sysCfg.off_time,ct);
		SysCFG_Save();
	}

	httpdRedirect(connData, "/working_time_selector/working_time_selector.tpl");
	return HTTPD_CGI_DONE;
}



