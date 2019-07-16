/*
 * set_temp_on.c
 *
 *  Created on: 2 жовт. 2017 р.
 *      Author: Kitaro
 */


#include <esp8266.h>
#include "pages/options.h"
#include "io.h"
#include "user_config.h"


int ICACHE_FLASH_ATTR tpl_set_temp_on(HttpdConnData *connData, char *token, void **arg)
{
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	char cj[80];
		if (os_strcmp(token,"v_tempOn")==0)
		{
			os_strcpy(cj, "\"");
			os_strcat(cj, sysCfg.tempOn_time);
			os_strcat(cj, "\"");
			os_sprintf(buff, cj);
		}
		else
		{
			os_strcpy(buff, "");
		}
	memset(&cj[0], 0, sizeof(cj));

	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}

int ICACHE_FLASH_ATTR cgi_set_temp_on(HttpdConnData *connData)
{
	int len;
	char buff[1024];
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}
	len=httpdFindArg(connData->post->buff, "temp_on", buff, sizeof(buff));
	if (len!=0) {
//		temporary_light_on_timer = atoi (sysCfg.tempOn_time)*60;
		os_strcpy(sysCfg.tempOn_time,buff);
		SysCFG_Save();
	}
	httpdRedirect(connData, "/set_temp_on/set_temp_on.tpl");
	return HTTPD_CGI_DONE;
}
