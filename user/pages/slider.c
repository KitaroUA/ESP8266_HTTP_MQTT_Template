/*
 * slider.c
 *
 *  Created on: 2 січ. 2019 р.
 *      Author: Kitaro
 */

#include "pages/page_ntp.h"
#include <esp8266.h>
#include "io.h"
#include "user_config.h"

uint8_t slider_pos = 0;

//#define debug_slider


int ICACHE_FLASH_ATTR tpl_slider(HttpdConnData *connData, char *token, void **arg)
{
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	char cj[80];
		if (os_strcmp(token,"v_slider_pos")==0)
		{
			sprintf(cj,"%d",slider_pos);
			os_sprintf(buff, cj);
		}
		else
		{
			os_strcpy(buff, "0");
		}
	memset(&cj[0], 0, sizeof(cj));

	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}
int ICACHE_FLASH_ATTR cgi_set_slider(HttpdConnData *connData)
{
	int len;
	char buff[1024];
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}
	len=httpdFindArg(connData->post->buff, "slider_pos", buff, sizeof(buff));
	if (len!=0) {
//		temporary_light_on_timer = atoi (sysCfg.tempOn_time)*60;
#ifdef debug_slider
//		INFO("\r\n\buff: %s\r\n",buff);
		INFO("\r\n\buff: %s\r\n",(slider_pos*4096)/100);
#endif
		slider_pos = atoi (buff);
//		i2c_PCA9685_Servo_Simple(1,0,slider_pos);

/*
 * 	uint8_t ii;
		for (ii=0;ii<24;ii++)
		{
		BitBang_TLC5947_setPWM(ii,(slider_pos*4096)/100);
		}
		BitBang_TLC5947_write();
*/
//		SysCFG_Save();
	}
	httpdRedirect(connData, "/set_temp_on/set_temp_on.tpl");
	return HTTPD_CGI_DONE;
}
