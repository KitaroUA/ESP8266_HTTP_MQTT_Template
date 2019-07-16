/*
Some random cgi routines. Used in the LED example and the page that returns the entire
flash as a binary. Also handles the hit counter on the main page.
*/

/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Jeroen Domburg <jeroen@spritesmods.com> wrote this file. As long as you retain 
 * this notice you can do whatever you want with this stuff. If we meet some day, 
 * and you think this stuff is worth it, you can buy me a beer in return. 
 * ----------------------------------------------------------------------------
 */


#include <esp8266.h>
#include "cgi.h"
#include "io.h"
#include "user_config.h"

char v_tempOffMsg[10] = "";



/*

//cause I can't be bothered to write an ioGetLed()
//static char currLedState=0;

uint8 currLedState[1];
//Cgi that turns the LED on or off according to the 'led' param in the POST data
int ICACHE_FLASH_ATTR cgiLed(HttpdConnData *connData) {
	int len;
	char buff[1024];
	currLedState[0]=0;
	
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}

	len=httpdFindArg(connData->post->buff, "led", buff, sizeof(buff));
	if (len!=0) {
		currLedState[0]=atoi(buff);
//		i2c_PCF8574_Write(0x4c,currLedState,1);

//		ioLed(currLedState);
	}

	httpdRedirect(connData, "/led/led.tpl");
	return HTTPD_CGI_DONE;
}



//Template code for the led page.
int ICACHE_FLASH_ATTR tplLed(HttpdConnData *connData, char *token, void **arg) {
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;

	os_strcpy(buff, "Unknown");
	if (os_strcmp(token, "ledstate")==0) {
		if (currLedState) {
			os_strcpy(buff, "on");
		} else {
			os_strcpy(buff, "off");
		}
	}
	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}


int ICACHE_FLASH_ATTR tplshowTime(HttpdConnData *connData, char *token, void **arg) {
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	char cj[80];
	if (os_strcmp(token, "showTime")==0)
	{
		os_sprintf(cj, "%02x:%02x:%02x %02x.%02x.20%02x",  DS3231_Time[2], DS3231_Time[1], DS3231_Time[0], DS3231_Date[0], DS3231_Date[1], DS3231_Date[2]);
		os_strcpy(buff, cj);
	}

	memset(&cj[0], 0, sizeof(cj));
	if (os_strcmp(token, "showHum")==0)
	{
		os_sprintf(cj, "%02d%c", (int)i2c_SHT3X_Hum, 0x25);
		os_strcpy(buff, cj);
	}

	memset(&cj[0], 0, sizeof(cj));
	if (os_strcmp(token, "showTemp")==0)
	{
		os_sprintf(cj, "%02dC", (int)i2c_SHT3X_Temp);
		os_strcpy(buff, cj);
	}
	memset(&cj[0], 0, sizeof(cj));



//	��� � ����� ��� �������� ��� ����� ������ ���� ���������
	os_strcpy(cj, "on_t_");
	os_strcat(cj, sysCfg.on_time);
	if (os_strncmp(token,cj,6)==0)
	{
		if (os_strcmp(token,cj)==0)
		{
			os_strcpy(buff, "selected");
		}
		else
		{
			os_strcpy(buff, "");
		}
	}
	memset(&cj[0], 0, sizeof(cj));

	os_strcpy(cj, "off_t_");
	os_strcat(cj, sysCfg.off_time);
	if (os_strncmp(token,cj,6)==0)
	{
		if (os_strcmp(token,cj)==0)
		{
			os_strcpy(buff, "selected");
		}
		else
		{
			os_strcpy(buff, "");
		}
	}
	memset(&cj[0], 0, sizeof(cj));


	if (os_strcmp(token, "showResult")==0)
	{
		os_strcpy(buff, "");
	}
	memset(&cj[0], 0, sizeof(cj));


	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}

*/


//Template code for the counter on the index page.








/*

int ICACHE_FLASH_ATTR cgi_off_time(HttpdConnData *connData)
{
	os_printf("\r\n \r\n Soff\r\n");

return 1;
}

*/





/*

int ICACHE_FLASH_ATTR cgi_on_time(HttpdConnData *connData)
{
	os_printf("\r\n \r\n Son\r\n");
	int len;
	char buff[1024];
currLedState[0]=0;

	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}

	len=httpdFindArg(connData->post->buff, "on_time", buff, sizeof(buff));
	if (len!=0) {
		char cj[80];
		os_sprintf(cj,"%s",buff);
		i2c_LCD_Send_String(2, 0, cj);

//		ioLed(currLedState);
	}

	httpdRedirect(connData, "time.tpl");
	return HTTPD_CGI_DONE;
}



int ICACHE_FLASH_ATTR cgi_btn_time(HttpdConnData *connData)
{
	int len;
	char buff[1024];
	char cj[80];
	os_printf("\r\n%s\r\n",buff);
	len=httpdFindArg(connData->post->buff, "on_time", buff, sizeof(buff));
	if (len!=0)
	{
		os_strcpy(sysCfg.on_time,buff);
	}
	len=httpdFindArg(connData->post->buff, "off_time", buff, sizeof(buff));
	if (len!=0)
	{
		os_strcpy(sysCfg.off_time,buff);
	}



	httpdRedirect(connData, "time.tpl");
	return HTTPD_CGI_DONE;
}


*/
































/*
 * set_min_light page
 */
int ICACHE_FLASH_ATTR tpl_set_min_light(HttpdConnData *connData, char *token, void **arg)
{
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	char cj[80];
		if (os_strcmp(token,"v_minLight")==0)
		{
			os_sprintf(cj,"\"%d\"",sysCfg.minLight);
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
int ICACHE_FLASH_ATTR cgi_set_min_light(HttpdConnData *connData)
{
	int len;
	char buff[1024];
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}
	len=httpdFindArg(connData->post->buff, "min_light", buff, sizeof(buff));
	if (len!=0) {
		sysCfg.minLight = atoi (buff);
		SysCFG_Save();
	}
	httpdRedirect(connData, "/set_min_light/set_min_light.tpl");
	return HTTPD_CGI_DONE;
}





/*
 * set_off_time page
 */
int ICACHE_FLASH_ATTR tpl_set_off_time(HttpdConnData *connData, char *token, void **arg)
{
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	char cj[80];
	os_strcpy(cj, "off_t_");
	os_strcat(cj, sysCfg.off_time);
	if (os_strncmp(token,cj,6)==0)
	{
		if (os_strcmp(token,cj)==0)
		{
			os_strcpy(buff, "selected");
		}
		else
		{
			os_strcpy(buff, "");
		}
	}
	memset(&cj[0], 0, sizeof(cj));

	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}

int ICACHE_FLASH_ATTR cgi_set_off_time(HttpdConnData *connData)
{
	int len;
	char buff[1024];
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}
	len=httpdFindArg(connData->post->buff, "off_time", buff, sizeof(buff));
	if (len!=0) {
		os_strcpy(sysCfg.off_time,buff);
		SysCFG_Save();
	}
	httpdRedirect(connData, "/set_off_time/set_off_time.tpl");
	return HTTPD_CGI_DONE;
}


/*
 * set_on_time page
 */
int ICACHE_FLASH_ATTR tpl_set_on_time(HttpdConnData *connData, char *token, void **arg)
{
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	char cj[80];
	os_strcpy(cj, "on_t_");
	os_strcat(cj, sysCfg.on_time);
	if (os_strncmp(token,cj,6)==0)
	{
		if (os_strcmp(token,cj)==0)
		{
			os_strcpy(buff, "selected");
		}
		else
		{
			os_strcpy(buff, "");
		}
	}
	memset(&cj[0], 0, sizeof(cj));

	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}

int ICACHE_FLASH_ATTR cgi_set_on_time(HttpdConnData *connData)
{
	int len;
	char buff[1024];
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}
	len=httpdFindArg(connData->post->buff, "on_time", buff, sizeof(buff));
	if (len!=0) {
		os_strcpy(sysCfg.on_time,buff);
		SysCFG_Save();
	}
	httpdRedirect(connData, "/set_on_time/set_on_time.tpl");
	return HTTPD_CGI_DONE;
}


/*
 * set_temp_off page
 */
int ICACHE_FLASH_ATTR tpl_set_temp_off(HttpdConnData *connData, char *token, void **arg)
{
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	char cj[80];
		if (os_strcmp(token,"v_tempOff")==0)
		{
			os_strcpy(cj, "\"");
			os_strcat(cj, sysCfg.tempOff_time);
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

int ICACHE_FLASH_ATTR cgi_set_temp_off(HttpdConnData *connData)
{
	int len;
	char buff[1024];
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}
	len=httpdFindArg(connData->post->buff, "temp_off", buff, sizeof(buff));
	if (len!=0) {
//		temporary_light_off_timer = atoi (sysCfg.tempOff_time)*60;
		os_strcpy(sysCfg.tempOff_time,buff);
		SysCFG_Save();
	}
	httpdRedirect(connData, "/set_temp_off/set_temp_off.tpl");
	return HTTPD_CGI_DONE;
}





















