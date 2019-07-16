/*
 * ntp.c
 *
 *  Created on: 30 жовт. 2017 р.
 *      Author: Kitaro
 */

#include "pages/page_ntp.h"
#include <esp8266.h>
#include "io.h"
#include "user_config.h"


char v_ntp[20] = "";
char v_hostname[20] = "";
char v_timezone[3] = "";
static char v_ntpresult[50] = "";



static uint8 ICACHE_FLASH_ATTR decToBcd(uint8 dec) {
  return(((dec / 10) * 16) + (dec % 10));
}

// convert binary coded decimal to normal decimal
static uint8 ICACHE_FLASH_ATTR bcdToDec(uint8 bcd) {
  return(((bcd / 16) * 10) + (bcd % 16));
}


/*
 * set_ntp page
 */
int ICACHE_FLASH_ATTR tpl_set_ntp(HttpdConnData *connData, char *token, void **arg)
{

	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	char cj[80];
	if (os_strcmp(token,"v_hostname")==0)
	{
//		os_sprintf(buff,"\"%s\"", sysCfg.hostname);
		os_strcpy(buff,sysCfg.hostname);
	}
	memset(&cj[0], 0, sizeof(cj));


	if (os_strcmp(token,"v_ntp")==0)
	{
//		os_sprintf(buff, "\"%s\"", sysCfg.ntp);
		os_strcpy(buff, sysCfg.ntp);
	}
	memset(&cj[0], 0, sizeof(cj));


	if (os_strcmp(token,"v_timezone")==0)
	{
		os_sprintf(cj,"%d", sysCfg.timezone);
		os_sprintf(buff, cj);
	}
	memset(&cj[0], 0, sizeof(cj));

	if (os_strcmp(token,"ntp_type_1")==0)
	{
		if (sysCfg.ntp_flag == 0)
		{
		os_sprintf(cj,"checked");
		}
		else
		{
		os_sprintf(cj," ");
		}
		os_sprintf(buff, cj);
	}
	memset(&cj[0], 0, sizeof(cj));


	if (os_strcmp(token,"ntp_type_2")==0)
	{
//		os_strcat(cj, sysCfg.tempOff_time);
		if (sysCfg.ntp_flag == 1)
		{
		os_sprintf(cj,"checked");
		}
		else
		{
		os_sprintf(cj," ");
		}
		os_sprintf(buff, cj);
	}
	memset(&cj[0], 0, sizeof(cj));



	if (os_strcmp(token,"DST_1")==0)
	{
		if (sysCfg.dst_flag == 1)
		{
		os_sprintf(cj,"checked");
		}
		else
		{
		os_sprintf(cj," ");
		}
		os_sprintf(buff, cj);
	}
	memset(&cj[0], 0, sizeof(cj));


	if (os_strcmp(token,"DST_2")==0)
	{
		if (sysCfg.dst_flag == 0)
		{
		os_sprintf(cj,"checked");
		}
		else
		{
		os_sprintf(cj," ");
		}
		os_sprintf(buff, cj);
	}
	memset(&cj[0], 0, sizeof(cj));

	if (os_strcmp(token,"v_ntpresult")==0)
	{
//		os_strcat(cj, sysCfg.tempOff_time);

		os_sprintf(cj,v_ntpresult);
		os_sprintf(buff, cj);
	}
	os_printf(v_ntpresult, "");
	memset(&cj[0], 0, sizeof(cj));




	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}

int ICACHE_FLASH_ATTR cgi_set_ntp(HttpdConnData *connData)
{
	int8 timezone = 0;
	uint8 ntp[30] = "";


	int len;
	char buff[1024];
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}

	len=httpdFindArg(connData->post->buff, "hn", buff, sizeof(buff));
	if (len!=0)
	{
		os_strcpy(sysCfg.hostname, buff);
	}
	memset(&buff[0], 0, sizeof(buff));

	len=httpdFindArg(connData->post->buff, "ntpip", buff, sizeof(buff));
	if (len!=0)
	{
		os_strcpy(sysCfg.ntp, buff);
//		os_strcpy(ntp, buff);
	}
	memset(&buff[0], 0, sizeof(buff));

	len=httpdFindArg(connData->post->buff, "timezone", buff, sizeof(buff));
	if (len!=0)
	{
		sysCfg.timezone = atoi(buff);
//		timezone = atoi(buff);
	}
	memset(&buff[0], 0, sizeof(buff));

	len=httpdFindArg(connData->post->buff, "DST", buff, sizeof(buff));
	if (len!=0)
	{
		if(os_strcmp(buff,"1") == 0)
		{
			sysCfg.dst_flag = 1;
		}
		else
		{
			sysCfg.dst_flag = 0;
		}
	}
	memset(&buff[0], 0, sizeof(buff));

	len=httpdFindArg(connData->post->buff, "ntp_type", buff, sizeof(buff));
	if (len!=0)
	{
		if(os_strcmp(buff,"1") == 0)
		{
			sysCfg.ntp_flag = 1;
		}
		else
		{
			sysCfg.ntp_flag = 0;
		}
	}
	memset(&buff[0], 0, sizeof(buff));
	/*
			ip_addr_t ntp_addr;
			os_printf("\r\n");
			ntp_addr.addr = ipaddr_addr(sysCfg.ntp);
			os_printf("\r\n");
			os_printf(IPSTR,IP2STR(&ntp_addr));
			os_printf("\r\n");
			os_printf("%d",ip4_addr1_16(&ntp_addr));
			os_printf("\r\n");
			os_printf("%d",ip4_addr2_16(&ntp_addr));
			os_printf("\r\n");
			os_printf("%d",ip4_addr3_16(&ntp_addr));
			os_printf("\r\n");
			os_printf("%d",ip4_addr4_16(&ntp_addr));
			os_printf("\r\n");
	*/
	//�������� �������� �����

//	sysCfg.timezone = timezone;
//	os_strcpy(sysCfg.ntp, ntp);
	SysCFG_Save();

	//������ NTP
	httpdRedirect(connData, "/set_ntp/set_ntp.tpl");
	return HTTPD_CGI_DONE;
}



static void ICACHE_FLASH_ATTR ntpDNSfound(const char *name, ip_addr_t *ip, void *arg) {
	char ntp_ip_from_dns[16] ="";
  static esp_tcp tcp;
  os_printf("\r\n\r\n\r\n");
  struct espconn *conn=(struct espconn *)arg;
  if (ip==NULL) {
    os_printf("Nslookup failed :/ Trying again...\n");
    os_printf("lfai");
    memset(&ntp_ip_from_dns[0], 0, sizeof(ntp_ip_from_dns));
  }
  else{
  os_printf("lokk");
  char page_buffer[20];
  os_sprintf(page_buffer,"DST: %d.%d.%d.%d",
  *((uint8 *)&ip->addr), *((uint8 *)&ip->addr + 1),
  *((uint8 *)&ip->addr + 2), *((uint8 *)&ip->addr + 3));
  os_printf(page_buffer);
  os_sprintf(ntp_ip_from_dns,"%d.%d.%d.%d",
  *((uint8 *)&ip->addr), *((uint8 *)&ip->addr + 1),
  *((uint8 *)&ip->addr + 2), *((uint8 *)&ip->addr + 3));
  }
	ip_addr_t ntp_addr;
	os_printf("\r\n");
	ntp_addr.addr = ipaddr_addr(ntp_ip_from_dns);
	ntp_server[0] = ip4_addr1_16(&ntp_addr);
	ntp_server[1] = ip4_addr2_16(&ntp_addr);
	ntp_server[2] = ip4_addr3_16(&ntp_addr);
	ntp_server[3] = ip4_addr4_16(&ntp_addr);

	ntp_get_time();
}



int ICACHE_FLASH_ATTR cgi_apply_ntp(HttpdConnData *connData)
{
	int8 timezone = 0;
	uint8 ntp[30] = "";
	uint8 v_error[50] = "";

	os_printf("\r\napply_ntp\r\n");

	int len;
	char buff[1024];
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}


	if (sysCfg.ntp_flag == 1)
	{// ntp addr - DNS
		os_printf("\r\ntp_flag_1\r\n");

		static struct espconn conn;
		static ip_addr_t ip;
		espconn_gethostbyname(&conn, sysCfg.ntp, &ip, ntpDNSfound);
	}
	else
	{
		os_printf("\r\ntp_flag_0\r\n");
		ip_addr_t ntp_addr;
		ntp_addr.addr = ipaddr_addr(sysCfg.ntp);
		ntp_server[0] = ip4_addr1_16(&ntp_addr);
		ntp_server[1] = ip4_addr2_16(&ntp_addr);
		ntp_server[2] = ip4_addr3_16(&ntp_addr);
		ntp_server[3] = ip4_addr4_16(&ntp_addr);
		ntp_get_time();
	}
	ntp_get_time();

	httpdRedirect(connData, "/set_ntp/set_ntp.tpl");
	return HTTPD_CGI_DONE;

}



int ICACHE_FLASH_ATTR cgi_PC_Time(HttpdConnData *connData)
{
	int8 timezone = 0;
	uint8 ntp[30] = "";


	int len;
	char buff[1024];
	if (connData->conn==NULL) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}

	len=httpdFindArg(connData->post->buff, "data", buff, sizeof(buff));
	if (len!=0)
	{
//		INFO(buff);
//		INFO("\r\n\r\n");


		struct tm *dt;
		time_t timestamp = 1522921929;
//		timestamp = 1522921929;
		dt = gmtime(&timestamp);



		char * pch;
		pch = strtok (buff,",");
		dt->tm_year = atoi (pch) - 1900;
//		INFO("y:");
//		INFO(pch);
//		INFO("\r\n\r\n");

		pch = strtok (NULL, ",");
//		dt->tm_mon = atoi (pch) - 1;
		dt->tm_mon = atoi (pch);
//		INFO("m:");
//		INFO(pch);
//		INFO("\r\n\r\n");

		pch = strtok (NULL, ",");
		dt->tm_mday = atoi (pch);
//		INFO("d:");
//		INFO(pch);
//		INFO("\r\n\r\n");

		pch = strtok (NULL, ",");
		dt->tm_hour = atoi (pch);
//		INFO("h:");
//		INFO(pch);
//		INFO("\r\n\r\n");

		pch = strtok (NULL, ",");
		dt->tm_min = atoi (pch);
//		INFO("m:");
//		INFO(pch);
//		INFO("\r\n\r\n");

		pch = strtok (NULL, ",");
		dt->tm_sec = atoi (pch);
//		INFO("s:");
//		INFO(pch);
//		INFO("\r\n\r\n");

		pch = strtok (NULL, ",");
		dt->tm_wday = atoi (pch);
//		INFO("wd:");
//		INFO(pch);
//		INFO("\r\n\r\n");

		dt->tm_hour += sysCfg.timezone;

		applyTZ(dt);


		if (dt->tm_wday == 7)
		{
			dt->tm_wday = 0; //� NTP ���� ����� �� 0 �� 6 ��������� � �����, ��������� � ����� �� 1 �� 7
		}

		DS3231_Date_to_Write[0] = decToBcd(dt->tm_sec);
		DS3231_Date_to_Write[1] = decToBcd(dt->tm_min);
		DS3231_Date_to_Write[2] = decToBcd(dt->tm_hour);
		DS3231_Date_to_Write[3] = decToBcd(dt->tm_wday);
		DS3231_Date_to_Write[4] = decToBcd(dt->tm_mday);
		DS3231_Date_to_Write[5] = decToBcd(dt->tm_mon + 1);
		DS3231_Date_to_Write[6] = decToBcd(dt->tm_year - 100);
		DS3231_Set_Date();


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
//	len=sprintf(output, "%s", buff);
	//Send HTML body to webbrowser. We use the length as calculated by sprintf here.
	//Using -1 again would also have worked, but this is more efficient.
//	httpdSend(connData, output, len);




	return HTTPD_CGI_DONE;
}
