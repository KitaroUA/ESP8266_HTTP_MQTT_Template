/*
 * wifi.c
 *
 *  Created on: Dec 30, 2014
 *      Author: Minh
 */
#include "wifi.h"
#include "user_interface.h"
#include "osapi.h"
#include "espconn.h"
#include "os_type.h"
#include "mem.h"
#include "mqtt_msg.h"
#include "debug.h"
#include "user_config.h"
#include "config.h"

static ETSTimer WiFiLinker;
WifiCallback wifiCb = NULL;
static uint8_t wifiStatus = STATION_IDLE, lastWifiStatus = STATION_IDLE;
static void ICACHE_FLASH_ATTR wifi_check_ip(void *arg)
{
	struct ip_info ipConfig;

	os_timer_disarm(&WiFiLinker);
	wifi_get_ip_info(STATION_IF, &ipConfig);
	wifiStatus = wifi_station_get_connect_status();
	if (wifiStatus == STATION_GOT_IP && ipConfig.ip.addr != 0)
	{

		os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
		os_timer_arm(&WiFiLinker, 2000, 0);


	}
	else
	{
		if(wifi_station_get_connect_status() == STATION_WRONG_PASSWORD)
		{

			INFO("STATION_WRONG_PASSWORD\r\n");
			wifi_station_connect();


		}
		else if(wifi_station_get_connect_status() == STATION_NO_AP_FOUND)
		{

			INFO("STATION_NO_AP_FOUND\r\n");
			wifi_station_connect();


		}
		else if(wifi_station_get_connect_status() == STATION_CONNECT_FAIL)
		{

			INFO("STATION_CONNECT_FAIL\r\n");
			wifi_station_connect();

		}
		else
		{
			INFO("STATION_IDLE\r\n");
		}

		os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
		os_timer_arm(&WiFiLinker, 500, 0);
	}
	if(wifiStatus != lastWifiStatus){
		lastWifiStatus = wifiStatus;
		if(wifiCb)
			wifiCb(wifiStatus);
	}
}

void ICACHE_FLASH_ATTR WIFI_Connect(uint8_t* ssid, uint8_t* pass, WifiCallback cb)
{
	struct station_config stationConf;

	INFO("WIFI_INIT\r\n");
	wifi_set_opmode(STATION_MODE);
	wifi_station_set_auto_connect(FALSE);
	wifiCb = cb;

	os_memset(&stationConf, 0, sizeof(struct station_config));

	INFO("WIFI config: \r\n");
	os_sprintf(stationConf.ssid, "%s", ssid);
	os_sprintf(stationConf.password, "%s", pass);
	INFO("\r\n%s", ssid);
	INFO("\r\n%s\r\n", pass);
	INFO("\r\n%d\r\n", sysCfg.dhcp_sta_enabled);

	wifi_station_set_config(&stationConf);

	if (sysCfg.dhcp_sta_enabled == 0)
	{

	struct ip_info ipinfo;

	wifi_station_dhcpc_stop();
	wifi_softap_dhcps_stop();


//	IP4_ADDR(&ipinfo.ip, 192, 168, 104, 200);
	ipinfo.ip=sysCfg.staipinfo.ip;
//	IP4_ADDR(&ipinfo.gw, 192, 168, 104, 1);
	ipinfo.gw=sysCfg.staipinfo.gw;
//	IP4_ADDR(&ipinfo.netmask, 255, 255, 255, 0);
	ipinfo.netmask=sysCfg.staipinfo.netmask;
	wifi_set_ip_info(STATION_IF, &ipinfo);


//	IP4_ADDR(&ipinfo.ip, 192, 168, 4, 1);
	ipinfo.ip=sysCfg.apipinfo.ip;
//	IP4_ADDR(&ipinfo.gw, 192, 168, 4, 1);
	ipinfo.gw=sysCfg.apipinfo.gw;
//	IP4_ADDR(&ipinfo.netmask, 255, 255, 255, 0);
	ipinfo.netmask=sysCfg.apipinfo.netmask;
	wifi_set_ip_info(SOFTAP_IF, &ipinfo);

	wifi_softap_dhcps_start();
	}
	else
	{
		wifi_station_dhcpc_stop();
		wifi_softap_dhcps_stop();

		wifi_station_dhcpc_start();
		wifi_softap_dhcps_start();

	}

	os_timer_disarm(&WiFiLinker);
	os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
	os_timer_arm(&WiFiLinker, 1000, 0);

	wifi_station_set_auto_connect(TRUE);

	wifi_station_connect();


}

