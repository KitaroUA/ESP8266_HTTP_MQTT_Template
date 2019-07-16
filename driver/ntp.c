//////////////////////////////////////////////////
// Simple NTP client for ESP8266.
// Copyright 2015 Richard A Burton
// richardaburton@gmail.com
// See license.txt for license terms.
//////////////////////////////////////////////////



#include "driver\ntp.h"
#include "driver\IIC_DS3231.h"

// list of major public servers http://tf.nist.gov/tf-cgi/servers.cgi
//uint8 ntp_server[] = {131, 107, 13, 100}; // microsoft
uint8 ntp_server[4] = {193, 106, 144, 7}; // microsoft


static os_timer_t ntp_timeout;
static struct espconn *pCon = 0;

// convert normal decimal to binary coded decimal
static uint8 ICACHE_FLASH_ATTR decToBcd(uint8 dec) {
  return(((dec / 10) * 16) + (dec % 10));
}

// convert binary coded decimal to normal decimal
static uint8 ICACHE_FLASH_ATTR bcdToDec(uint8 bcd) {
  return(((bcd / 16) * 10) + (bcd % 16));
}

bool ICACHE_FLASH_ATTR check_dst(struct tm *time) {

	bool dst = false;


	mktime(time);
    if (time->tm_mon < 2 || time->tm_mon > 9) {
		// these months are completely out of DST
	} else if (time->tm_mon > 2 && time->tm_mon < 9) {
		// these months are completely in DST
		dst = true;
	} else {
		// else we must be in one of the change months
		// work out when the last sunday was (could be today)
		int previousSunday = time->tm_mday - time->tm_wday;
		if (time->tm_mon == 2) { // march
			// was last sunday (which could be today) the last sunday in march
			if (previousSunday >= 25) {
				// are we actually on the last sunday today
				if (time->tm_wday == 0) {
					// if so are we at/past 2am gmt
					int s = (time->tm_hour * 3600) + (time->tm_min * 60) + time->tm_sec;
					//					if (s >= 7200)  // UTC dst time
					if (s >= 10800)  // local dst time
						{
						dst = true;
						}
				} else {
					dst = true;
				}
			}
		} else if (time->tm_mon == 9) {
			// was last sunday (which could be today) the last sunday in october
			if (previousSunday >= 25) {
				// we have reached/passed it, so is it today?
				if (time->tm_wday == 0) {
					// change day, so are we before 1am gmt (2am localtime)
					int s = (time->tm_hour * 3600) + (time->tm_min * 60) + time->tm_sec;
					if (s < 14400) dst = true;
				}
			} else {
				// not reached the last sunday yet
				dst = true;
			}
		}
	}
	if (dst) {
		time->tm_isdst = 1;
	} else {
		time->tm_isdst = 0;
	}

return dst;

}



void ICACHE_FLASH_ATTR applyTZ(struct tm *time) {

	bool dst = false;

	// apply base timezone offset
	//time->tm_hour += 1; // e.g. central europe

	// call mktime to fix up (needed if applying offset has rolled the date back/forward a day)
	// also sets yday and fixes wday (if it was wrong from the rtc)
	dst = check_dst(time);

	// work out if we should apply dst, modify according to your local rules

	if (dst) {
		// add the dst hour
		time->tm_hour += 1;
		// mktime will fix up the time/date if adding an hour has taken us to the next day
		mktime(time);
		// don't rely on isdst returned by mktime, it doesn't know about timezones and tends to reset this to 0
		time->tm_isdst = 1;
	} else {
		time->tm_isdst = 0;
	}

}





static void ICACHE_FLASH_ATTR ntp_udp_timeout(void *arg) {
	
	os_timer_disarm(&ntp_timeout);
	uart0_tx_buffer("ntp timout\r\n", 12);
	os_printf("ntp timeout");
	// clean up connection
	if (pCon) {
		espconn_delete(pCon);
		os_free(pCon->proto.udp);
		os_free(pCon);
		pCon = 0;
	}
}

static void ICACHE_FLASH_ATTR ntp_udp_recv(void *arg, char *pdata, unsigned short len) {
	struct tm *dt;
	time_t timestamp;
	ntp_t *ntp;

	os_timer_disarm(&ntp_timeout);
	// extract ntp time
	ntp = (ntp_t*)pdata;
	timestamp = ntp->trans_time[0] << 24 | ntp->trans_time[1] << 16 |ntp->trans_time[2] << 8 | ntp->trans_time[3];
	// convert to unix time
	timestamp -= 2208988800UL;
	// create tm struct












	dt = gmtime(&timestamp);

	if (dt->tm_wday == 0)
	{
		dt->tm_wday = 7; //� NTP ���� ����� �� 0 �� 6 ��������� � �����, ��������� � ����� �� 1 �� 7
	}
	// do something with it, like setting an rtc
	//ds1307_setTime(dt);
	// or just print it out

	os_printf("\r\n");
	os_printf("\r\n");
	os_printf("\r\n");

	char cj[80];



	os_sprintf(cj, "gmt: %d %02d:%02d:%02d %02d.%02d.%04d, %d",  dt->tm_isdst, dt->tm_hour, dt->tm_min, dt->tm_sec, dt->tm_mday, dt->tm_mon + 1, dt->tm_year + 1900, dt->tm_wday);
	os_printf(cj);
	os_printf("\r\n");


//	applyTZ(dt);

	os_printf("\r\n");
	os_printf("\r\n");
	os_printf("\r\n");

//	dt->tm_hour += sysCfg.timezone;
//	mktime(dt);
	_daylight = sysCfg.dst_flag;                 // Non-zero if daylight savings time is used
	_dstbias = 3600;                  			// Offset for Daylight Saving Time
	_timezone = 0 - (sysCfg.timezone*3600);      // Difference in seconds between GMT and local time

	dt = localtime(&timestamp);
	applyTZ(dt);


	if (dt->tm_wday == 0)
	{
		dt->tm_wday = 7; //� NTP ���� ����� �� 0 �� 6 ��������� � �����, ��������� � ����� �� 1 �� 7
	}

	os_sprintf(cj, "local time: %d %02d:%02d:%02d %02d.%02d.%04d, %d",  dt->tm_isdst, dt->tm_hour, dt->tm_min, dt->tm_sec, dt->tm_mday, dt->tm_mon + 1, dt->tm_year + 1900, dt->tm_wday);
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

	// clean up connection
	if (pCon) {
		espconn_delete(pCon);
		os_free(pCon->proto.udp);
		os_free(pCon);
		pCon = 0;
	}
}

void ICACHE_FLASH_ATTR ntp_get_time() {

	ntp_t ntp;
	os_printf("\r\n");
	os_printf("ntp start");
	os_printf("%d.%d.%d.%d", ntp_server[0], ntp_server[1], ntp_server[2], ntp_server[3]);

	// set up the udp "connection"
	pCon = (struct espconn*)os_zalloc(sizeof(struct espconn));
	pCon->type = ESPCONN_UDP;
	pCon->state = ESPCONN_NONE;
	pCon->proto.udp = (esp_udp*)os_zalloc(sizeof(esp_udp));
	pCon->proto.udp->local_port = espconn_port();
	pCon->proto.udp->remote_port = 123;
	os_memcpy(pCon->proto.udp->remote_ip, ntp_server, 4);

	// create a really simple ntp request packet
	os_memset(&ntp, 0, sizeof(ntp_t));
	ntp.options = 0b00100011; // leap = 0, version = 4, mode = 3 (client)

	// set timeout timer
	os_timer_disarm(&ntp_timeout);
	os_timer_setfn(&ntp_timeout, (os_timer_func_t*)ntp_udp_timeout, pCon);
	os_timer_arm(&ntp_timeout, NTP_TIMEOUT_MS, 0);

	// send the ntp request
	espconn_create(pCon);
	espconn_regist_recvcb(pCon, ntp_udp_recv);
	espconn_sent(pCon, (uint8*)&ntp, sizeof(ntp_t));
}



