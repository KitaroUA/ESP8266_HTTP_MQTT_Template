/* config.h
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef USER_CONFIG_H_
#define USER_CONFIG_H_
#include "os_type.h"
#include "user_config.h"
#include "ip_addr.h"


#define number_of_temperature_options_channels 4
#define number_of_relay_channels 4 //	PCF8574 1...8
#define number_of_PWM_channels 5 //	PCA9685 1...16
#define temperature_options_off 255
typedef struct {
    uint8 temparture_sensor;	//from temperature sensors array
    							//if 0xFF - channel is disabled
    uint8 control_channel;		//from external control array 0..99 - Relay, 100..199 - PWM, if 0xFF - channel is disabled
	int on_temperature;		//temperature, when to turn on external load, or to start/lower dimming
	int off_temperature;		//temperature, when to turn off external load, or for upper dimmer
	uint8 lower_dimmer_value;	//
	uint8 upper_dimmer_value;
} Temperature_selector_struct;



typedef struct{
	uint8_t device_id[16];

	uint8_t sta_ssid[64];
	uint8_t sta_pwd[64];
	uint32_t sta_type;

	uint8_t mqtt_host[64];
	uint32_t mqtt_port;
	uint8_t mqtt_user[32];
	uint8_t mqtt_pass[32];
	uint32_t mqtt_keepalive;
	uint8_t security;
	uint8_t dhcp_sta_enabled; // 1 - true
	struct ip_info staipinfo;
	struct ip_info apipinfo;
	uint8 tempOn_time[3];
	uint8 tempOff_time[3];
	int8  timezone;
	uint8 hostname[30];
	uint8 ntp[30];
	uint8 ntp_flag;
	uint8 dst_flag;
	uint8 dst_active;
	uint8 mqtt_task_enabled;
	uint8 try;
	uint32 minLight;
    uint8 on_time[5];
	uint8 off_time[5];
	Temperature_selector_struct Temperature_selector_array[number_of_temperature_options_channels];
	uint8 Temperature_display_array[2] ;



} SYSCFG;

typedef struct {
    uint8 flag;
    uint8 pad[3];
} SAVE_FLAG;




typedef struct {
    uint8 flag;
    uint8 on_time[5];
	uint8 off_time[5];
	uint8 tempOff_time[5];
	uint8 tempOn_time[5];
	uint32 minLight;
	int8 timezone;
	uint8 hostname[30];
	uint8 ntp[30];
	uint8 ntp_flag;
	uint8 dst_flag;
	uint8 dst_active;
	Temperature_selector_struct Temperature_selector_array[number_of_temperature_options_channels];
	uint8 Temperature_display_array[2] ;
	uint8 mqtt_server[30];
	uint8 mqtt_task_enabled;
	uint8 try;
} MY_FLASH_STR;



void ICACHE_FLASH_ATTR SysCFG_Save();
void ICACHE_FLASH_ATTR SysCFG_Load();

extern SYSCFG sysCfg;
//extern MY_FLASH_STR sysCfg;

void ICACHE_FLASH_ATTR SysCFG_Save (void);
void ICACHE_FLASH_ATTR AddCFG_Load  (void);
void ICACHE_FLASH_ATTR Default_CFG(void);



#define STA_SSID "Power AP"
#define STA_PASS "shall we dance"
#define STA_TYPE AUTH_WPA2_PSK

#define sta_ip_a 192
#define sta_ip_b 168
#define sta_ip_c 104
#define sta_ip_d 202

#define sta_gw_a 192
#define sta_gw_b 168
#define sta_gw_c 104
#define sta_gw_d 1

#define sta_nm_a 255
#define sta_nm_b 255
#define sta_nm_c 255
#define sta_nm_d 0



#define sap_ip_a 192
#define sap_ip_b 168
#define sap_ip_c 4
#define sap_ip_d 1

#define sap_gw_a 192
#define sap_gw_b 168
#define sap_gw_c 4
#define sap_gw_d 1

#define sap_nm_a 255
#define sap_nm_b 255
#define sap_nm_c 255
#define sap_nm_d 0

#define default_mqtt_host "192.168.104.100"

#endif /* USER_CONFIG_H_ */
