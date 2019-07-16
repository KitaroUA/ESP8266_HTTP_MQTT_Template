#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include <osapi.h>
#include <os_type.h>
#include <gpio.h>
#include "driver/gpio16.h"
#include <ets_sys.h>
#include "user_interface.h"
#include "driver/time.h"
#include <mem.h>


#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/my_i2c.h"
#include "driver/IIC_DS3231.h"
#include "driver/IIC_PCF8574.h"
//#include "driver/hd44780.h"
#include "driver/IIC_SHT3X.h"
#include "driver/IIC_PCA9685.h"
#include "driver/IIC_mpr121.h"


#include "driver/DS18B20.h"


#include "driver/IIC_SSD1306.h"

#include "driver/IIC_mpr121.h"



uint8 global_light_status;





#include "driver/ntp.h"

#define RODATA_ATTR  __attribute__((section(".irom.text"))) __attribute__((aligned(4)))
#define ROMSTR_ATTR  __attribute__((section(".irom.text.romstr"))) __attribute__((aligned(4)))

// D0 - not used

#define D1pin	5		//	GPIO5
#define D1mux	PERIPHS_IO_MUX_GPIO5_U
#define D1f_g	FUNC_GPIO5	//	gpio function

#define D2pin	4		//	GPIO4
#define D2mux	PERIPHS_IO_MUX_GPIO4_U
#define D2f_g	FUNC_GPIO4	//	gpio function

#define D3pin	0		//	GPIO0
#define D3mux	PERIPHS_IO_MUX_GPIO0_U
#define D3f_g	FUNC_GPIO0	//	gpio function

#define D4pin	2		//	GPIO2
#define D4mux	PERIPHS_IO_MUX_GPIO2_U
#define D4f_g	FUNC_GPIO2	//	gpio function
#define D4f_1tx	FUNC_U1TXD_BK	//	uart 1 tx function
#define D4f_0tx	FUNC_U0TXD_BK	//	uart 0 tx function

#define D5pin	14		//	GPIO14
#define D5mux	PERIPHS_IO_MUX_MTMS_U
#define D5f_g	FUNC_GPIO14	//	gpio function

#define D6pin	12		//	GPIO12
#define D6mux	PERIPHS_IO_MUX_MTDI_U
#define D6f_g	FUNC_GPIO12	//	gpio function

#define D7pin	13		//	GPIO13
#define D7mux	PERIPHS_IO_MUX_MTCK_U
#define D7f_g	FUNC_GPIO13	//	gpio function

#define D8pin	15		//	GPIO15
#define D8mux	PERIPHS_IO_MUX_MTDO_U
#define D8f_g	FUNC_GPIO15	//	gpio function
#define D8f_0rts	FUNC_U0RTS	//	U0rts function

#define S3pin	10		//	GPIO10
#define S3mux	PERIPHS_IO_MUX_SD_DATA3_U
#define S3f_g	FUNC_GPIO10	//	gpio function
#define S3f_sd	FUNC_SDDATA3	//	sd data
#define S3f_spi	FUNC_SPIWP	//	SPI wp

#define S2pin	9		//	GPIO09
#define S2mux	PERIPHS_IO_MUX_SD_DATA2_U
#define S2f_g	FUNC_GPIO9	//	gpio function
#define S2f_sd	FUNC_SDDATA2	//	sd data
#define S2f_spi FUNC_SPIHD	//	SPI hd




/*
	!!!!!!!!!!!!!!!
	!! OS_Timers !!
	!!!!!!!!!!!!!!!
*/
#include "driver/timer_routines.h"

#define WIFI_Button_Pin D5pin
#define WIFI_Button_Mux D5mux
#define WIFI_Button_Func D5f_g

/*
LOCAL os_timer_t blink_timer2;
LOCAL uint8_t led_state2=0;
*/




#define SetTimerTask(Timer, Task, Delay, Repeat)		do	{	\
					os_timer_disarm(&Timer); \
					os_timer_setfn(&Timer, (os_timer_func_t *)Task, NULL);	\
					os_timer_arm(&Timer, Delay, Repeat);	\
					}\
					while (0)




// Relay on GPI
#define RELAY_MUX D8mux
#define RELAY_FUNC D8f_g
#define RELAY_PIN D8pin
uint16 temporary_light_off_timer;
uint16 temporary_light_on_timer;





uint16 light_avg;











/* for MQTT */

#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#include <espconn.h>



#define CFG_HOLDER	0x00FF55A4	/* Change this value to load default configurations */
#define CFG_LOCATION	0x3C	/* Please don't change or if you know what you doing */
#define CLIENT_SSL_ENABLE

/*DEFAULT CONFIGURATIONS*/

#define MQTT_PORT			1883
#define MQTT_BUF_SIZE		1024
#define MQTT_KEEPALIVE		120	 /*second*/

#define MQTT_CLIENT_ID		"DVES_%08X"
#define MQTT_USER			"DVES_USER"
#define MQTT_PASS			"DVES_PASS"


#define MQTT_RECONNECT_TIMEOUT 	5	/*second*/

#define DEFAULT_SECURITY	0
#define QUEUE_BUFFER_SIZE		 		2048

#define PROTOCOL_NAMEv31	/*MQTT version 3.1 compatible with Mosquitto v0.15*/
//PROTOCOL_NAMEv311			/*MQTT version 3.11 compatible with https://eclipse.org/paho/clients/testing/*/

MQTT_Client mqttClient;

/* End for MQTT */



//#define Timezone 3




















//#include "driver/encoder.h"


#include "pages/temperature_selector.h"
#include "pages/options.h"
#include "pages/page_ntp.h"
#include "pages/set_temp_on.h"
#include "pages/working_time_selector.h"
#include "pages/mqtt_settings.h"
#include "pages/set_ip.h"
#include "pages/page_index.h"
#include "pages/slider.h"









uint32 	uptime;

#include "driver/bme280.h"
os_timer_t bme280_sensor_timer;


#include "driver/ws2812.h"



#include "driver/IIC_BH1750FVI.h"






#include "driver/nextion_var.h"




#include "driver/gpio16.h"




#include "driver/BitBang_TLC5947.h"


#include "driver/IIC_ADS1x15.h"


//#define SCAN_IIC
#define SCAN_IIC_first_address 	0x90
#define SCAN_IIC_last_address	0x92




#endif
