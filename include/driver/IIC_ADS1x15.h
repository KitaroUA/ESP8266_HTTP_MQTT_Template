/*
 * IIC_ADS1x15.h
 *
 *  Created on: 30 січ. 2019 р.
 *      Author: Kitaro
 */

#ifndef INCLUDE_DRIVER_IIC_ADS1X15_H_
#define INCLUDE_DRIVER_IIC_ADS1X15_H_

#include "user_config.h"





/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADS1015_ADDRESS                 (0x90)    // 1001 000 shifted left 1 bit = 0x90 (ADDR = GND)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1015_REG_POINTER_MASK        (0x03)
    #define ADS1015_REG_POINTER_CONVERT     (0x00)
    #define ADS1015_REG_POINTER_CONFIG      (0x01)
    #define ADS1015_REG_POINTER_LOWTHRESH   (0x02)
    #define ADS1015_REG_POINTER_HITHRESH    (0x03)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1015_REG_CONFIG_OS_MASK      (0x8000)
    #define ADS1015_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
    #define ADS1015_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress
    #define ADS1015_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion

    #define ADS1015_REG_CONFIG_MUX_MASK     (0x7000)
    #define ADS1015_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
    #define ADS1015_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
    #define ADS1015_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
    #define ADS1015_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
    #define ADS1015_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
    #define ADS1015_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
    #define ADS1015_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
    #define ADS1015_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

    #define ADS1015_REG_CONFIG_PGA_MASK     (0x0E00)
    #define ADS1015_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range	1 bit = 3mV      0.1875mV (default)
    #define ADS1015_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range	1 bit = 2mV      0.125mV
    #define ADS1015_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range 	1 bit = 1mV      0.0625mV
    #define ADS1015_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range	1 bit = 0.5mV    0.03125mV
    #define ADS1015_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range	1 bit = 0.25mV   0.015625mV
    #define ADS1015_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range 	1 bit = 0.125mV  0.0078125mV

    #define ADS1015_REG_CONFIG_MODE_MASK    (0x0100)
    #define ADS1015_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode
    #define ADS1015_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

    #define ADS1015_REG_CONFIG_DR_MASK      (0x00E0)
    #define ADS1015_REG_CONFIG_DR_128SPS    (0x0000)  // 128 samples per second
    #define ADS1015_REG_CONFIG_DR_250SPS    (0x0020)  // 250 samples per second
    #define ADS1015_REG_CONFIG_DR_490SPS    (0x0040)  // 490 samples per second
    #define ADS1015_REG_CONFIG_DR_920SPS    (0x0050)  // 920 samples per second
    #define ADS1015_REG_CONFIG_DR_1600SPS   (0x0080)  // 1600 samples per second (default)
    #define ADS1015_REG_CONFIG_DR_2400SPS   (0x00A0)  // 2400 samples per second
    #define ADS1015_REG_CONFIG_DR_3300SPS   (0x00C0)  // 3300 samples per second

    #define ADS1015_REG_CONFIG_CMODE_MASK   (0x0010)
    #define ADS1015_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
    #define ADS1015_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

    #define ADS1015_REG_CONFIG_CPOL_MASK    (0x0008)
    #define ADS1015_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
    #define ADS1015_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

    #define ADS1015_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
    #define ADS1015_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
    #define ADS1015_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

    #define ADS1015_REG_CONFIG_CQUE_MASK    (0x0003)
    #define ADS1015_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions
    #define ADS1015_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
    #define ADS1015_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
    #define ADS1015_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/

typedef enum
{
  ADS1015_ERROR_OK = 0,                // Everything executed normally
  ADS1015_ERROR_I2CINIT,               // Unable to initialise I2C
  ADS1015_ERROR_I2CBUSY,               // I2C already in use
  ADS1015_ERROR_INVALIDCHANNEL,        // Invalid channel specified
  ADS1015_ERROR_LAST
}
ads1015Error_t;

ads1015Error_t ads1015Init(void);
void ads1015ReadADC_SingleRead(void);
ads1015Error_t ads1015ReadADC_Differential_0_1(int16_t *value);
ads1015Error_t ads1015ReadADC_Differential_2_3(int16_t *value);
ads1015Error_t ads1015StartComparator_SingleEnded(uint8_t channel, int16_t threshold);
ads1015Error_t ads1015GetLastConversionResults(int16_t *value);

uint16 ads1015_data[4];
uint8 ads1015_channel_to_read;
uint8 ADS1015_REG_GAIN_selector; // 0 - mask, 6V,4V,2V,1V,0.5V,0.25V

uint16 ADS1015_REG_GAIN_Array[7];
float ADS1015_REG_volt_converter_Array[7];

os_timer_t ads1015_timer;



#endif /* INCLUDE_DRIVER_IIC_ADS1X15_H_ */
