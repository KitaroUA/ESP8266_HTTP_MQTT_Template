/*
 * IIC_ADS1x15.c
 *
 *  Created on: 30 січ. 2019 р.
 *      Author: Kitaro
 */


#include "driver/IIC_ADS1x15.h"



#define HI(X) (X>>8)
#define LO(X) (X & 0xFF)

void ICACHE_FLASH_ATTR ads1015ReadADC_SingleRead_01 (void);
void ICACHE_FLASH_ATTR ads1015ReadADC_SingleReaded (void);

uint16 ADS1015_REG_GAIN_Array[] = {
ADS1015_REG_CONFIG_PGA_MASK,
ADS1015_REG_CONFIG_PGA_6_144V,// +/-6.144V range	1 bit = 3mV      0.1875mV (default)
ADS1015_REG_CONFIG_PGA_4_096V,// +/-4.096V range	1 bit = 2mV      0.125mV
ADS1015_REG_CONFIG_PGA_2_048V,// +/-2.048V range 	1 bit = 1mV      0.0625mV
ADS1015_REG_CONFIG_PGA_1_024V,// +/-1.024V range	1 bit = 0.5mV    0.03125mV
ADS1015_REG_CONFIG_PGA_0_512V,// +/-0.512V range	1 bit = 0.25mV   0.015625mV
ADS1015_REG_CONFIG_PGA_0_256V,// +/-0.256V range 	1 bit = 0.125mV  0.0078125mV
};
float ADS1015_REG_volt_converter_Array[] =
		{
		0,3,2,1,0.5,0.25,0.125
		};


/*
extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

static bool _ads1015Initialised = false;
*/

/**************************************************************************/
/*!
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/

/*
static ads1015Error_t ads1015WriteRegister (uint8_t reg, uint16_t value)
{
  ads1015Error_t error = ADS1015_ERROR_OK;

  // Clear write buffers
  uint32_t i;
  for ( i = 0; i < I2C_BUFSIZE; i++ )
  {
    I2CMasterBuffer[i] = 0x00;
  }

  I2CWriteLength = 4;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = ADS1015_ADDRESS;       // I2C device address
  I2CMasterBuffer[1] = reg;                   // Register
  I2CMasterBuffer[2] = value >> 8;            // Upper 8-bits
  I2CMasterBuffer[3] = value & 0xFF;          // Lower 8-bits
  i2cEngine();

  // ToDo: Add in proper I2C error-checking
  return error;
}
*/

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/

/*
static ads1015Error_t ina219Read16(uint8_t reg, uint16_t *value)
{
  ads1015Error_t error = ADS1015_ERROR_OK;

  // Clear write buffers
  uint32_t i;
  for ( i = 0; i < I2C_BUFSIZE; i++ )
  {
    I2CMasterBuffer[i] = 0x00;
  }

  I2CWriteLength = 2;
  I2CReadLength = 2;
  I2CMasterBuffer[0] = ADS1015_ADDRESS;           // I2C device address
  I2CMasterBuffer[1] = reg;                       // Command register
  // Append address w/read bit
  I2CMasterBuffer[2] = ADS1015_ADDRESS | ADS1015_READBIT;
  i2cEngine();

  // Shift values to create properly formed integer
  *value = ((I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1]);

  // ToDo: Add in proper I2C error-checking
  return error;
}
*/


/**************************************************************************/
/*!
    @brief  Reads the 12-bit conversion results from the specified channel
*/
/**************************************************************************/








void ICACHE_FLASH_ATTR  ads1015ReadADC_SingleRead(void)
{

	  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
	                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
	                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
	                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
	                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
	                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

	  // Set PGA/voltage range
//	  config |= ADS1015_REG_CONFIG_PGA_4_096V;            // +/- 6.144V range (limited to VDD +0.3V max!)

	  config |= ADS1015_REG_GAIN_Array[ADS1015_REG_GAIN_selector];

	  // Set single-ended input channel
	  switch (ads1015_channel_to_read)
	  {
	    case (0):
	      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
	      break;
	    case (1):
	      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
	      break;
	    case (2):
	      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
	      break;
	    case (3):
	      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
	      break;
	  }

	  // Set 'start single-conversion' bit
	  config |= ADS1015_REG_CONFIG_OS_SINGLE;

	  uint8 ads_data[2];
	  ads_data[0] = HI(config);
	  ads_data[1] = LO(config);
//	  INFO ("\r\n ADS config %X\r\n", config);

	if (!i2c_Write_Addr_Buffer(ADS1015_ADDRESS, ADS1015_REG_POINTER_CONFIG, ads_data, 2, ads1015ReadADC_SingleRead_01)  )     // ������
	{
		os_timer_disarm (&ads1015_timer);
		os_timer_setfn (&ads1015_timer, (os_timer_func_t *)ads1015ReadADC_SingleRead, NULL);
		os_timer_arm(&ads1015_timer, 1500, 0);	// ��������� ������� ����� 2��
	}
}


void ICACHE_FLASH_ATTR ads1015ReadADC_SingleRead_01 (void)
{

	i2c_Do &= i2c_Free;			// ����������� ����

	if(i2c_Do & (i2c_ERR_NA|i2c_ERR_BF))	// ���� ������ �� �������
	{
		os_timer_disarm (&ads1015_timer);
		os_timer_setfn (&ads1015_timer, (os_timer_func_t *)ads1015ReadADC_SingleRead, NULL);
		os_timer_arm(&ads1015_timer, 2, 0);	// ��������� ������� ����� 2��
	}
	else
	{
		if (!i2c_Read_Addr_Buffer(ADS1015_ADDRESS, ADS1015_REG_POINTER_CONVERT, 2, ads1015ReadADC_SingleReaded)  )     // ������
		{
			os_timer_disarm (&ads1015_timer);
			os_timer_setfn (&ads1015_timer, (os_timer_func_t *)ads1015ReadADC_SingleRead, NULL);
			os_timer_arm(&ads1015_timer, 2, 0);	// ��������� ������� ����� 2��
		}



	}
}



void ICACHE_FLASH_ATTR ads1015ReadADC_SingleReaded (void)
{

	os_timer_disarm (&ads1015_timer);

	i2c_Do &= i2c_Free;			// ����������� ����

	if(i2c_Do & (i2c_ERR_NA|i2c_ERR_BF))	// ���� ������ �� �������
	{


		os_timer_setfn (&ads1015_timer, (os_timer_func_t *)ads1015ReadADC_SingleRead, NULL);
		os_timer_arm(&ads1015_timer, 2, 0);	// ��������� ������� ����� 2��
	}
	else
	{

		ads1015_data[ads1015_channel_to_read] = (i2c_Buffer[1] | (i2c_Buffer[0]<<8))>>4;        		// ����� �������� ���� �� ������ �������� � ����������

		INFO("\r\n ADS data %d \r\n",ads1015_data[ads1015_channel_to_read]);
		INFO("\r\n ADS Vin %d,%03d \r\n",(int)(ads1015_data[ads1015_channel_to_read]*ADS1015_REG_volt_converter_Array[ADS1015_REG_GAIN_selector])/1000,\
									   ((int)(ads1015_data[ads1015_channel_to_read]*ADS1015_REG_volt_converter_Array[ADS1015_REG_GAIN_selector]))%1000);
		MQTT_Client* client_publish = &mqttClient;
		char tc[80];
		os_sprintf(tc, "%d",(2048 - ads1015_data[ads1015_channel_to_read]));
		MQTT_Publish(client_publish, "/esp1/ADS01", tc, strlen(tc), 0, 0);

	}


}

/*
ads1015Error_t ads1015ReadADC_SingleEnded(uint8_t channel, uint16_t *value)
{
  ads1015Error_t error = ADS1015_ERROR_OK;

  if (!(_ads1015Initialised))
  {
    ads1015Init();
  }

  if (channel > 3)
  {
    *value = 0;
    return ADS1015_ERROR_INVALIDCHANNEL;
  }

  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= ADS1015_REG_CONFIG_PGA_6_144V;            // +/- 6.144V range (limited to VDD +0.3V max!)

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  error = ads1015WriteRegister(ADS1015_REG_POINTER_CONFIG, config);
  if (error) return error;

  // Wait for the conversion to complete
  systickDelay(1);

  // Read the conversion results
  error = ina219Read16(ADS1015_REG_POINTER_CONVERT, value);
  if (error) return error;

  // Shift results 4-bits to the right
  *value = *value >> 4;

  return error;
}
*/

/**************************************************************************/
/*!
    @brief  Reads the 12-bit conversion results, measuring the voltage
            difference between the P (AIN0) and N (AIN1) input.  Generates
            a signed 12-bit value since the difference can be either
            positive or negative.
*/
/**************************************************************************/

/*
ads1015Error_t ads1015ReadADC_Differential_0_1(int16_t *value)
{
  ads1015Error_t error = ADS1015_ERROR_OK;

  if (!(_ads1015Initialised))
  {
    ads1015Init();
  }

  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= ADS1015_REG_CONFIG_PGA_6_144V;            // +/- 6.144V range (limited to VDD +0.3V max!)

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_0_1;          // AIN0 = P, AIN1 = N

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  error = ads1015WriteRegister(ADS1015_REG_POINTER_CONFIG, config);
  if (error) return error;

  // Wait for the conversion to complete
  systickDelay(1);

  // Read the conversion results
  error = ina219Read16(ADS1015_REG_POINTER_CONVERT, value);
  if (error) return error;

  // Shift results 4-bits to the right
  *value = *value >> 4;

  return error;
}
*/

/**************************************************************************/
/*!
    @brief  Reads the 12-bit conversion results, measuring the voltage
            difference between the P (AIN2) and N (AIN3) input.  Generates
            a signed 12-bit value since the difference can be either
            positive or negative.
*/
/**************************************************************************/

/*
ads1015Error_t ads1015ReadADC_Differential_2_3(int16_t *value)
{
  ads1015Error_t error = ADS1015_ERROR_OK;

  if (!(_ads1015Initialised))
  {
    ads1015Init();
  }

  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= ADS1015_REG_CONFIG_PGA_6_144V;            // +/- 6.144V range (limited to VDD +0.3V max!)

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_2_3;          // AIN2 = P, AIN3 = N

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  error = ads1015WriteRegister(ADS1015_REG_POINTER_CONFIG, config);
  if (error) return error;

  // Wait for the conversion to complete
  systickDelay(1);

  // Read the conversion results
  error = ina219Read16(ADS1015_REG_POINTER_CONVERT, value);
  if (error) return error;

  // Shift results 4-bits to the right
  *value = *value >> 4;

  return error;
}
*/

/**************************************************************************/
/*!
    @brief  Sets up the comparator to operate in basic mode, causing the
            ALERT/RDY pin to assert (go from high to low) when the ADC
            value exceeds the specified threshold.

            This will also set the ADC in continuous conversion mode.

    @section EXAMPLE

    @code
    ads1015Init();

    // Setup 3V comparator on channel 0
    ads1015StartComparator_SingleEnded(0, 1000);

    int16_t results;
    while(1)
    {
      // Need to read to clear com bit once it's set
      ads1015GetLastConversionResults(&results);
      printf("%d\r\n", results);
    }
    @endcode
*/
/**************************************************************************/

/*
ads1015Error_t ads1015StartComparator_SingleEnded(uint8_t channel, int16_t threshold)
{
  uint16_t value;

  ads1015Error_t error = ADS1015_ERROR_OK;

  if (!(_ads1015Initialised))
  {
    ads1015Init();
  }

  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_1CONV   | // Comparator enabled and asserts on 1 match
                    ADS1015_REG_CONFIG_CLAT_LATCH   | // Latching mode
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_CONTIN  | // Continuous conversion mode
                    ADS1015_REG_CONFIG_PGA_6_144V   | // +/- 6.144V range (limited to VDD +0.3V max!)
                    ADS1015_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set the high threshold register
  error = ads1015WriteRegister(ADS1015_REG_POINTER_HITHRESH, threshold << 4);
  if (error) return error;

  // Write config register to the ADC
  error = ads1015WriteRegister(ADS1015_REG_POINTER_CONFIG, config);
  if (error) return error;
}
*/

/**************************************************************************/
/*!
    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value.
*/
/**************************************************************************/

/*
ads1015Error_t ads1015GetLastConversionResults(int16_t *value)
{
  ads1015Error_t error = ADS1015_ERROR_OK;

  if (!(_ads1015Initialised))
  {
    ads1015Init();
  }

  // Wait for the conversion to complete
  systickDelay(1);

  // Read the conversion results
  error = ina219Read16(ADS1015_REG_POINTER_CONVERT, value);
  if (error) return error;

  // Shift results 4-bits to the right
  *value = *value >> 4;

  return error;
}
*/





