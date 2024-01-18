/*

 Company: 	TEXA S.p.a.
 Author: 	Alberto Ferraro
 Date: 		19/10/16
 Version: 	1.0

 File Description:
 This file implements the I2C based driver for an image sensor that uses I2C for control and other functions
 to initialize, power-up and configure the sensors.

*/

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include <cyu3i2c.h>
#include <cyu3types.h>
#include <cyu3gpio.h>
#include <cyu3utils.h>
#include "uvc.h"
#include "sensor.h"
#include "OV5640.h"



/* Send a reset pulse to the rear camera. */
void SensorReset_R(void)
{
	/* Drive the GPIO low to reset the sensor. */
	CyU3PGpioSetValue(RESET_REAR_CAM_GPIO, CyFalse);

	/* Wait for some time to allow proper reset. */
	CyU3PThreadSleep(10);

	/* Drive the GPIO high to bring the sensor out of reset. */
	CyU3PGpioSetValue(RESET_REAR_CAM_GPIO, CyTrue);

	/* Wait for some time to allow proper reset. */
	CyU3PThreadSleep(50);
}


/* Send a reset pulse to the front camera. */
void SensorReset_F(void)
{
	/* Drive the GPIO low to reset the sensor. */
	CyU3PGpioSetValue(RESET_FRONT_CAM_GPIO, CyFalse);

	/* Wait for some time to allow proper reset. */
	CyU3PThreadSleep(10);

	/* Drive the GPIO high to bring the sensor out of reset. */
	CyU3PGpioSetValue(RESET_FRONT_CAM_GPIO, CyTrue);

	/* Wait for some time to allow proper reset. */
	CyU3PThreadSleep(50);
}


/* Power-down or power-up the rear camera. To power-down: status = CyTrue. */
void SensorPwdn_R(CyBool_t status)
{
	/* Drive the GPIO high to power-down the sensor. */
	CyU3PGpioSetValue(PWDN_REAR_CAM_GPIO, status);
	CyU3PThreadSleep (40);								/* Wait 20 ms: MIN 20 ms */
}


/* Power-down or power-up the front camera. To power-down: status = CyTrue. */
void SensorPwdn_F(CyBool_t status)
{
	/* Drive the GPIO high to power-down the sensor. */
	CyU3PGpioSetValue(PWDN_FRONT_CAM_GPIO, status);
	CyU3PThreadSleep (40);								/* Wait 20 ms: MIN 20 ms */
}


/* This function inserts a delay between successful I2C transfers to prevent
 false errors due to the slave being busy.*/
void SensorI2CAccessDelay(CyU3PReturnStatus_t status)
{
	/* Add a 10us delay if the I2C operation that preceded this call was successful. */
	if (status == CY_U3P_SUCCESS)
		CyU3PBusyWait(10);
}


/* Read from an I2C slave count bytes of data (count < 64). */
CyU3PReturnStatus_t SensorRead(uint8_t slaveAddr, uint16_t Addr,
		uint8_t count, uint8_t *buf)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;

	if (count > 64)
	{
		myCyU3PDebugPrint(4, "ERROR: SensorWrite count > 64 \n");
		apiRetStatus = 1;
	}
	else
	{
		/* Validate the slave address and read data. */
		if (slaveAddr == SENSOR_ADDR_RD)
		{
			preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK; 	/*  Mask out the transfer type bit. */
			preamble.buffer[1] = (uint8_t)(Addr>>8);
			preamble.buffer[2] = (uint8_t)(Addr);
			preamble.buffer[3] = slaveAddr;
			preamble.length = 4;
			preamble.ctrlMask = 0x0004; 	/* Send start bit after third byte of preamble. */

			apiRetStatus = CyU3PI2cReceiveBytes(&preamble, buf, count, 0);
			SensorI2CAccessDelay(apiRetStatus);
		}
		else
		{
			myCyU3PDebugPrint(4, "I2C Slave address is not a sensor valid address! \n");
			apiRetStatus = 1;
		}
	}
	return apiRetStatus;
}


/* Write to an I2C slave with 16 bit registers a count (< 64) amount of data bytes. */
CyU3PReturnStatus_t SensorWrite(uint8_t slaveAddr, uint16_t Addr,
		uint8_t count, uint8_t *buf)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t preamble;

	if (count > 64)
	{
		myCyU3PDebugPrint(4, "ERROR: SensorWrite count > 64 \n");
		apiRetStatus = 1;
	}
	else
	{
		/* Validate the I2C slave address and write the information. */
		if (slaveAddr == SENSOR_ADDR_WR)
		{
			/* Set up the I2C control parameters and invoke the write API. */
			preamble.buffer[0] = slaveAddr;
			preamble.buffer[1] = (uint8_t)(Addr>>8);
			preamble.buffer[2] = (uint8_t)(Addr);
			preamble.length = 3;
			preamble.ctrlMask = 0x0000;

			apiRetStatus = CyU3PI2cTransmitBytes(&preamble, buf, count, 0);
			SensorI2CAccessDelay(apiRetStatus);
		}
		else
		{
			myCyU3PDebugPrint(4, "I2C Slave address is not a sensor valid address! \n");
			apiRetStatus = 1;
		}
	}
	return apiRetStatus;
}


/* Read a list of register settings and compare the content with that of the structure pointed by regs (pay attention,
 * some camera internal registers can be overwritten by the camera!). */
CyU3PReturnStatus_t SensorReadArray(uint8_t slaveAddr, regval_list_t *regs, uint16_t size)
{
	uint16_t i;
	uint8_t value = 0x00;
	CyBool_t ErrorCompareFlag = 0;

	if (size/sizeof(regval_list_t) == 0)
	{
		myCyU3PDebugPrint(4, "No registers to read. \n");
		return 1;
	}
	else
	{
		if (slaveAddr == SENSOR_ADDR_RD)
		{
			for(i = 0; i < (size/sizeof(regval_list_t)); i++)
			{
				if(regs->address != 0xffff)
				{
					if (SensorRead(slaveAddr, regs->address, 0x01, &value) == CY_U3P_SUCCESS)
					{
						if (value != regs->value)
						{
							myCyU3PDebugPrint(4, "Wrong sensor configuration. \n");
							ErrorCompareFlag = 1;
						}
					}
					else
					{
						myCyU3PDebugPrint(4, "Error reading sensor registers! \n");
						return 1;
					}
				}
				regs++;		// Sum a number of byte equal to the dimension of a regval_list structure.
			}
		}
		else
		{
			myCyU3PDebugPrint(4, "Wrong sensor address. \n");
			return 1;
		}
	}
	return ErrorCompareFlag;
}


/* Write a list of register settings */
CyU3PReturnStatus_t SensorWriteArray(uint8_t slaveAddr, regval_list_t *regs, uint16_t size)
{
	uint16_t i;
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	if (size/sizeof(regval_list_t) == 0)
	{
		myCyU3PDebugPrint(4, "No registers to configure. \n");
		return 1;
	}
	else
	{
		if (slaveAddr == SENSOR_ADDR_WR)
		{
			for(i = 0; i < (size/sizeof(regval_list_t)); i++)
			{
				if(regs->address == 0xffff)
				{
					CyU3PThreadSleep(regs->value);		// Wait for regs->value ms
				}
				else
				{
					if (SensorWrite(slaveAddr, regs->address, 0x01, &(regs->value)) != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint(4, "Error in writing sensor registers! \n");
						return 1;
					}
				}
				regs++;		// Sum a number of byte equal to the dimension of a regval_list structure
			}
		}
		else
		{
			myCyU3PDebugPrint(4, "Wrong sensor address. \n");
			return 1;
		}
	}
	return apiRetStatus;
}


/* Verify that the sensor can be accessed over the I2C bus from FX3.
 * The sensor ID register can be read here to verify sensor connectivity. */
CyU3PReturnStatus_t SensorI2cBusTest(uint8_t slaveAddr_RD)
{
	CyU3PReturnStatus_t apiRetStatus = 1;
	uint8_t buf[2];

	if (slaveAddr_RD == SENSOR_ADDR_RD)
	{
		/* Reading sensor ID */
		if (SensorRead(SENSOR_ADDR_RD, 0x300a, 0x01, &buf[0]) == CY_U3P_SUCCESS) {
			if (SensorRead(SENSOR_ADDR_RD, 0x300b, 0x01, &buf[1]) == CY_U3P_SUCCESS) {
				if ((buf[0] == 0x56) && (buf[1] == 0x40)) {
					apiRetStatus = CY_U3P_SUCCESS;
				}
			}
		}
	}
	else
	{
		myCyU3PDebugPrint(4, "Error: Wrong Sensor address!\r\n");
	}

	return apiRetStatus;
}


/* Image sensor initialization: Reset and then initialize with appropriate configuration. */
CyU3PReturnStatus_t SensorInit(CyBool_t EN_FRONT_CAM, CyBool_t EN_REAR_CAM)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	if ((EN_FRONT_CAM && (!EN_REAR_CAM)) || EN_REAR_CAM)
	{
		if (EN_FRONT_CAM && (!EN_REAR_CAM))
		{
			SensorPwdn_R(CyTrue);		// Hold the rear camera in power-down
			SensorReset_F();			// Hard reset of the front image sensors and wait 20 ms
		}
		else
		{
			SensorPwdn_F(CyTrue);		// Hold the front camera in power-down
			SensorReset_R();			// Hard reset of the rear image sensors and wait 20 ms
		}

		/* Verify that the sensor is ready to communicate */
		apiRetStatus = SensorI2cBusTest(SENSOR_ADDR_RD);
		if (apiRetStatus != CY_U3P_SUCCESS)
		{
			myCyU3PDebugPrint(4, "Error: Reading Sensor ID failed!\r\n");
			return apiRetStatus;
		}

		/* Load default front camera settings */
		apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, sensor_default_regs ,sizeof(sensor_default_regs));
		if (apiRetStatus != CY_U3P_SUCCESS)
		{
			myCyU3PDebugPrint(4, "Error: Writing Sensor Registers failed!\r\n");
			return apiRetStatus;
		}

		/* Bypass the internal regulator of the camera */
		apiRetStatus = Bypass_Reg();
		if (apiRetStatus != CY_U3P_SUCCESS)
		{
			myCyU3PDebugPrint(4, "Error: Bypass internal regulator failed!\r\n");
			return apiRetStatus;
		}

		/* Update sensor configuration based on default desired video stream parameters (640x480 15fps)*/
		apiRetStatus = SensorScaling_VGA_30fps();
		if (apiRetStatus != CY_U3P_SUCCESS)
		{
			myCyU3PDebugPrint(4, "Error: Writing VGA configuration failed!\r\n");
		}
	}
    else
    {
    	myCyU3PDebugPrint (4, "Power-down mode!\n");
    	return 1;
    }

    return apiRetStatus;
}


/* Set the camera in VGA (640x480) 30 fps streaming mode. */
CyU3PReturnStatus_t SensorScaling_VGA_30fps(void)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	/* Load default front camera settings */
	apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, sensor_default_regs ,sizeof(sensor_default_regs));
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint(4, "Error: Writing Sensor Registers failed!\r\n");
		return apiRetStatus;
	}

	apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, sensor_VGA_regs, sizeof(sensor_VGA_regs));
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint(4, "Error: Writing Sensor Registers failed!\r\n");
		return apiRetStatus;
	}

	return apiRetStatus;
}


/* Set the selected camera in 720p (1280x720) 30 fps streaming mode. */
CyU3PReturnStatus_t SensorScaling_720p_30fps(void)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	/* Load default front camera settings */
	apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, sensor_default_regs ,sizeof(sensor_default_regs));
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint(4, "Error: Writing Sensor Registers failed!\r\n");
		return apiRetStatus;
	}

	apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, sensor_720p_regs, sizeof(sensor_720p_regs));
	return apiRetStatus;
}


/* Set the selected camera in 1080p (1920x1080) 30 fps streaming mode. */
CyU3PReturnStatus_t SensorScaling_1080p_30fps(void)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	/* Load default front camera settings */
	apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, sensor_default_regs ,sizeof(sensor_default_regs));
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint(4, "Error: Writing Sensor Registers failed!\r\n");
		return apiRetStatus;
	}

	apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, sensor_1080p_regs, sizeof(sensor_1080p_regs));
	return apiRetStatus;
}


#ifdef STILL_IMAGE_CAPTURE_M2

/* Set the selected camera in 5M (2592x1944) 7.5 fps streaming mode. */
CyU3PReturnStatus_t SensorScaling_5M(void)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	/* Load default front camera settings */
	apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, sensor_default_regs ,sizeof(sensor_default_regs));
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint(4, "Error: Writing Sensor Registers failed!\r\n");
		return apiRetStatus;
	}

	apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, sensor_5M_regs, sizeof(sensor_5M_regs));
	return apiRetStatus;
}

#endif


#ifdef BRIGHTNESS_CONTROL

/* Get the current brightness level from the camera. */
uint8_t SensorGetBrightness(void)
{
	uint8_t buf[2];
	uint8_t level = 4;

	SensorRead(SENSOR_ADDR_RD, BRIGHT, 0x01, &buf[0]); 			// Read Y bright.
	SensorRead(SENSOR_ADDR_RD, BRIGHT_SIGN, 0x01, &buf[1]); 	// Read Y bright sign.

	if ((buf[1] & SIGN_MASK) == 0x08)
	{
		/* Negative sign */
		switch (buf[0])
		{
			case 0x40:
				level = 0;	/* Brightness level -4 */
				break;
			case 0x30:
				level = 1;	/* Brightness level -3 */
				break;
			case 0x20:
				level = 2;	/* Brightness level -2 */
				break;
			case 0x10:
				level = 3;	/* Brightness level -1 */
				break;
			default:
				break;
		}
	}
	else
	{
		/* Positive sign */
		switch (buf[0])
		{
			case 0x40:
				level = 8;	/* Brightness level +4 */
				break;
			case 0x30:
				level = 7;	/* Brightness level +3 */
				break;
			case 0x20:
				level = 6;	/* Brightness level +2 */
				break;
			case 0x10:
				level = 5;	/* Brightness level +1 */
				break;
			case 0x00:
				level = 4;	/* Default brightness level 0 */
				break;
			default:
				break;
		}
	}
	return level;
}


/* Update the brightness of the image sensor. */
CyU3PReturnStatus_t SensorSetBrightness(uint8_t brightness)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	switch (brightness)
	{
		case 0:
			apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, cam_brightness_lvn4, sizeof(cam_brightness_lvn4));
			break;
		case 1:
			apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, cam_brightness_lvn3, sizeof(cam_brightness_lvn3));
			break;
		case 2:
			apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, cam_brightness_lvn2, sizeof(cam_brightness_lvn2));
			break;
		case 3:
			apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, cam_brightness_lvn1, sizeof(cam_brightness_lvn1));
			break;
		case 4:
			apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, cam_brightness_lv0, sizeof(cam_brightness_lv0));
			break;
		case 5:
			apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, cam_brightness_lvp1, sizeof(cam_brightness_lvp1));
			break;
		case 6:
			apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, cam_brightness_lvp2, sizeof(cam_brightness_lvp2));
			break;
		case 7:
			apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, cam_brightness_lvp3, sizeof(cam_brightness_lvp3));
			break;
		case 8:
			apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, cam_brightness_lvp4, sizeof(cam_brightness_lvp4));
			break;
		default:
			myCyU3PDebugPrint(4, "Error: Wrong brightness level selected.\r\n");
			apiRetStatus = 1;
			break;
	}
	return apiRetStatus;
}

#endif

#ifdef AUTOFOCUS

/* Download the Camera Auto Focus Firmware inside the MCU via I2C. */
CyU3PReturnStatus_t AF_init(void)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint8_t value = 0x00;

	apiRetStatus = SensorWriteArray(SENSOR_ADDR_WR, AF_firmware, sizeof(AF_firmware));
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	value = 0x08;		// Release focus command.
	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, CMD_MAIN, 0x01, &value);
	return apiRetStatus;
}


/* Set up Auto Focus. */
CyU3PReturnStatus_t AF_single_focus(void)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint8_t value = 0x01;

	value = 0x01;		// Command is running
	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, CMD_ACK, 0x01, &value);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	value = 0x03;		// Start single focus
	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, CMD_MAIN, 0x01, &value);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	while(value != 0x00)
	{
		// Auto Focus command is running.
		apiRetStatus = SensorRead(SENSOR_ADDR_RD, CMD_ACK, 0x01, &value);
		if (apiRetStatus != CY_U3P_SUCCESS)
		{
			return apiRetStatus;
		}
		CyU3PThreadSleep (20);	/* Wait 20 ms */
	}
	return apiRetStatus;
}


/* Release Focus. */
CyU3PReturnStatus_t AF_release_focus(void)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint8_t value = 0x01;

	value = 0x01;		// Command is running
	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, CMD_ACK, 0x01, &value);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	value = 0x08;		// Release focus
	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, CMD_MAIN, 0x01, &value);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	while(value != 0x00)
	{
		// Release Focus command is running.
		apiRetStatus = SensorRead(SENSOR_ADDR_RD, CMD_ACK, 0x01, &value);
		if (apiRetStatus != CY_U3P_SUCCESS)
		{
			return apiRetStatus;
		}
		CyU3PThreadSleep (20);	/* Wait 20 ms */
	}
	return apiRetStatus;
}
#endif


/* Auto/Manual AEC/AGC control */
CyU3PReturnStatus_t AEC_AGC_Auto(CyBool_t Auto_Enable)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint8_t value;

	if (Auto_Enable)
	{
		value = 0x00;
		apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x3503, 0x01, &value);
	}
	else
	{
		value = 0x07;
		apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x3503, 0x01, &value);
	}
	return apiRetStatus;
}


#ifdef STILL_IMAGE_CAPTURE_M2

/* Read back the preview's Exposure, Gain, Maxlines */
CyU3PReturnStatus_t ReadPreviewRegs(uint16_t *preReg)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint8_t Temp1, Temp2, Temp3;

	/* Read the Exposure */
	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3500, 0x01, &Temp1);		/* AEC PK EXPOSURE */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}
	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3501, 0x01, &Temp2);		/* AEC PK EXPOSURE */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}
	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3502, 0x01, &Temp3);		/* AEC PK EXPOSURE */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	*preReg = (((uint16_t)(Temp1) & 0x000f) << 12) + ((uint16_t)(Temp2) << 4) + (((uint16_t)(Temp3) & 0x00f0) >> 4);

	/* Read the Gain */
	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x350a, 0x01, &Temp1);		/* AEC PK REAL GAIN */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}
	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x350b, 0x01, &Temp2);		/* AEC PK REAL GAIN */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	*(preReg+1) = ((uint16_t)(Temp1) << 8) + ((uint16_t)(Temp2));

	/* Read the AEC VTS */
	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x350c, 0x01, &Temp1);		/* AEC PK VTS (maxlines) */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}
	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x350d, 0x01, &Temp2);		/* AEC PK VTS (maxlines) */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	*(preReg+2) = ((uint16_t)(Temp1) << 8) + ((uint16_t)(Temp2));

	/* Read the average value */
	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x56a1, 0x01, &Temp1);		/* AVG READOUT */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	*(preReg+3) = (uint16_t)(Temp1);

	/* Read Preview Sysclk */
	apiRetStatus = Get_Sysclk(preReg+4);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	/* Read Preview HTS */
	apiRetStatus = Get_HTS(preReg+5);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	/* Read Preview VTS */
	apiRetStatus = Get_VTS(preReg+6);
	return apiRetStatus;
}


/* Read back the capture's parameters, calculate the new ones and re-configure the camera */
CyU3PReturnStatus_t CaptureSettings(uint16_t *preReg)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint8_t Light_Freq, Temp, Average;
	uint16_t Capture_VTS, Capture_HTS, Preview_HTS, Preview_Sysclk, Capture_Sysclk, Capture_Max_Band, Capture_Gain, Capture_Bandingfilter, Preview_Gain, Preview_Shutter;
	uint32_t Capture_Shutter, Capture_Gain_Shutter;

	/* Explicit Preview Parameters */
	Preview_Gain = *(preReg+1);
	Preview_Shutter = *preReg;
	Preview_HTS = *(preReg+5);
	Preview_Sysclk = *(preReg+4);
	Average = (uint8_t)(*(preReg+3));

	// Read capture VTS, HTS and Sysclk
	apiRetStatus = Get_VTS(&Capture_VTS);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	apiRetStatus = Get_HTS(&Capture_HTS);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	apiRetStatus = Get_Sysclk(&Capture_Sysclk);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	// Calculate capture banding filter
	apiRetStatus = Get_Light_Frequency(&Light_Freq);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	if (Light_Freq == 60)
	{
		Capture_Bandingfilter = Capture_Sysclk * 100 / Capture_HTS * 100 / 120;		// 60Hz
	}
	else
	{
		Capture_Bandingfilter = Capture_Sysclk * 100 / Capture_HTS;					// 50Hz
	}

	Capture_Max_Band = (uint16_t)((Capture_VTS - 4)/Capture_Bandingfilter);

	/* Gain times Shutter */
	if (Average > AE_Low && Average < AE_High)
	{
		// In stable range
		Capture_Gain_Shutter = Preview_Gain * Preview_Shutter * Capture_Sysclk/Preview_Sysclk * Preview_HTS/Capture_HTS * AE_Target / Average;
	}
	else
	{
		Capture_Gain_Shutter = Preview_Gain * Preview_Shutter * Capture_Sysclk/Preview_Sysclk * Preview_HTS/Capture_HTS;
	}

	/* Calculate Capture Shutter/Gain */
	if(Capture_Gain_Shutter < (Capture_Bandingfilter * 16))
	{
		/* Shutter < 1/100 */
		Capture_Shutter = Capture_Gain_Shutter/16;
		if(Capture_Shutter < 1)
		{
			Capture_Shutter = 1;
		}

		Capture_Gain = Capture_Gain_Shutter/Capture_Shutter;
		if(Capture_Gain < 16)
		{
			Capture_Gain = 16;
		}
	}
	else
	{
		if(Capture_Gain_Shutter > (Capture_Bandingfilter * Capture_Max_Band * 16))
		{
			/* Exposure reach max */
			Capture_Shutter = Capture_Bandingfilter * Capture_Max_Band;
			Capture_Gain = Capture_Gain_Shutter / Capture_Shutter;
		}
		else
		{
			/* 1/100 < Capture_Shutter =< Exposure max ---> Capture_Shutter = n/100 */
			Capture_Shutter = (uint16_t)(Capture_Gain_Shutter/16/Capture_Bandingfilter)*Capture_Bandingfilter;
			Capture_Gain = Capture_Gain_Shutter / Capture_Shutter;
		}
	}

	/* Write Capture Gain */
	Capture_Gain = Capture_Gain & 0x03ff;
	Temp = (uint8_t)(Capture_Gain);

	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x350b, 0x01, &Temp);		/* Write Low Gain Byte */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	Temp = (uint8_t)(Capture_Gain >> 8);

	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x350a, 0x01, &Temp);		/* Write High Gain Byte */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	/* Write Capture VTS Timing */
	if (Capture_Shutter > (Capture_VTS - 4))
	{
		Capture_VTS = Capture_Shutter + 4;
		Temp = (uint8_t)(Capture_VTS & 0x00ff);

		apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x380f, 0x01, &Temp);		/* Write TIMING VTS Low Byte */
		if (apiRetStatus != CY_U3P_SUCCESS)
		{
			return apiRetStatus;
		}

		Temp = (uint8_t)(Capture_VTS >> 8);
		apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x380e, 0x01, &Temp);		/* Write TIMING VTS High Byte */
		if (apiRetStatus != CY_U3P_SUCCESS)
		{
			return apiRetStatus;
		}
	}

	/* Write Capture Shutter */
	Temp = (uint8_t)(Capture_Shutter & 0x000f);
	Temp = Temp << 4;

	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x3502, 0x01, &Temp);			/* Write Exposure Low Byte */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	Temp = (uint8_t)(Capture_Shutter & 0x0fff);
	Temp = Temp >> 4;

	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x3501, 0x01, &Temp);			/* Write Exposure Mid Byte */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	Temp = (uint8_t)(Capture_Shutter >> 12);

	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x3500, 0x01, &Temp);			/* Write Exposure High Byte */

	/* End Capture Settings */

	return apiRetStatus;
}


/* Get the sysclk of the camera from register settings */
CyU3PReturnStatus_t Get_Sysclk(uint16_t *sysclk)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	uint8_t temp1, Bit_div2x, SysDiv, Multiplier, PreDiv, Pll_rdiv, sclk_rdiv, sclk_rdiv_map[] = {1, 2, 4, 8};
	uint32_t VCO;

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3034, 0x01, &temp1);		/* Read SC PLL CONTRL0 register */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	temp1 = temp1 & 0x0f;
	if (temp1 == 8 || temp1 == 10)
	{
		Bit_div2x = temp1 / 2;
	}
	else
	{
		Bit_div2x = 1;
	}

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3035, 0x01, &temp1);		/* Read SC PLL CONTRL1 register */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	SysDiv = temp1 >> 4;
	if(SysDiv == 0)
	{
		SysDiv = 16;
	}

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3036, 0x01, &temp1);		/* Read SC PLL CONTRL2 register */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	Multiplier = temp1;

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3037, 0x01, &temp1);		/* Read SC PLL CONTRL3 register */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	PreDiv = temp1 & 0x0f;
	Pll_rdiv = ((temp1 >> 4) & 0x01) + 1;

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3108, 0x01, &temp1);		/* Read SYSTEM ROOT DIVIDER register */

	temp1 = temp1 & 0x03;
	sclk_rdiv = sclk_rdiv_map[temp1];
	VCO = XVCLK * Multiplier / PreDiv;
	*sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;

	return apiRetStatus;
}


/* Read HTS from register settings */
CyU3PReturnStatus_t Get_HTS(uint16_t *HTS)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint8_t HTS_H, HTS_L;

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x380c, 0x01, &HTS_H);		/* HTS high byte */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x380d, 0x01, &HTS_L);		/* HTS low byte */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	*HTS = ((uint16_t)(HTS_H)<<8) + (uint16_t)(HTS_L);

	return apiRetStatus;
}


/* Read VTS from register settings */
CyU3PReturnStatus_t Get_VTS(uint16_t *VTS)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint8_t VTS_H, VTS_L;

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x380e, 0x01, &VTS_H);		/* VTS high byte */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x380f, 0x01, &VTS_L);		/* VTS low byte */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	*VTS = ((uint16_t)(VTS_H)<<8) + (uint16_t)(VTS_L);

	return apiRetStatus;
}


/* Get banding filter value */
CyU3PReturnStatus_t Get_Light_Frequency(uint8_t *light_freq)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint8_t temp, temp1;

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3c01, 0x01, &temp);			/* Read 5060HZ CTRL01 register */
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	if (temp & 0x80)
	{
		// Manual mode
		apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3c00, 0x01, &temp1);			/* Read 5060HZ CTRL00 register */
		if (apiRetStatus != CY_U3P_SUCCESS)
		{
			return apiRetStatus;
		}

		if (temp1 & 0x04)
		{
			*light_freq = 50;		// 50Hz
		}
		else
		{
			*light_freq = 60;		// 60Hz
		}
	}
	else
	{
		// Auto mode
		apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3c0c, 0x01, &temp1);			/* Read SIGMADELTA CTRL0C register */
		if (apiRetStatus != CY_U3P_SUCCESS)
		{
			return apiRetStatus;
		}

		if (temp1 & 0x01)
		{
			*light_freq = 50;		// 50Hz
		}
		else
		{
			*light_freq = 60;		// 60Hz
		}
	}

	return apiRetStatus;
}

#endif


/* Turn on/off the night mode */
CyU3PReturnStatus_t Set_Night_Mode(CyBool_t NightMode)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint8_t temp;

	switch (NightMode)
	{
		case CyFalse:	// Off
			apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3a00, 0x01, &temp);
			if (apiRetStatus != CY_U3P_SUCCESS)
			{
				return apiRetStatus;
			}

			temp = temp & 0xfb; 			// night mode off, bit[2] = 0

			apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x3a00, 0x01, &temp);
			break;
		case CyTrue:	// On
			apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3a00, 0x01, &temp);
			if (apiRetStatus != CY_U3P_SUCCESS)
			{
				return apiRetStatus;
			}

			temp = temp | 0x04; 			// night mode on, bit[2] = 1

			apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x3a00, 0x01, &temp);
			break;
		default:
			break;
	}
	return apiRetStatus;
}


/* This function write the 0x3031 register of the camera to bypass the internal regulator */
CyU3PReturnStatus_t Bypass_Reg(void)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint8_t temp;

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3031, 0x01, &temp);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	temp = temp | (0x08);		/* Bit 3 set to 1: Bypass internal regulator */

	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x3031, 0x01, &temp);

	return apiRetStatus;
}


/* This function perform the mirror action on the image, the flip action or both */
CyU3PReturnStatus_t Mirror_Flip(CyBool_t Mir, CyBool_t Flip)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint8_t temp, temp1;

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3820, 0x01, &temp);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	apiRetStatus = SensorRead(SENSOR_ADDR_RD, 0x3821, 0x01, &temp1);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	if (Mir && (!Flip))				// Only Mirror
	{
		temp = temp & 0xf9;
		temp = temp | 0x00;
		temp1 = temp1 & 0xf9;
		temp1 = temp1 | 0x06;
	}
	else if ((!Mir) && (Flip))		// Only Flip
	{
		temp = temp & 0xf9;
		temp = temp | 0x06;
		temp1 = temp1 & 0xf9;
		temp1 = temp1 | 0x00;
	}
	else if (Mir && Flip)			// Mirror and Flip
	{
		temp = temp & 0xf9;
		temp = temp | 0x06;
		temp1 = temp1 & 0xf9;
		temp1 = temp1 | 0x06;
	}
	else							// Normal Mode
	{
		temp = temp & 0xf9;
		temp = temp | 0x00;
		temp1 = temp1 & 0xf9;
		temp1 = temp1 | 0x00;
	}

	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x3820, 0x01, &temp);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	apiRetStatus = SensorWrite(SENSOR_ADDR_WR, 0x3821, 0x01, &temp1);

	return apiRetStatus;
}
