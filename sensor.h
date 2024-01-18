/*

 Company: 	TEXA S.p.a.
 Author: 	Alberto Ferraro
 Date: 		19/10/16
 Version: 	1.0

 File Description:
 This file defines the parameters and the interface for the image sensor driver.

*/


#ifndef _INCLUDED_SENSOR_H_
#define _INCLUDED_SENSOR_H_

#define XVCLK 2400								/* Real cameras clock / 10000 */
#define I2C_SLAVEADDR_MASK (uint8_t)(0xFE)      /* Mask to get actual I2C slave address value without direction bit. */

/* GPIO output and input pins definitions for cameras control. */
#define PWDN_REAR_CAM_GPIO		(uint8_t)(24)
#define PWDN_FRONT_CAM_GPIO		(uint8_t)(21)
#define RESET_REAR_CAM_GPIO		(uint8_t)(25)
#define RESET_FRONT_CAM_GPIO	(uint8_t)(22)

/* I2C Slave address for the image sensor. */
#define SENSOR_ADDR_WR (uint8_t)(0x78)     		/* Slave address of OV5640 Camera, used to write sensor registers. */
#define SENSOR_ADDR_RD (uint8_t)(0x79)     		/* Slave address of OV5640 Camera, used to read from sensor registers. */

/* Define and variables for auto focus */
#define CMD_MAIN	(uint16_t)(0x3022) 			// Auto focus main command register
#define CMD_ACK		(uint16_t)(0x3023) 			// ACK of command
#define FW_STATUS	(uint16_t)(0x3029)	 		// Status of focus

#ifdef BRIGHTNESS_CONTROL

/* Define and variables for brightness */
#define BRIGHT  		(uint16_t)(0x5587)		// Y bright for contrast of camera sensor
#define BRIGHT_SIGN		(uint16_t)(0x5588)		// Y bright sign for contrast of camera sensor
#define SIGN_MASK 		(uint8_t)(0x08)			// Mask to get brightness sign.

#endif

/* Define and variables for Exposure */
#define AE_Target	(uint8_t)(52)
#define AE_Low		(uint8_t)(AE_Target * 23 /25)
#define AE_High		(uint8_t)(AE_Target * 27 /25)

/* The default register settings */
typedef struct regval_list {
	uint16_t address;
	uint8_t value;
} regval_list_t;


/* Function    : myCyU3PDebugPrint
   Description : Print a message.
   Parameters  : priority, the priority level for this message and message, a pointer to a string of characters.
 */
extern CyU3PReturnStatus_t
myCyU3PDebugPrint (
		uint8_t priority,
		char *message);

/* Function    : SensorReset_R
   Description : Send a reset pulse to the Rear Camera.
   Parameters  : None
 */
extern void
SensorReset_R (
        void);

/* Function    : SensorReset_F
   Description : Send a reset pulse to the Front Camera.
   Parameters  : None
 */
extern void
SensorReset_F (
        void);

/* Function    : SensorPwdn_R
   Description : Power-down the Rear Camera.
   Parameters  : Status of the power-down pin.
 */
extern void
SensorPwdn_R (
        CyBool_t status);

/* Function    : SensorPwdn_F
   Description : Power-down the Front Camera.
   Parameters  : Status of the power-down pin.
 */
extern void
SensorPwdn_F (
        CyBool_t status);

/* Function    : SensorI2CAccessDelay
   Description : Inserts a delay between successful I2C transfers to prevent false errors due to the slave being busy.
   Parameters  : The result of the last I2C operation.
 */
extern void
SensorI2CAccessDelay (
		CyU3PReturnStatus_t status);

/* Function    : SensorRead
   Description : Read data from image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address of the sensor.
                 Addr  	   - Register address being read from.
                 count     - Size of data to be read in bytes. Limited to a max of 64.
                 buf       - Buffer to be filled with data.
 */
extern CyU3PReturnStatus_t
SensorRead (
        uint8_t slaveAddr,
        uint16_t Addr,
        uint8_t count,
        uint8_t *buf);

/* Function    : SensorWrite
   Description : Write a count (<64) data bytes to an I2C slave having 16 bit registers.
   Parameters  :
                 slaveAddr - I2C slave address of the sensor.
                 Addr      - Register Address of the slave device.
                 count     - Number of bytes that must be written.
                 buf	   - Pointer to the buffer of data that must be written.
 */
extern CyU3PReturnStatus_t
SensorWrite (
        uint8_t slaveAddr,
        uint16_t Addr,
        uint8_t count,
        uint8_t *buf);

/* Function    : SensorReadArray
   Description : Read an array of image sensor registers over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address of the sensor.
                 regs      - Pointer to a structure of register addresses and values.
                 size      - Size of the structure pointed by regs.
 */
extern CyU3PReturnStatus_t
SensorReadArray (
        uint8_t slaveAddr,
        regval_list_t *regs,
        uint16_t size);

/* Function    : SensorWriteArray
   Description : Write an array of image sensor registers over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address of the sensor.
                 regs      - Pointer to a structure of register addresses and values.
                 size      - Size of the structure pointed by regs.
 */
extern CyU3PReturnStatus_t
SensorWriteArray (
        uint8_t slaveAddr,
        regval_list_t *regs,
        uint16_t size);

/* Function    : SensorI2cBusTest
   Description : Test whether the camera is connected on the I2C bus.
   Parameters  : Slave address of the camera to test.
 */
extern CyU3PReturnStatus_t
SensorI2cBusTest (
		uint8_t slaveAddr_RD);

/* Function    : SensorInit
   Description : Initialize the Front/Rear Camera with the default settings.
   Parameters  : EN_FRONT_CAM, EN_REAR_CAM to signals the right camera to initialize.
 */
extern CyU3PReturnStatus_t
SensorInit (
		CyBool_t EN_FRONT_CAM,
		CyBool_t EN_REAR_CAM);

/* Function     : SensorScaling_VGA
   Description  : Configure the camera for VGA 30fps video streaming.
   Parameters   : None
 */
extern CyU3PReturnStatus_t
SensorScaling_VGA_30fps (
		void);

/* Function     : SensorScaling_720p_30fps
   Description  : Configure the camera for 720p 30 fps video streaming.
   Parameters   : None.
 */
extern CyU3PReturnStatus_t
SensorScaling_720p_30fps (
		void);

/* Function     : SensorScaling_1080p_30fps
   Description  : Configure the camera for 1080p 30 fps video streaming.
   Parameters   : None.
 */
extern CyU3PReturnStatus_t
SensorScaling_1080p_30fps (
		void);

/* Function     : SensorScaling_5M
   Description  : Configure the camera for 5M 7.5 fps video streaming.
   Parameters   : None.
 */
extern CyU3PReturnStatus_t
SensorScaling_5M (
		void);

#ifdef BRIGHTNESS_CONTROL

/* Function    : SensorGetBrightness
   Description : Get the current brightness setting from the camera.
   Parameters  : None
 */
extern uint8_t
SensorGetBrightness (
        void);

/* Function    : SensorSetBrightness
   Description : Set the desired brightness level of the camera.
   Parameters  : Desired brightness level: -4,-3,-2,-1,0,+1,+2,+3,+4.
 */
extern CyU3PReturnStatus_t
SensorSetBrightness (
        uint8_t brightness);

#endif

#ifdef AUTOFOCUS

/* Function    : AF_init
   Description : Download the Camera Auto Focus Firmware inside the MCU via I2C.
   Parameters  : None.
 */
extern CyU3PReturnStatus_t
AF_init (
		void);

/* Function    : AF_set_up
   Description : Set Auto Focus.
   Parameters  : None.
 */
extern CyU3PReturnStatus_t
AF_single_focus (
		void);

/* Function    : AF_release_focus
   Description : Release Focus.
   Parameters  : None.
 */
extern CyU3PReturnStatus_t
AF_release_focus (
		void);

#endif

/* Function    : AEC_AGC_Auto
   Description : Auto/Manual AEC/AGC control
   Parameters  : Auto_Enable (if CyTrue Automatic AEC/AGC enable)
 */
extern CyU3PReturnStatus_t
AEC_AGC_Auto(
		CyBool_t Auto_Enable);

#ifdef STILL_IMAGE_CAPTURE_M2

/* Function    : ReadPreview_Reg
   Description : Read back the preview's Exposure, Gain, Maxlines, Average
   Parameters  : Pointer to a vector of preview settings
 */
extern CyU3PReturnStatus_t
ReadPreviewRegs(
		uint16_t *preReg);

/* Function    : CaptureSettings
   Description : Read back the capture's parameters, calculate the new ones and re-configure the camera
   Parameters  : preReg, pointer to a vector of preview settings.
 */
extern CyU3PReturnStatus_t
CaptureSettings(
		uint16_t *preReg);

/* Function    : Set_Night_Mode
   Description : Turn on/off the night mode
   Parameters  : Status of the night mode
 */
extern CyU3PReturnStatus_t
Set_Night_Mode(
		CyBool_t NightMode);

/* Function    : Get_HTS
   Description : Read HTS from register settings
   Parameters  : Pointer to the result variable
 */
extern CyU3PReturnStatus_t
Get_HTS(
		uint16_t *HTS);

/* Function    : Get_VTS
   Description : Read VTS from register settings
   Parameters  : Pointer to the result variable
 */
extern CyU3PReturnStatus_t
Get_VTS(
		uint16_t *VTS);

/* Function    : Get_Sysclk
   Description : Get the sysclk of the camera from register settings
   Parameters  : Pointer to the result variable
 */
extern CyU3PReturnStatus_t
Get_Sysclk(
		uint16_t *sysclk);

#endif

/* Function    : Get_Light_Frequency
   Description : Get the light frequency
   Parameters  : Pointer to the result variable
 */
extern CyU3PReturnStatus_t
Get_Light_Frequency(
		uint8_t *light_freq);

/* Function    : Bypass_Reg
   Description : Bypass the internal regulator of the camera
   Parameters  : None
 */
extern CyU3PReturnStatus_t
Bypass_Reg(
		void);

/* Function    : Mirror_Flip
   Description : This function perform the mirror action on the image, the flip action or both
   Parameters  : Mir, indicate if mirror is required. Flip, indicate if flip is required
 */
extern CyU3PReturnStatus_t
Mirror_Flip(
		CyBool_t Mir,
		CyBool_t Flip);


#endif /* _INCLUDED_SENSOR_H_ */


