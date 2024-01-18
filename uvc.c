/*

 Company: 	TEXA S.p.a.
 Author: 	Alberto Ferraro
 Date: 		19/10/16
 Version: 	1.0		- Change in cyfxuvcdscr.c USB descriptor -

 File Description:
 This file defines the main functions to create the threads of the application, to manage the DMA transfers
 and the UVC control commands.

*/

/* This project implements a USB Video Class device that streams uncompressed video
   data from two image sensors (front/rear cameras) to a USB host PC.

   Please refer to the Cypress Application Note: "AN75779: Interfacing an Image
   Sensor to EZ-USB FX3 in a USB video class (UVC) Framework" (http://www.cypress.com/?rID=62824)
   for a detailed design description of a similar application.

   As the UVC class driver on Windows hosts does not support burst enabled Isochronous
   endpoints on USB 3.0, this implementation makes use of Bulk endpoints for the video
   streaming.

   Three video formats are supported when the system functions on a USB 3.0 link:
     1. 720p (1280 * 720) video
     2. VGA (640 * 480) video
     3. 1080p (1920x1080) video
   Only the VGA video stream is supported on a USB 2.0 link, due to bandwidth limitations.

   The video streaming is accomplished with the help of a many-to-one manual DMA channel.
   Two producer sockets are used to receive data on the GPIF side to prevent data loss. The
   data is aggregated into one pipe and sent to the USB host over a bulk endpoint; after the
   addition of appropriate UVC headers.

   This firmware application makes use of two threads:
     1. The video streaming thread is responsible for handling the USB video streaming.
        If the UVC host has enabled video streaming, this thread continuously waits for
        a filled buffer, adds the appropriate UVC headers and commits the data. This
        thread also ensures the DMA multi-channel is reset and restarted at the end of
        each video frame. This thread is only idle when the UVC host has not enable video
        streaming.
     2. The UVC control thread handles UVC class specific requests that arrive on the
        control endpoint. The USB setup callback sets up events to notify this thread that
        a request has been received, and the thread handles them as soon as possible.
 */

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3uart.h>
#include <cyu3gpif.h>
#include <cyu3i2c.h>
#include <cyu3gpio.h>
#include <cyu3pib.h>
#include <cyu3utils.h>
#include <cyu3mmu.h>

#include "uvc.h"
#include "sensor.h"
#include "cyfxgpif2config.h"		// Obtained with GPIF II Designer


/*************************************************************************************************
                                         Global Variables
 *************************************************************************************************/
static CyU3PThread   uvcAppThread;                      /* UVC video streaming thread. */
static CyU3PThread   uvcAppEP0Thread;                   /* UVC control request handling thread. */
static CyU3PEvent    glFxUVCEvent;                      /* Event group used to signal threads. */
CyU3PDmaMultiChannel glChHandleUVCStream;               /* DMA multi-channel handle. */

/* Current UVC control request fields. See USB specification for definition. */
uint8_t  bmReqType, bRequest;                           /* bmReqType and bRequest fields. */
uint16_t wValue, wIndex, wLength;                       /* wValue, wIndex and wLength fields. */

CyBool_t        isUsbConnected = CyFalse;               /* Whether USB connection is active. */
CyBool_t 		isReadyToStream = CyFalse;				/* Whether Camera is active and ready to stream data */
CyU3PUSBSpeed_t usbSpeed = CY_U3P_NOT_CONNECTED;        /* Current USB connection speed. */
CyBool_t        clearFeatureRqtReceived = CyFalse;      /* Whether a CLEAR_FEATURE (stop streaming) request has been received. */
CyBool_t        streamingStarted = CyFalse;             /* Whether USB host has started streaming data */
CyBool_t		hold_focus = CyFalse;					/* Status of the single focus (if CyTrue is active) */


#ifdef BACKFLOW_DETECT
uint8_t back_flow_detected = 0;                         /* Whether buffer overflow error is detected. The host application consumes buffer slowly. */
#endif

#ifdef USB_DEBUG_INTERFACE
CyU3PDmaChannel  glDebugCmdChannel;                     /* Channel to receive debug commands on. */
CyU3PDmaChannel  glDebugRspChannel;                     /* Channel to send debug responses on. */
uint8_t         *glDebugRspBuffer;                      /* Buffer used to send debug responses. */
#endif


/* UVC Probe Control Settings for a USB 3.0 connection. */
uint8_t glProbeCtrl[CY_FX_UVC_MAX_PROBE_SETTING] = {
    0x00, 0x00,                 				/* bmHint : no hit */
    0x01,                       				/* Use 1st Video format index */
    0x01,                       				/* Use 1st Video frame index: 1080p 10fps */
    0x2A, 0x2C, 0x0A, 0x00,     				/* Desired frame interval in the unit of 100ns: 15 fps */
    0x00, 0x00,                 				/* Key frame rate in key frame/video frame units: only applicable
                                   	   	   	   	   to video streaming with adjustable compression parameters */
    0x00, 0x00,                 				/* PFrame rate in PFrame / key frame units: only applicable to
                                   	   	   	   	   video streaming with adjustable compression parameters */
    0x00, 0x00,                 				/* Compression quality control: only applicable to video streaming
                                   	   	   	   	   with adjustable compression parameters */
    0x00, 0x00,                 				/* Window size for average bit rate: only applicable to video
                                   	   	   	   	   streaming with adjustable compression parameters */
    0x00, 0x00,                 				/* Internal video streaming i/f latency in ms */
    0x00, 0x48, 0x3F, 0x00,     				/* Max video frame size in bytes (take from OV5640 descriptor) */
    0x00, 0x40, 0x00, 0x00      				/* No. of bytes device can rx in single payload = 16 KB */
};

/* UVC Probe Control Setting for a USB 2.0 connection. */
uint8_t glProbeCtrl20[CY_FX_UVC_MAX_PROBE_SETTING] = {
    0x00, 0x00,                 				/* bmHint : no hit */
    0x01,                       				/* Use 1st Video format index */
    0x01,                       				/* Use 1st Video frame index: 640x480 30 fps */
    0x15, 0x16, 0x05, 0x00,     				/* Desired frame interval in the unit of 100ns: 30 fps */
    0x00, 0x00,                 				/* Key frame rate in key frame/video frame units: only applicable
                                   	   	   	   	   to video streaming with adjustable compression parameters */
    0x00, 0x00,                 				/* PFrame rate in PFrame / key frame units: only applicable to
                                   	   	   	   	   video streaming with adjustable compression parameters */
    0x00, 0x00,                 				/* Compression quality control: only applicable to video streaming
                                   	   	   	   	   with adjustable compression parameters */
    0x00, 0x00,                 				/* Window size for average bit rate: only applicable to video
                                   	   	   	   	   streaming with adjustable compression parameters */
    0x00, 0x00,                 				/* Internal video streaming i/f latency in ms */
    0x00, 0x60, 0x09, 0x00,     				/* Max video frame size in bytes (take from OV5640 descriptor) */
    0x00, 0x40, 0x00, 0x00      				/* No. of bytes device can rx in single payload = 16 KB */
};

#ifdef STILL_IMAGE_CAPTURE_M2
/* Still Probe Control Setting */
uint8_t glStillProbeCtrl[CY_FX_UVC_MAX_STILL_PROBE_SETTING] =
{
    0x02,                            	/* Use 1st Video format index */
    0x01,                            	/* Use 1st Video frame index */
    0x00,							 	/* Compression quality */
    0x00, 0x8C, 0x33, 0x01,     		/* Max video frame size in bytes: 2592x1944x4 */
    0x00, 0x40, 0x00, 0x00             	/* No. of bytes device can rx in single payload */
};

/* Still Commit Control Setting */
static uint8_t glStillCommitCtrl[CY_FX_UVC_MAX_STILL_PROBE_SETTING_ALIGNED];

static uint8_t glCurrentStillFrameIndex = 0;			/* Still Image Size Patterns selected (Still Image Frame descriptor). */
static uint8_t glStillTriggerCtrl = 0;					/* bTrigger field of VS_STILL_IMAGE_TRIGGER_CONTROL selector  */
static uint16_t preReg[7];								/* Vector that contains preview's exposure, gain, maxlines, average
 	 	 	 	 	 	 	 	 	 	 	 	 	 	   VTS, HTS, Sysclk */
#endif

/* Video Probe Commit Control. This array is filled out when the host sends down the SET_CUR request. */
static uint8_t glCommitCtrl[CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED];

/* Scratch buffer used for handling UVC class requests with a data phase. */
static uint8_t glEp0Buffer[32];

/* UVC Header to be prefixed at the top of each 16 KB video data buffer. */
volatile uint8_t glUVCHeader[CY_FX_UVC_MAX_HEADER] =
{
    0x0C,                               /* Header Length */
    0x8C,                               /* Bit field header field */
    0x00, 0x00, 0x00, 0x00,             /* Presentation time stamp field */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* Source clock reference field */
};

static volatile CyBool_t hitFV = CyFalse;               /* Whether end of frame (FV) signal has been hit. */
static volatile CyBool_t gpif_initialized = CyFalse;    /* Whether the GPIF init function has been called. */
static volatile uint16_t prodCount = 0, consCount = 0;  /* Count of buffers received and committed during
                                                           the current video frame. */
static volatile CyBool_t gotPartial = CyFalse;          /* Helps track the last partial buffer to make sure it is committed to USB.*/
static CyBool_t glStillCaptureStart = CyFalse;			/* Flag that indicate the start of Still Image Capture action. */
static CyBool_t glStillCaptureFinish = CyFalse;			/* Flag that indicate the end of Still Image Capture action. */
static uint8_t glCaptureFrame = 0;						/* Frame number in Capture phase (we must capture the third frame)  */
static CyBool_t glDiscardBuffer = CyFalse;				/* If CyTrue we discard the buffer produced by the camera */

/* Selection of camera sensors and flash state flags */

/* Before gpio switching event */
static CyBool_t EN_FRONT_CAM = CyFalse;			/* Initial settings: Power-down mode */
static CyBool_t EN_REAR_CAM = CyFalse;			/* Initial settings: Power-down mode */
static CyBool_t EN_FLASH = CyTrue;				/* Flash LED disabled */

/* After gpio switching event */
static CyBool_t EN_FLASH_p = CyTrue;			/* Initial settings: Power-down mode */
static CyBool_t EN_REAR_CAM_p = CyFalse;		/* Initial settings: Power-down mode */
static CyBool_t EN_FRONT_CAM_p = CyFalse;		/* Flash LED disabled */

/* Gpio switching event flag */
static CyBool_t GpioSwitch = CyFalse;

/* Generic GPIO output and input pins definitions */
#define EN_1V5_GPIO 		(uint8_t)(38)
#define EN_FLASH_GPIO 	 	(uint8_t)(36)
#define EN_MCLK_CAM_GPIO 	(uint8_t)(27)
#define STROBE_FX3_GPIO	 	(uint8_t)(42)
#define EN_FRONT_CAM_GPIO 	(uint8_t)(33)
#define EN_REAR_CAM_GPIO 	(uint8_t)(35)

/* Generic output driven at GND to shield the sync cameras signals*/
#define CTL0	(uint8_t)(17)
#define CTL3	(uint8_t)(20)
#define CTL6	(uint8_t)(23)
#define CTL9	(uint8_t)(26)
#define CTL11	(uint8_t)(28)
#define CTL12	(uint8_t)(29)

/* GPIO output and input pins definitions for cameras control. */
#define PWDN_REAR_CAM_GPIO		(uint8_t)(24)
#define PWDN_FRONT_CAM_GPIO		(uint8_t)(21)
#define RESET_REAR_CAM_GPIO		(uint8_t)(25)
#define RESET_FRONT_CAM_GPIO	(uint8_t)(22)

/* Low power mode and wakeup defines and variables */
#define POLARITY 	(uint16_t)(0xFFFF)				/* Polarity of the wake-up sources. High = Wakeup when the corresponding source goes high. */
static uint16_t wakeupsource;						/* It contains the source of the wakeup */
static CyBool_t LP_Mode = CyFalse;					/* It signals whether we are in Low Power Mode  */

/* Format/Frame video definitions */
#define HD_1080p	(uint8_t)(0x11)				/* High-Nibble: Format index, Low-Nibble: Frame index */
#define VGA_640x480	(uint8_t)(0x12)				/* High-Nibble: Format index, Low-Nibble: Frame index */
#define _720p		(uint8_t)(0x13)				/* High-Nibble: Format index, Low-Nibble: Frame index */
#define _5Mp		(uint8_t)(0x14)				/* High-Nibble: Format index, Low-Nibble: Frame index */

/* Define mask to clear event flags */
#define LP_MASK			(uint32_t)(0xFF2C)
#define REAR_CAM_MASK	(uint32_t)(0xFF5C)
#define FRONT_CAM_MASK	(uint32_t)(0xFF9C)

/* Defines and variables for application Frame Timer */
#ifdef FRAME_TIMER
#define TIMER_PERIOD	(500)
static volatile CyBool_t UvcAbort = CyFalse;
static volatile CyBool_t UvcTimerStarted = CyFalse;
static CyU3PTimer        UvcTimer;
#endif



/* Custom Debug print function.
 * Comment the DEBUG_PRINT macro in uvc.h to remove the print messages. */
CyU3PReturnStatus_t myCyU3PDebugPrint (uint8_t priority, char *message)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

#ifdef DEBUG_PRINT
	apiRetStatus = CyU3PDebugPrint (priority, message);
#else
	INUTILE(priority);
	INUTILE(message);
#endif

	return apiRetStatus;
}


#ifdef FRAME_TIMER
static void UvcAppProgressTimer (uint32_t arg)
{
	INUTILE(arg);

    /* This frame has taken too long to complete. Abort it, so that the next frame can be started. */
    UvcAbort = CyTrue;
}
#endif


/* Power-down sequence (EN_FRONT_CAM = CyFalse, EN_REAR_CAM = CyFalse) */
CyU3PReturnStatus_t PowerDown(void)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

#ifdef LOW_POWER_MODE

#ifdef AUTOFOCUS
	/* Release Focus */
	if (hold_focus == CyTrue)
	{
		AF_release_focus();
	}
#endif

#ifdef FRAME_TIMER
	UvcAbort        = CyFalse;
	UvcTimerStarted = CyFalse;
	CyU3PTimerStop (&UvcTimer);
	CyU3PTimerDestroy (&UvcTimer);
#endif

	/* Power-down of the cameras. The camera hold the status of the internal registers. */
	SensorPwdn_R(CyTrue);
	SensorPwdn_F(CyTrue);
	CyU3PThreadSleep(1);			/* Wait 1 ms */

	/* Disable the Camera Oscillator */
	CyU3PGpioSetValue(EN_MCLK_CAM_GPIO, CyFalse);
	CyU3PThreadSleep(1);			/* Wait 1 ms */

	/* Disable the 1.5V power */
	CyU3PGpioSetValue(EN_1V5_GPIO, CyFalse);
	CyU3PThreadSleep(1);			/* Wait 1 ms */

	/* To enter in Suspend mode all FX3 blocks such as USB, GPIF, UART, I2C etc have to be de-initialized. Only the GPIO block can be
	left on, and its state will be restored when the device wakes up. */
	apiRetStatus = CyU3PI2cDeInit();
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	CyU3PGpifDisable(CyTrue);

	// Place the EP in NAK mode before cleaning up the pipe
	CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyTrue);
	CyU3PBusyWait(10);

	// Destroy DMA MultiChannel
	apiRetStatus = CyU3PDmaMultiChannelDestroy (&glChHandleUVCStream);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		return apiRetStatus;
	}

	// Flush the Endpoint memory
	CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);

	// Reset USB endpoints while functioning at Super speed. Clean error associated with endpoint
	CyU3PUsbResetEp(CY_FX_EP_BULK_VIDEO);

	// Flush the Endpoint memory
	CyU3PUsbFlushEp(CY_FX_EP_CONTROL_STATUS);

	// Reset USB endpoints while functioning at Super speed. Clean error associated with endpoint
	CyU3PUsbResetEp(CY_FX_EP_CONTROL_STATUS);

	// Reset and re-initialize the endpoint memory block on FX3
	CyU3PUsbResetEndpointMemories ();

	// Close NAK mode for the EP
	CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyFalse);
	CyU3PBusyWait(10);

	/* Clear the stall condition and sequence numbers. */
	CyU3PUsbStall(CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);

	/* Complete Control request handshake */
	CyU3PUsbAckSetup();

	apiRetStatus = CyU3PUsbStop();

#endif

	return apiRetStatus;
}


/* Power-up sequence and image sensor initialization */
CyU3PReturnStatus_t PowerUp()
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	/* Enable the 1.5V regulator for Image sensors core. */
	CyU3PGpioSetValue(EN_1V5_GPIO, CyTrue);
	CyU3PThreadSleep(10);			/* Wait 10 ms */

	/* Hold the unselected camera sensor in power-down mode and power-up the selected camera */
	if (EN_FRONT_CAM && (!EN_REAR_CAM))
	{
		SensorReset_R();			/* Hard reset of the rear image sensors and wait 20 ms */
		SensorPwdn_R(CyTrue);		/* Power-down the rear camera */
		CyU3PThreadSleep (20);		/* Wait 20 ms */
		SensorPwdn_F(CyFalse);		/* Power-up the front camera */
	}
	else if (EN_REAR_CAM)
	{
		SensorReset_F();			/* Hard reset of the front image sensors and wait 20 ms */
		SensorPwdn_F(CyTrue);		/* Power-down the front camera */
		CyU3PThreadSleep (20);		/* Wait 20 ms */
		SensorPwdn_R(CyFalse);		/* Power-up the rear camera */
	}
	else
	{
		myCyU3PDebugPrint (4, "Power-down mode! \r\n");
		return 1;
	}

	/* Enable the Camera Oscillator. */
	CyU3PGpioSetValue(EN_MCLK_CAM_GPIO, CyTrue);
	CyU3PThreadSleep(100);			/* Wait 100 ms */

	/* Camera initialization. Reset and then initialize the image sensor with a default configuration. */
	apiRetStatus = SensorInit(EN_FRONT_CAM, EN_REAR_CAM);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "Camera Initialization Error \r\n");
	}

	/* Cameras power-down */
	SensorPwdn_F(CyTrue);			/* Power-down of the Front Camera */
	SensorPwdn_R(CyTrue);			/* Power-down of the Rear Camera */

	CyU3PThreadSleep (50);			/* Wait 50 ms */
	return apiRetStatus;
}


/* Add the UVC packet header to the top of the specified DMA buffer. */
void CyFxUVCAddHeader (	uint8_t *buffer_p,              /* Buffer pointer */
        				uint8_t frameInd )              /* EOF or normal frame indication */
{
    /* Copy header to buffer */
    CyU3PMemCopy (buffer_p, (uint8_t *)glUVCHeader, CY_FX_UVC_MAX_HEADER);

    /* The EOF flag needs to be set if this is the last packet for this video frame. */
    if (frameInd & CY_FX_UVC_HEADER_EOF)
    {
        buffer_p[1] |= CY_FX_UVC_HEADER_EOF;
    }
}


/* Application Error Handler */
void CyFxAppErrorHandler(
		CyU3PReturnStatus_t apiRetStatus  /* API return status */
		)
{
	INUTILE(apiRetStatus);

    /* This function is hit when we have hit a critical application error. This is not
       expected to happen, and the current implementation of this function does nothing
       except stay in a loop printing error messages through the UART port.

       This function can be modified to take additional error handling actions such
       as cycling the USB connection or performing a warm reset.
     */
    for (;;)
    {
        myCyU3PDebugPrint (4, "Error handler...\r\n");
        CyU3PThreadSleep (1000);
    }
}


/* This function performs the operations for a Video Streaming Abort.
   This is called every time there is a USB reset, suspend or disconnect event.
 */
static void CyFxUVCApplnAbortHandler (void)
{
	uint32_t flag;
	if (CyU3PEventGet (&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_AND, &flag, CYU3P_NO_WAIT) == CY_U3P_SUCCESS)
	{
        /* Clear the Video Stream Request Event */
        CyU3PEventSet (&glFxUVCEvent, ~(CY_FX_UVC_STREAM_EVENT), CYU3P_EVENT_AND);

        /* Set Video Stream Abort Event */
        CyU3PEventSet (&glFxUVCEvent, CY_FX_UVC_STREAM_ABORT_EVENT, CYU3P_EVENT_OR);
	}
}


/* This is the Callback function to handle the USB Events */
static void CyFxUVCApplnUSBEventCB (
        CyU3PUsbEventType_t 	evtype,  /* Event type */
        uint16_t             	evdata  /* Event data */
        )
{
	INUTILE(evdata);

    switch (evtype)
    {
        case CY_U3P_USB_EVENT_RESET:
            myCyU3PDebugPrint (4, "RESET encountered...\r\n");
            CyU3PGpifDisable (CyTrue);
            gpif_initialized = 0;
            streamingStarted = CyFalse;
            CyFxUVCApplnAbortHandler ();
            break;

        case CY_U3P_USB_EVENT_SUSPEND:
            myCyU3PDebugPrint (4, "SUSPEND encountered...\r\n");
            CyU3PGpifDisable (CyTrue);
            gpif_initialized = 0;
            streamingStarted = CyFalse;
            CyFxUVCApplnAbortHandler ();
            break;

        case CY_U3P_USB_EVENT_DISCONNECT:
            myCyU3PDebugPrint (4, "USB disconnected...\r\n");
            CyU3PGpifDisable (CyTrue);
            gpif_initialized = 0;
            isUsbConnected = CyFalse;
            streamingStarted = CyFalse;
            CyFxUVCApplnAbortHandler ();
            break;

#ifdef BACKFLOW_DETECT
        case CY_U3P_USB_EVENT_EP_UNDERRUN:
            myCyU3PDebugPrint (4, "CY_U3P_USB_EVENT_EP_UNDERRUN encountered...\r\n");
            break;
#endif

        default:
            break;
    }
}


/* Callback to handle the USB Setup Requests and UVC Class events */
static CyBool_t CyFxUVCApplnUSBSetupCB (
        uint32_t setupdat0, /* SETUP Data 0 */
        uint32_t setupdat1  /* SETUP Data 1 */
        )
{
    CyBool_t uvcHandleReq = CyFalse;
    uint32_t status;

    /* Obtain Request Type and Request */
    bmReqType = (uint8_t)(setupdat0 & CY_FX_USB_SETUP_REQ_TYPE_MASK);
    bRequest  = (uint8_t)((setupdat0 & CY_FX_USB_SETUP_REQ_MASK) >> 8);
    wValue    = (uint16_t)((setupdat0 & CY_FX_USB_SETUP_VALUE_MASK) >> 16);
    wIndex    = (uint16_t)(setupdat1 & CY_FX_USB_SETUP_INDEX_MASK);
    wLength   = (uint16_t)((setupdat1 & CY_FX_USB_SETUP_LENGTH_MASK) >> 16);

    /* Check for UVC Class Requests */
    switch (bmReqType)
    {
        case CY_FX_USB_UVC_GET_REQ_TYPE:
        case CY_FX_USB_UVC_SET_REQ_TYPE:
            /* UVC Specific requests are handled in the EP0 thread. */
            switch (wIndex & 0xFF)
            {
                case CY_FX_UVC_CONTROL_INTERFACE:
                    {
                        uvcHandleReq = CyTrue;
                        status = CyU3PEventSet (&glFxUVCEvent, CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT,
                                CYU3P_EVENT_OR);
                        if (status != CY_U3P_SUCCESS)
                        {
                            myCyU3PDebugPrint (4, "Set CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT Failed \n");
                            CyU3PUsbStall (0, CyTrue, CyFalse);
                        }
                    }
                    break;

                case CY_FX_UVC_STREAM_INTERFACE:
                    {
                        uvcHandleReq = CyTrue;
                        status = CyU3PEventSet (&glFxUVCEvent, CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT,
                                CYU3P_EVENT_OR);
                        if (status != CY_U3P_SUCCESS)
                        {
                            /* Error handling */
                            myCyU3PDebugPrint (4, "Set CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT Failed \n");
                            CyU3PUsbStall (0, CyTrue, CyFalse);
                        }
                    }
                    break;

                default:
                    break;
            }
            break;

        case CY_FX_USB_SET_INTF_REQ_TYPE:
            if (bRequest == CY_FX_USB_SET_INTERFACE_REQ)
            {
            	/* MAC OS sends Set Interface Alternate Setting 0 command after
            	 * stopping to stream. This application needs to stop streaming. */
                if ((wIndex == CY_FX_UVC_STREAM_INTERFACE) && (wValue == 0))
                {
                	/* Stop GPIF state machine to stop data transfers through FX3 */
                	myCyU3PDebugPrint (4, "Alternate setting 0..\r\n");
                    CyU3PGpifDisable (CyTrue);
                    gpif_initialized = 0;
                    streamingStarted = CyFalse;
                    /* Place the EP in NAK mode before cleaning up the pipe. */
                    CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyTrue);
                    CyU3PBusyWait (100);

                    /* Reset and flush the endpoint pipe. */
                    CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
                    CyU3PUsbFlushEp (CY_FX_EP_BULK_VIDEO);
                    CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyFalse);
                    CyU3PBusyWait (100);

                    /* Clear the stall condition and sequence numbers. */
                    CyU3PUsbStall (CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);
                    uvcHandleReq = CyTrue;
                    /* Complete Control request handshake */
                    CyU3PUsbAckSetup ();
                    /* Indicate stop streaming to main thread */
                    clearFeatureRqtReceived = CyTrue;
                    CyFxUVCApplnAbortHandler ();

                }
            }
            break;

        case CY_U3P_USB_TARGET_ENDPT:
            if (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
            {
                if (wIndex == CY_FX_EP_BULK_VIDEO)
                {
                	/* Windows OS sends Clear Feature Request after it stops streaming,
                	 * however MAC OS sends clear feature request right after it sends a
                	 * Commit -> SET_CUR request. Hence, stop streaming only of streaming
                	 * has started. */
                    if (streamingStarted == CyTrue)
                    {
                    	myCyU3PDebugPrint (4, "Clear feature request detected..\r\n");

                        /* Disable the GPIF state machine. */
                        CyU3PGpifDisable (CyTrue);
                        gpif_initialized = 0;
                        streamingStarted = CyFalse;

                        /* Place the EP in NAK mode before cleaning up the pipe. */
                        CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyTrue);
                        CyU3PBusyWait (100);

                        /* Reset and flush the endpoint pipe. */
                        CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
                        CyU3PUsbFlushEp (CY_FX_EP_BULK_VIDEO);
                        CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyFalse);
                        CyU3PBusyWait (100);

                        /* Clear the stall condition and sequence numbers. */
                        CyU3PUsbStall (CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);

                        uvcHandleReq = CyTrue;
                        /* Complete Control request handshake */
                        CyU3PUsbAckSetup ();
                        /* Indicate stop streaming to main thread */
                        clearFeatureRqtReceived = CyTrue;
                        CyFxUVCApplnAbortHandler ();
                    }
                    else
                    {
                        uvcHandleReq = CyTrue;
                        CyU3PUsbAckSetup ();
                    }
                }
            }
            break;

        default:
            break;
    }

    /* Return status of request handling to the USB driver */
    return uvcHandleReq;
}


/* DMA callback providing notification when each buffer has been sent out to the USB host.
 * This is used to track whether all of the data has been sent out.
 */
void CyFxUvcApplnDmaCallback (CyU3PDmaMultiChannel *multiChHandle, CyU3PDmaCbType_t type,CyU3PDmaCBInput_t *input)
{
	INUTILE(multiChHandle);
	INUTILE(input);

    if (type == CY_U3P_DMA_CB_CONS_EVENT)
    {
        consCount++;
        streamingStarted = CyTrue;
    }
}


/*
 * This function is called from the GPIF callback when we have reached the end of a video frame.
 * The DMA buffer containing the last part of the frame may not have been committed, and need to
 * be manually wrapped up. This function uses the current GPIF state ID to identify the socket on
 * which this last buffer is pending, and then uses the CyU3PDmaMultiChannelSetWrapUp function
 * to commit the buffer.
 */
static uint8_t CyFxUvcAppCommitEOF (CyU3PDmaMultiChannel *handle,           /* Handle to DMA channel. */
        							uint8_t stateId)                         /* Current GPIF state ID. */
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    uint8_t socket = 0xFF;      /*  Invalid value. */

    /* Verify that the current state is a terminal state for the GPIF state machine. */
    switch (stateId)
    {
        case FULL_BUF_IN_SCK0:
        case FULL_BUF_IN_SCK1:
            /* Buffer is already full and would have been committed. Do nothing. */
            break;

        case PARTIAL_BUF_IN_SCK0:
        {
            socket = 0;
            break;
        }
        case PARTIAL_BUF_IN_SCK1:
        {
            socket = 1;
            break;
        }
        default:
            /* Unexpected current state. Return error. */
            return 1;
    }

    if (socket != 0xFF)
    {
        /* We have a partial buffer. Commit the buffer manually. The Wrap Up API, here, helps produce a
           partially filled buffer on the producer side. This action will cause CyU3PDmaMultiChannelGetBuffer API
           in the UVCAppThread_Entry function to succeed one more time with less than full producer buffer count */

    	apiRetStatus = CyU3PDmaMultiChannelSetWrapUp (handle, socket);
        if (apiRetStatus != CY_U3P_SUCCESS)
        {
            myCyU3PDebugPrint (4, "Channel Set WrapUp failed\r\n");
            // CyFxAppErrorHandler (apiRetStatus);		// COMMENT! If inserted Timeout problem in DMA Get buffer
        }

#ifdef STILL_IMAGE_CAPTURE_M2
        else
        {
        	gotPartial = CyTrue; /* Flag is set to indicate the partial buffer is acquired */
        }
#endif
    }

    return 0;
}


/* GpioCB callback function is invoked when a change in the state of EN_FRONT_CAM_GPIO, EN_REAR_CAM_GPIO or EN_FLASH_GPIO
 * pins is detected */
void CyFxGpioCB(uint8_t gpioId)
{
	INUTILE(gpioId);

	CyU3PBusyWait(10);		/* Wait 10 us to debounce */

	/* Write the global variables with the state of the pins EN_FRONT_CAM_GPIO, EN_REAR_CAM_GPIO, EN_FLASH_GPIO */
	CyU3PGpioSimpleGetValue (EN_FRONT_CAM_GPIO, &EN_FRONT_CAM_p);
	CyU3PGpioSimpleGetValue (EN_REAR_CAM_GPIO, &EN_REAR_CAM_p);
	CyU3PGpioSimpleGetValue (EN_FLASH_GPIO, &EN_FLASH_p);

	/* Set a Gpio switching flag */
	GpioSwitch = CyTrue;
}


/* GpifCB callback function is invoked when FV triggers GPIF interrupt */
void CyFxGpifCB(CyU3PGpifEventType event, uint8_t currentState)
{
	if (event == CYU3P_GPIF_EVT_SM_INTERRUPT)
	    {
	        // hitFV = CyTrue;
	        if (CyFxUvcAppCommitEOF (&glChHandleUVCStream, currentState) != CY_U3P_SUCCESS)
	            myCyU3PDebugPrint (4, "Commit EOF failed!\n");
	    }
}


/* This function initializes the Debug Module for the UVC Application */
/*
static void CyFxUVCApplnDebugInit(void)
{
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    // Initialize the UART for printing debug messages
    apiRetStatus = CyU3PUartInit ();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "UART initialization failed! \n");
        CyFxAppErrorHandler (apiRetStatus);
    }

    // Set UART Configuration
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit  = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity   = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma    = CyTrue;

    // Set the UART configuration
    apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    // Set the UART transfer
    apiRetStatus = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    // Initialize the Debug logger module.
    apiRetStatus = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 4);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler (apiRetStatus);
    }

    // Disable log message headers.
    CyU3PDebugPreamble (CyFalse);
}
*/


/* I2C initialization. */
static void CyFxUVCApplnI2CInit(void)
{
    CyU3PI2cConfig_t i2cConfig;;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    status = CyU3PI2cInit ();
    if (status != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "I2C initialization failed!\n");
        CyFxAppErrorHandler (status);
    }

    /*  Set I2C Configuration */
    i2cConfig.bitRate    = 100000;      /*  100 KHz */
    i2cConfig.isDma      = CyFalse;
    i2cConfig.busTimeout = 0xffffffffU;
    i2cConfig.dmaTimeout = 0xffff;

    status = CyU3PI2cSetConfig (&i2cConfig, 0);
    if (CY_U3P_SUCCESS != status)
    {
        myCyU3PDebugPrint (4, "I2C configuration failed!\n");
        CyFxAppErrorHandler (status);
    }
}


#ifdef BACKFLOW_DETECT
static void CyFxUvcAppPibCallback (
        CyU3PPibIntrType cbType,
        uint16_t cbArg)
{
    if ((cbType == CYU3P_PIB_INTR_ERROR) && ((cbArg == 0x1005) || (cbArg == 0x1006)))
    {
        if (!back_flow_detected)
        {
            CyU3PDebugPrint (4, "Backflow detected...\r\n");
            back_flow_detected = 1;
        }
    }
}
#endif

#ifdef USB_DEBUG_INTERFACE
static void
CyFxUvcAppDebugCallback (
        CyU3PDmaChannel   *handle,
        CyU3PDmaCbType_t   type,
        CyU3PDmaCBInput_t *input)
{
    if (type == CY_U3P_DMA_CB_PROD_EVENT)
    {
        /* Data has been received. Notify the EP0 thread which handles the debug commands as well. */
        CyU3PEventSet (&glFxUVCEvent, CY_FX_USB_DEBUG_CMD_EVENT, CYU3P_EVENT_OR);
    }
}
#endif


/* Initialize and enable USB connection (and DMA channel). */
CyU3PReturnStatus_t USBInit(void)
{
	CyU3PDmaMultiChannelConfig_t dmaMultiConfig;
	CyU3PEpConfig_t              endPointConfig;
	CyU3PReturnStatus_t          apiRetStatus = CY_U3P_SUCCESS;

    /* USB and control endpoint initialization. */
    apiRetStatus = CyU3PUsbStart ();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "USB Function Failed to Start \n");
        CyFxAppErrorHandler (apiRetStatus);
        return apiRetStatus;
    }

    /* Setup the Callback to Handle the USB Setup Requests */
    CyU3PUsbRegisterSetupCallback (CyFxUVCApplnUSBSetupCB, CyFalse);

    /* Setup the Callback to Handle the USB Events */
    CyU3PUsbRegisterEventCallback (CyFxUVCApplnUSBEventCB);

    /* Register the USB device descriptors with the driver. */
	CyU3PUsbSetDesc (CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSBDeviceDscr);
	CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSBDeviceDscrSS);

	/* BOS and Device qualifier descriptors. */
	CyU3PUsbSetDesc (CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
	CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);

	/* String Descriptors */
	CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
	CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
	CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);

	/* Configuration descriptors. */
	CyU3PUsbSetDesc (CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
	CyU3PUsbSetDesc (CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
	CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);

    /* Configure the video streaming endpoint. */
    endPointConfig.enable   = 1;
    endPointConfig.epType   = CY_U3P_USB_EP_BULK;
    endPointConfig.pcktSize = CY_FX_EP_BULK_VIDEO_PKT_SIZE;
    endPointConfig.isoPkts  = 1;
    endPointConfig.burstLen = 16;
    endPointConfig.streams  = 0;
    apiRetStatus = CyU3PSetEpConfig (CY_FX_EP_BULK_VIDEO, &endPointConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        myCyU3PDebugPrint (4, "USB Set Endpoint config failed \n");
        CyFxAppErrorHandler (apiRetStatus);
        return apiRetStatus;
    }

    /* Configure the status interrupt endpoint.
       Note: This endpoint is not being used by the application as of now. This can be used in case
       UVC device needs to notify the host about any error conditions. A MANUAL_OUT DMA channel
       can be associated with this endpoint and used to send these data packets.
     */
    endPointConfig.enable   = 1;
    endPointConfig.epType   = CY_U3P_USB_EP_INTR;
    endPointConfig.pcktSize = 64;
    endPointConfig.isoPkts  = 0;
    endPointConfig.streams  = 0;
    endPointConfig.burstLen = 1;
    apiRetStatus = CyU3PSetEpConfig (CY_FX_EP_CONTROL_STATUS, &endPointConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        myCyU3PDebugPrint (4, "USB Set Endpoint config failed \n");
        CyFxAppErrorHandler (apiRetStatus);
        return apiRetStatus;
    }

    /* Create a DMA Manual channel for sending the video data to the USB host. */
    dmaMultiConfig.size           = CY_FX_UVC_STREAM_BUF_SIZE;
    dmaMultiConfig.count          = CY_FX_UVC_STREAM_BUF_COUNT;
    dmaMultiConfig.validSckCount  = 2;
    dmaMultiConfig.prodSckId [0]  = (CyU3PDmaSocketId_t)CY_U3P_PIB_SOCKET_0;
    dmaMultiConfig.prodSckId [1]  = (CyU3PDmaSocketId_t)CY_U3P_PIB_SOCKET_1;
    dmaMultiConfig.consSckId [0]  = (CyU3PDmaSocketId_t)(CY_U3P_UIB_SOCKET_CONS_0 | CY_FX_EP_VIDEO_CONS_SOCKET);
    dmaMultiConfig.prodAvailCount = 0;
    dmaMultiConfig.prodHeader     = 12;                 /* 12 byte UVC header to be added. */
    dmaMultiConfig.prodFooter     = 4;                  /* 4 byte footer to compensate for the 12 byte header. */
    dmaMultiConfig.consHeader     = 0;
    dmaMultiConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaMultiConfig.notification   = CY_U3P_DMA_CB_CONS_EVENT;
    dmaMultiConfig.cb             = CyFxUvcApplnDmaCallback;

    apiRetStatus = CyU3PDmaMultiChannelCreate (&glChHandleUVCStream, CY_U3P_DMA_TYPE_MANUAL_MANY_TO_ONE,
            &dmaMultiConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        myCyU3PDebugPrint (4, "DMA Channel Creation Failed \n");
        CyFxAppErrorHandler (apiRetStatus);
        return apiRetStatus;
    }

#ifdef USB_DEBUG_INTERFACE
    /* Configure the endpoints and create DMA channels used by the USB debug interface.
       The command (OUT) endpoint is configured in packet mode and enabled to receive data.
       Once the CY_U3P_DMA_CB_PROD_EVENT callback is received, the received data packet is
       processed and the data is returned through the CyU3PDmaChannelSetupSendBuffer API call.
     */

    endPointConfig.enable   = 1;
    endPointConfig.epType   = CY_U3P_USB_EP_BULK;
    endPointConfig.pcktSize = 1024;                     /* Use SuperSpeed settings here. */
    endPointConfig.isoPkts  = 0;
    endPointConfig.streams  = 0;
    endPointConfig.burstLen = 1;

    apiRetStatus = CyU3PSetEpConfig (CY_FX_EP_DEBUG_CMD, &endPointConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "Debug Command endpoint config failed \n");
        CyFxAppErrorHandler (apiRetStatus);
    }

    CyU3PUsbSetEpPktMode (CY_FX_EP_DEBUG_CMD, CyTrue);

    apiRetStatus = CyU3PSetEpConfig (CY_FX_EP_DEBUG_RSP, &endPointConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "Debug Response endpoint config failed \n");
        CyFxAppErrorHandler (apiRetStatus);
    }

    channelConfig.size           = 1024;
    channelConfig.count          = 1;
    channelConfig.prodSckId      = CY_U3P_UIB_SOCKET_PROD_0 | CY_FX_EP_DEBUG_CMD_SOCKET;
    channelConfig.consSckId      = CY_U3P_CPU_SOCKET_CONS;
    channelConfig.prodAvailCount = 0;
    channelConfig.prodHeader     = 0;
    channelConfig.prodFooter     = 0;
    channelConfig.consHeader     = 0;
    channelConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    channelConfig.notification   = CY_U3P_DMA_CB_PROD_EVENT;
    channelConfig.cb             = CyFxUvcAppDebugCallback;

    apiRetStatus = CyU3PDmaChannelCreate (&glDebugCmdChannel, CY_U3P_DMA_TYPE_MANUAL_IN, &channelConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "Debug Command channel create failed \n");
        CyFxAppErrorHandler (apiRetStatus);
    }

    apiRetStatus = CyU3PDmaChannelSetXfer (&glDebugCmdChannel, 0);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "Debug channel SetXfer failed \n");
        CyFxAppErrorHandler (apiRetStatus);
    }

    channelConfig.size           = 1024;
    channelConfig.count          = 0;           /* No buffers allocated. We will only use the SetupSend API. */
    channelConfig.prodSckId      = CY_U3P_CPU_SOCKET_PROD;
    channelConfig.consSckId      = CY_U3P_UIB_SOCKET_CONS_0 | CY_FX_EP_DEBUG_RSP_SOCKET;
    channelConfig.prodAvailCount = 0;
    channelConfig.prodHeader     = 0;
    channelConfig.prodFooter     = 0;
    channelConfig.consHeader     = 0;
    channelConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    channelConfig.notification   = 0;
    channelConfig.cb             = 0;

    apiRetStatus = CyU3PDmaChannelCreate (&glDebugRspChannel, CY_U3P_DMA_TYPE_MANUAL_OUT, &channelConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "Debug Response channel create failed \n");
        CyFxAppErrorHandler (apiRetStatus);
    }

    glDebugRspBuffer = (uint8_t *)CyU3PDmaBufferAlloc (1024);
    if (glDebugRspBuffer == 0)
    {
        myCyU3PDebugPrint (4, "Failed to allocate memory for debug buffer\r\n");
        CyFxAppErrorHandler (CY_U3P_ERROR_MEMORY_ERROR);
    }
#endif

    /* Enable USB connection from the FX3 device, preferably at USB 3.0 speed. */
    apiRetStatus = CyU3PConnectState (CyTrue, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "USB Connect failed \n");
        CyFxAppErrorHandler (apiRetStatus);
        return apiRetStatus;
    }

    /* Delay: Recommended by Cypress */
    CyU3PThreadSleep (10);

    /* Disable LPM here, immediately after the connect state function: Recommended by Cypress */
    apiRetStatus = CyU3PUsbLPMDisable();
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "Error in Link Power Management\r\n");
	}

    // Place the EP in NAK mode before cleaning up the pipe
	CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyTrue);
	CyU3PBusyWait(100);

	// Flush the Endpoint memory
	CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);

	// Reset USB endpoints while functioning at Super speed. Clean error associated with endpoint
	CyU3PUsbResetEp(CY_FX_EP_BULK_VIDEO);

	// Flush the Endpoint memory
	CyU3PUsbFlushEp(CY_FX_EP_CONTROL_STATUS);

	// Reset USB endpoints while functioning at Super speed. Clean error associated with endpoint
	CyU3PUsbResetEp(CY_FX_EP_CONTROL_STATUS);

	// Reset and re-initialize the endpoint memory block on FX3
	CyU3PUsbResetEndpointMemories ();

	// Close NAK mode for the EP
	CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyFalse);
	CyU3PBusyWait(100);

	/* Clear the stall condition and sequence numbers. */
	CyU3PUsbStall(CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);

	/* Complete Control request handshake */
	CyU3PUsbAckSetup();

    return apiRetStatus;
}


/* This function initializes the USB Module, creates event group,
   sets the enumeration descriptors, configures the Endpoints and
   configures the DMA module for the UVC Application */
static void CyFxUVCApplnInit(void)
{
    CyU3PReturnStatus_t          apiRetStatus = CY_U3P_SUCCESS;
    CyU3PGpioClock_t             gpioClock;
    CyU3PGpioSimpleConfig_t      gpioConfig;
    CyU3PPibClock_t              pibclock;

#ifdef USB_DEBUG_INTERFACE
    CyU3PDmaChannelConfig_t channelConfig;
#endif

    /* Create UVC event group */
    apiRetStatus = CyU3PEventCreate (&glFxUVCEvent);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "UVC Create Event failed \n");
        CyFxAppErrorHandler (apiRetStatus);
    }

#ifdef UVC_PTZ_SUPPORT
    CyFxUvcAppPTZInit ();
#endif

    isUsbConnected = CyFalse;
    clearFeatureRqtReceived = CyFalse;

    /* Init the GPIO module */
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 2;
    gpioClock.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc     = CY_U3P_SYS_CLK;
    gpioClock.halfDiv    = 0;

    /* Initialize Gpio interface */
    apiRetStatus = CyU3PGpioInit (&gpioClock, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "GPIO Init failed \n");
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* DQ0-DQ15, CTL0-CTL4 and PMODE0-PMODE2 pins are restricted and cannot be configured using I/O matrix configuration function,
       must use GpioOverride to configure they. */

    /* STROBE_FX3_GPIO permits to control the strobe flash from FX3 firmware */
	apiRetStatus = CyU3PDeviceGpioOverride (STROBE_FX3_GPIO, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyTrue;	/* LED disabled */
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (STROBE_FX3_GPIO, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

    /* PWDN_FRONT_CAM_GPIO is the power-down control pin of the front camera (high state = power-down mode) */
    apiRetStatus = CyU3PDeviceGpioOverride (PWDN_FRONT_CAM_GPIO, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "GPIO Override failed \n");
        CyFxAppErrorHandler (apiRetStatus);
    }

    gpioConfig.outValue    = CyTrue;	/* Power-down enabled */
    gpioConfig.driveLowEn  = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.inputEn     = CyFalse;
    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
    apiRetStatus           = CyU3PGpioSetSimpleConfig (PWDN_FRONT_CAM_GPIO, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* CTL[0] driven to GND (to shield the sync camera signals) */
    apiRetStatus = CyU3PDeviceGpioOverride (CTL0, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyFalse;	/* Driven at GND */
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (CTL0, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* CTL[3] driven to GND (to shield the sync camera signals) */
	apiRetStatus = CyU3PDeviceGpioOverride (CTL3, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyFalse;	/* Driven at GND */
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (CTL3, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* CTL[6] driven to GND (to shield the sync camera signals) */
	 apiRetStatus = CyU3PDeviceGpioOverride (CTL6, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyFalse;	/* Driven at GND */
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (CTL6, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* CTL[9] driven to GND (to shield the sync camera signals) */
	 apiRetStatus = CyU3PDeviceGpioOverride (CTL9, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyFalse;	/* Driven at GND */
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (CTL9, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* CTL[11] driven to GND (to shield the sync camera signals) */
	 apiRetStatus = CyU3PDeviceGpioOverride (CTL11, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyFalse;	/* Driven at GND */
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (CTL11, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* CTL[12] driven to GND (to shield the sync camera signals) */
	 apiRetStatus = CyU3PDeviceGpioOverride (CTL12, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyFalse;	/* Driven at GND */
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (CTL12, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* EN_MCLK_CAM_GPIO is the enable pin of the oscillator */
	apiRetStatus = CyU3PDeviceGpioOverride (EN_MCLK_CAM_GPIO, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyFalse;	/* MCLK_CAM output in high impedance */
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (EN_MCLK_CAM_GPIO, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* PWDN_REAR_CAM_GPIO is the power-down control pin of the rear camera (high state = power-down mode) */
	apiRetStatus = CyU3PDeviceGpioOverride (PWDN_REAR_CAM_GPIO, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyTrue;	/* Power-down enabled */
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (PWDN_REAR_CAM_GPIO, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* RESET_FRONT_CAM_GPIO is the reset pin of the front camera (Low state = Reset enabled) */
	apiRetStatus = CyU3PDeviceGpioOverride (RESET_FRONT_CAM_GPIO, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyFalse;	/* Reset enabled */
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (RESET_FRONT_CAM_GPIO, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* RESET_REAR_CAM_GPIO is the reset pin of the rear camera (Low state = Reset enabled) */
	apiRetStatus = CyU3PDeviceGpioOverride (RESET_REAR_CAM_GPIO, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyFalse;	/* Reset enabled */
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (RESET_REAR_CAM_GPIO, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* EN_1V5_GPIO permits to enable/disable the 1.5V of the camera core */
	apiRetStatus = CyU3PDeviceGpioOverride (EN_1V5_GPIO, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyFalse;	/* 1.5V Power disabled  */
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.inputEn     = CyFalse;
	gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (EN_1V5_GPIO, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* EN_FLASH_GPIO is an input controlled by the AAEON board. The flash LED will be ON when this signal is high.
	 * An interrupt is associated to this input. Interrupt is triggered on either edge of input. */
	apiRetStatus = CyU3PDeviceGpioOverride (EN_FLASH_GPIO, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyTrue;
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyFalse;
	gpioConfig.inputEn     = CyTrue;	/* Input */
	gpioConfig.intrMode    = CY_U3P_GPIO_INTR_BOTH_EDGE;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (EN_FLASH_GPIO, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* EN_FRONT_CAM_GPIO is an input controlled by the AAEON board. When high signal that the front camera is selected.
	 * An interrupt is associated to this input. Interrupt is triggered on either edge of input. */
	apiRetStatus = CyU3PDeviceGpioOverride (EN_FRONT_CAM_GPIO, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyTrue;
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyFalse;
	gpioConfig.inputEn     = CyTrue;	/* Input */
	gpioConfig.intrMode    = CY_U3P_GPIO_INTR_BOTH_EDGE;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (EN_FRONT_CAM_GPIO, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* EN_REAR_CAM_GPIO is an input controlled by the AAEON board. When high signal that the rear camera is selected.
	 * An interrupt is associated to this input. Interrupt is triggered on either edge of input. */
	apiRetStatus = CyU3PDeviceGpioOverride (EN_REAR_CAM_GPIO, CyTrue);
	if (apiRetStatus != 0)
	{
		myCyU3PDebugPrint (4, "GPIO Override failed \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	gpioConfig.outValue    = CyTrue;
	gpioConfig.driveLowEn  = CyTrue;
	gpioConfig.driveHighEn = CyFalse;
	gpioConfig.inputEn     = CyTrue;	/* Input */
	gpioConfig.intrMode    = CY_U3P_GPIO_INTR_BOTH_EDGE;
	apiRetStatus           = CyU3PGpioSetSimpleConfig (EN_REAR_CAM_GPIO, &gpioConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO Set Config Error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* Setup the Callback to Handle the switch camera, power-down mode, USB boot and flash control events */
	CyU3PRegisterGpioCallBack (CyFxGpioCB);

    /* Initialize the P-port. */
    pibclock.clkDiv      = 2;
    pibclock.clkSrc      = CY_U3P_SYS_CLK;
    pibclock.isDllEnable = CyFalse;
    pibclock.isHalfDiv   = CyFalse;

    apiRetStatus = CyU3PPibInit (CyTrue, &pibclock);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        myCyU3PDebugPrint (4, "PIB Function Failed to Start \n");
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Setup the Callback to Handle the GPIF INTR event */
    CyU3PGpifRegisterCallback (CyFxGpifCB);

#ifdef BACKFLOW_DETECT
    back_flow_detected = 0;
    CyU3PPibRegisterCallback (CyFxUvcAppPibCallback, CYU3P_PIB_INTR_ERROR);
#endif


    /* Write the global variables with the state of the pins EN_FRONT_CAM_GPIO, EN_REAR_CAM_GPIO, EN_FLASH_GPIO */
	apiRetStatus = CyU3PGpioSimpleGetValue (EN_FRONT_CAM_GPIO, &EN_FRONT_CAM);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO error \n");
	}
	apiRetStatus = CyU3PGpioSimpleGetValue (EN_REAR_CAM_GPIO, &EN_REAR_CAM);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO error\n");
	}
	apiRetStatus = CyU3PGpioSimpleGetValue (EN_FLASH_GPIO, &EN_FLASH);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "GPIO error\n");
	}

	/* Drive the STROBE_FX3_GPIO to turn-on/off the flash LED. */
	CyU3PGpioSetValue(STROBE_FX3_GPIO, EN_FLASH);

    switch(((uint8_t)(EN_FRONT_CAM) << 1) | ((uint8_t)(EN_REAR_CAM)))
	{
		case 0:
			/* Set Low-power event (Suspend Mode (L2)) */
			apiRetStatus = CyU3PEventSet (&glFxUVCEvent, CY_FX_LP_MODE_EVENT, CYU3P_EVENT_OR);
			if (apiRetStatus != CY_U3P_SUCCESS)
			{
				myCyU3PDebugPrint (4, "Failed to set the CY_FX_LP_MODE_EVENT \n");
			}

			LP_Mode = CyTrue;
			break;

		default:
			/* Power-up sequence of the selected camera */
			apiRetStatus = PowerUp();
			if (apiRetStatus != CY_U3P_SUCCESS)
			{
				myCyU3PDebugPrint (4, "Board power-up error \n");
				CyFxAppErrorHandler (apiRetStatus);
			}

			break;
	}

    /* Initialize and enable USB connection (and DMA channel) */
    apiRetStatus = USBInit();
    if (apiRetStatus != CY_U3P_SUCCESS)
	{
		myCyU3PDebugPrint (4, "USB initialization error \n");
		CyFxAppErrorHandler (apiRetStatus);
	}
}


/*
 * Load the GPIF configuration on the GPIF-II engine. This operation is performed whenever a new video
 * streaming session is started.
 */
static void CyFxUvcAppGpifInit(void)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	apiRetStatus =  CyU3PGpifLoad ((CyU3PGpifConfig_t *) &CyFxGpifConfig);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
		/* Error Handling */
		myCyU3PDebugPrint (4, "Loading GPIF Configuration failed \r\n");
		CyFxAppErrorHandler (apiRetStatus);
	}

	/* Start the state machine from the designated start state. */
    apiRetStatus = CyU3PGpifSMStart (START, ALPHA_START);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error Handling */
        myCyU3PDebugPrint (4, "Starting GPIF state machine failed \r\n");
        CyFxAppErrorHandler (apiRetStatus);
    }
}


/*
 * Entry function for the UVC Application Thread
 */
void UVCAppThread_Entry()
{
    CyU3PDmaBuffer_t    produced_buffer;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    uint32_t flag;

#ifdef DEBUG_PRINT_FRAME_COUNT
    uint32_t frameCnt = 0;
#endif

    /* Initialize the Uart Debug Module */
    // CyFxUVCApplnDebugInit ();

    /* Initialize the I2C interface */
    CyFxUVCApplnI2CInit ();

    /* Initialize the UVC Application */
    CyFxUVCApplnInit ();

#ifdef FRAME_TIMER
    /* Create a timer to produce a frame timeout*/
    CyU3PTimerCreate (&UvcTimer, UvcAppProgressTimer, 0x00, TIMER_PERIOD, 0, CYU3P_NO_ACTIVATE);
#endif

    /*
       This thread continually checks whether video streaming is enabled, and commits video data if so.

       The CY_FX_UVC_STREAM_EVENT and CY_FX_UVC_STREAM_ABORT_EVENT event flags are monitored by this
       thread. The CY_FX_UVC_STREAM_EVENT event flag is enabled when the USB host sends a COMMIT control
       request to the video streaming interface, and stays ON as long as video streaming is enabled.

       The CY_FX_UVC_STREAM_ABORT_EVENT event indicates that we need to abort the video streaming. This
       only happens when we receive a CLEAR_FEATURE request indicating that streaming is to be stopped,
       or when we have a critical error in the data path. In both of these cases, the CY_FX_UVC_STREAM_EVENT
       event flag will be cleared before the CY_FX_UVC_STREAM_ABORT_EVENT event flag is enabled.

       This sequence ensures that we do not get stuck in a loop where we are trying to send data instead
       of handling the abort request.

       This thread continually checks whether LP_MODE, REAR_CAM_SEL or FRONT_CAM_SEL events flag are enabled
       and responds with their respective actions.
     */


    for (;;)
    {
    	if(GpioSwitch)
    	{
    		if (EN_FLASH != EN_FLASH_p)
			{
				EN_FLASH = EN_FLASH_p;		/* Update the Flash variable */

				/* Drive the STROBE_FX3_GPIO low to turn-on the flash LED. */
				CyU3PGpioSetValue(STROBE_FX3_GPIO, EN_FLASH);
			}

			if ((EN_FRONT_CAM != EN_FRONT_CAM_p) || (EN_REAR_CAM != EN_REAR_CAM_p))
			{
				EN_FRONT_CAM = EN_FRONT_CAM_p;		/* Update the Front camera selection variable */
				EN_REAR_CAM = EN_REAR_CAM_p;		/* Update the Rear camera selection variable */

				/* Events management */
				switch(((uint8_t)(EN_FRONT_CAM) << 1) | (uint8_t)(EN_REAR_CAM))
				{
					case 0:

#ifdef LOW_POWER_MODE

						// Clear the undesired Events
						apiRetStatus = CyU3PEventSet (&glFxUVCEvent, LP_MASK, CYU3P_EVENT_AND);
						if (apiRetStatus != CY_U3P_SUCCESS)
						{
							myCyU3PDebugPrint (4, "Failed to clear events \n");
						}

						/* Set the LP MODE event. */
						apiRetStatus = CyU3PEventSet (&glFxUVCEvent, CY_FX_LP_MODE_EVENT, CYU3P_EVENT_OR);
						if (apiRetStatus != CY_U3P_SUCCESS)
						{
							myCyU3PDebugPrint (4, "Failed to set the CY_FX_LP_MODE_EVENT \n");
						}

						LP_Mode = CyTrue;
#endif
						break;

					case 1:

#ifndef LOW_POWER_MODE
						// Clear the undesired Events
						apiRetStatus = CyU3PEventSet (&glFxUVCEvent, REAR_CAM_MASK, CYU3P_EVENT_AND);
						if (apiRetStatus != CY_U3P_SUCCESS)
						{
							myCyU3PDebugPrint (4, "Failed to clear events \n");
						}

						/* Set the REAR CAM SELECTED event. */
						apiRetStatus = CyU3PEventSet (&glFxUVCEvent, CY_FX_REAR_CAM_SEL_EVENT, CYU3P_EVENT_OR);
						if (apiRetStatus != CY_U3P_SUCCESS)
						{
							myCyU3PDebugPrint (4, "Failed to set the CY_FX_REAR_CAM_SEL_EVENT \n");
						}
#endif

						isReadyToStream = CyFalse;
						LP_Mode = CyFalse;
						break;

					case 2:

#ifndef LOW_POWER_MODE
						// Clear the undesired Events
						apiRetStatus = CyU3PEventSet (&glFxUVCEvent, FRONT_CAM_MASK, CYU3P_EVENT_AND);
						if (apiRetStatus != CY_U3P_SUCCESS)
						{
							myCyU3PDebugPrint (4, "Failed to clear events \n");
						}

						/* Set the FRONT CAM SELECTED event. */
						apiRetStatus = CyU3PEventSet (&glFxUVCEvent, CY_FX_FRONT_CAM_SEL_EVENT, CYU3P_EVENT_OR);
						if (apiRetStatus != CY_U3P_SUCCESS)
						{
							myCyU3PDebugPrint (4, "Failed to set the CY_FX_FRONT_CAM_SEL_EVENT \n");
						}
#endif

						isReadyToStream = CyFalse;
						LP_Mode = CyFalse;
						break;

					case 3:

						if (EN_FLASH)
						{
							/* USB BOOT event. There will be an hardware reset. */
						}
						else
						{
							/* Rear Camera Enabled with flash in strobe mode */
							/* Insert code here */
						}
						break;

					default:
						break;
				}
			}

			GpioSwitch = CyFalse;
    	}


    	/* Front camera selected */
    	if (CyU3PEventGet (&glFxUVCEvent, CY_FX_FRONT_CAM_SEL_EVENT, CYU3P_EVENT_AND, &flag,
    	                    CYU3P_NO_WAIT) == CY_U3P_SUCCESS)
    	{

#ifndef LOW_POWER_MODE
#ifdef AUTOFOCUS
    		/* Release Focus */
    		if (hold_focus == CyTrue)
    		{
    			apiRetStatus = AF_release_focus();
				if (apiRetStatus != CY_U3P_SUCCESS)
				{
					myCyU3PDebugPrint(4, "Error: Release Focus failed!\r\n");
				}
    		}
#endif

			/* Power-up sequence and front camera initialization. */
			apiRetStatus = PowerUp();
			if (apiRetStatus == CY_U3P_SUCCESS)
			{
				//isConfigured_F = CyTrue;
				myCyU3PDebugPrint (4, "Front camera initialized \n");
			}
			else
			{
				//isConfigured_F = CyFalse;
				myCyU3PDebugPrint (4, "Board power-up error \n");
				CyFxAppErrorHandler (apiRetStatus);
			}

    		// Clear variables
			streamingStarted = CyFalse;
			hitFV     = CyFalse;
			prodCount = 0;
			consCount = 0;
			clearFeatureRqtReceived = CyFalse;
			glCaptureFrame = 0;
			glStillCaptureStart = CyFalse;
			glStillCaptureFinish = CyFalse;
			glDiscardBuffer = CyFalse;
			gpif_initialized = CyFalse;
			isReadyToStream = CyFalse;

			// Close USB connection
			apiRetStatus = CyU3PConnectState(CyFalse, CyTrue);
			if (apiRetStatus != CY_U3P_SUCCESS)
			{
				myCyU3PDebugPrint (4, "USB Re-enumeration failed! \n");
			}

			CyU3PThreadSleep (10);

			// Re-enumerate the UVC device (new streaming request)
			apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
			if (apiRetStatus != CY_U3P_SUCCESS)
			{
				myCyU3PDebugPrint (4, "USB Re-enumeration failed! \n");
			}

			// Place the EP in NAK mode before cleaning up the pipe
			CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyTrue);
			CyU3PBusyWait(10);

			// Flush the Endpoint memory
			CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);

			// Reset USB endpoints while functioning at Super speed. Clean error associated with endpoint
			CyU3PUsbResetEp(CY_FX_EP_BULK_VIDEO);

			// Reset and re-initialize the endpoint memory block on FX3
			CyU3PUsbResetEndpointMemories ();

			// Close NAK mode for the EP
			CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyFalse);
			CyU3PBusyWait(10);

			/* Clear the stall condition and sequence numbers. */
			CyU3PUsbStall(CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);

			/* Complete Control request handshake */
			CyU3PUsbAckSetup();
#endif

			// Clear the Front camera selection Event
			CyU3PEventSet (&glFxUVCEvent, ~(CY_FX_FRONT_CAM_SEL_EVENT), CYU3P_EVENT_AND);
    	}

    	/* Rear camera selected */
    	else if (CyU3PEventGet (&glFxUVCEvent, CY_FX_REAR_CAM_SEL_EVENT, CYU3P_EVENT_AND, &flag,
    	    	                    CYU3P_NO_WAIT) == CY_U3P_SUCCESS)
		{

#ifndef LOW_POWER_MODE
#ifdef AUTOFOCUS
    		/* Release Focus */
			if (hold_focus == CyTrue)
			{
				apiRetStatus = AF_release_focus();
				if (apiRetStatus != CY_U3P_SUCCESS)
				{
					myCyU3PDebugPrint(4, "Error: Release Focus failed!\r\n");
				}
			}
#endif

			/* Power-up sequence and rear camera initialization. */
			apiRetStatus = PowerUp();
			if (apiRetStatus == CY_U3P_SUCCESS)
			{
				//isConfigured_R = CyTrue;
				myCyU3PDebugPrint (4, "Rear camera initialized \n");
			}
			else
			{
				//isConfigured_R = CyFalse;
				myCyU3PDebugPrint (4, "Board power-up error \n");
				CyFxAppErrorHandler (apiRetStatus);
			}

			// Clear variables
			streamingStarted = CyFalse;
			hitFV     = CyFalse;
			prodCount = 0;
			consCount = 0;
			clearFeatureRqtReceived = CyFalse;
			glCaptureFrame = 0;
			glStillCaptureStart = CyFalse;
			glStillCaptureFinish = CyFalse;
			glDiscardBuffer = CyFalse;
			gpif_initialized = CyFalse;
			isReadyToStream = CyFalse;

			// Close USB connection
			apiRetStatus = CyU3PConnectState(CyFalse, CyTrue);
			if (apiRetStatus != CY_U3P_SUCCESS)
			{
				myCyU3PDebugPrint (4, "USB Re-enumeration failed! \n");
			}

			CyU3PThreadSleep (10);

			// Re-enumerate the UVC device (new streaming request)
			apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
			if (apiRetStatus != CY_U3P_SUCCESS)
			{
				myCyU3PDebugPrint (4, "USB Re-enumeration failed! \n");
			}

			// Place the EP in NAK mode before cleaning up the pipe
			CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyTrue);
			CyU3PBusyWait (10);

			// Flush the Endpoint memory
			CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);

			// Reset USB endpoints while functioning at Super speed. Clean error associated with endpoint
			CyU3PUsbResetEp(CY_FX_EP_BULK_VIDEO);

			// Reset and re-initialize the endpoint memory block on FX3
			CyU3PUsbResetEndpointMemories();

			// Close NAK mode for the EP
			CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyFalse);
			CyU3PBusyWait(10);

			/* Clear the stall condition and sequence numbers. */
			CyU3PUsbStall(CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);

			/* Complete Control request handshake */
			CyU3PUsbAckSetup();

#endif

			// Clear the Rear camera selection Event
			CyU3PEventSet (&glFxUVCEvent, ~(CY_FX_REAR_CAM_SEL_EVENT), CYU3P_EVENT_AND);

		}

    	/* Low-Power mode selected */
		else if (CyU3PEventGet (&glFxUVCEvent, CY_FX_LP_MODE_EVENT, CYU3P_EVENT_AND, &flag,
									CYU3P_NO_WAIT) == CY_U3P_SUCCESS)
		{
			// Clear the Low Power mode Event
			CyU3PEventSet (&glFxUVCEvent, ~(CY_FX_LP_MODE_EVENT), CYU3P_EVENT_AND);

#ifdef LOW_POWER_MODE

			/* Power-down sequence (Standby Mode (L3)) */
			apiRetStatus = PowerDown();
			if (apiRetStatus == CY_U3P_SUCCESS)
			{
				/* Verify FX3 device is ready to enter in low power suspend mode. VBus signal as a source to wake FX3 from suspend/standby. */
				apiRetStatus = CyU3PSysCheckSuspendParams ( CY_U3P_SYS_USB_VBUS_WAKEUP_SRC, POLARITY);
				if (apiRetStatus == CY_U3P_SUCCESS)
				{
					// Places the FX3 device in low power standby mode
					apiRetStatus = CyU3PSysEnterSuspendMode ( CY_U3P_SYS_USB_VBUS_WAKEUP_SRC, POLARITY, &wakeupsource);

				}

				CyU3PThreadSleep(100);			/* Wait 100 ms (min. 10 ms) */
				LP_Mode = CyFalse;				/* Out of Power Down Mode */

				/* Re-initialize the I2C interface */
				CyFxUVCApplnI2CInit();

				/* Initialize and enable USB connection (and DMA channel) */
				USBInit();

#ifdef FRAME_TIMER
				/* Create a timer to produce a frame timeout */
				CyU3PTimerCreate (&UvcTimer, UvcAppProgressTimer, 0x00, TIMER_PERIOD, 0, CYU3P_NO_ACTIVATE);
#endif

				/* Drive the STROBE_FX3_GPIO low to turn-on the flash LED. */
				CyU3PGpioSetValue(STROBE_FX3_GPIO, EN_FLASH);

				/* Write the global variables with the state of the pins EN_FRONT_CAM_GPIO, EN_REAR_CAM_GPIO, EN_FLASH_GPIO */
				CyU3PGpioSimpleGetValue (EN_FRONT_CAM_GPIO, &EN_FRONT_CAM);
				CyU3PGpioSimpleGetValue (EN_REAR_CAM_GPIO, &EN_REAR_CAM);

				if ((EN_FRONT_CAM && (!EN_REAR_CAM)) || EN_REAR_CAM)
				{
					/* Power-up camera sequence. */
					apiRetStatus = PowerUp();
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint (4, "Board power-up error \n");
					}

					// Clear variables
					streamingStarted = CyFalse;
					hitFV     = CyFalse;
					prodCount = 0;
					consCount = 0;
					clearFeatureRqtReceived = CyFalse;
					glCaptureFrame = 0;
					glStillCaptureStart = CyFalse;
					glStillCaptureFinish = CyFalse;
					glDiscardBuffer = CyFalse;
					gpif_initialized = CyFalse;
				}
			}
			else
			{
				myCyU3PDebugPrint (4, "Board power-down error \n");
			}

			isReadyToStream = CyFalse;		/* Here we are ready to stream data */
#endif

		}

    	/* Video Stream Event */
    	else if ((CyU3PEventGet (&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_AND, &flag,
    			CYU3P_NO_WAIT) == CY_U3P_SUCCESS ) && isReadyToStream)
        {

			/* Check if we have a buffer ready to go. */
			apiRetStatus = CyU3PDmaMultiChannelGetBuffer (&glChHandleUVCStream, &produced_buffer, CYU3P_NO_WAIT);
			if (apiRetStatus == CY_U3P_SUCCESS)
			{

#ifdef STILL_IMAGE_CAPTURE_M2
				if(glDiscardBuffer)
				{
					/* Discard the buffer while we are waiting for the third frame in Still Capture */
					CyU3PDmaMultiChannelDiscardBuffer (&glChHandleUVCStream);
					if (apiRetStatus == CY_U3P_SUCCESS)
					{
						gotPartial = CyFalse;
					}
				}
				else
				{
					if (produced_buffer.count == CY_FX_UVC_BUF_FULL_SIZE)
					{
						CyFxUVCAddHeader (produced_buffer.buffer - CY_FX_UVC_MAX_HEADER, CY_FX_UVC_HEADER_FRAME);
					}
					else
					{
						/* If we have a partial buffer, this is guaranteed to be the end of the video frame for uncompressed images. */
						CyFxUVCAddHeader (produced_buffer.buffer - CY_FX_UVC_MAX_HEADER, CY_FX_UVC_HEADER_EOF);
						hitFV = CyTrue;
					}

					/* Commit the updated DMA buffer to the USB endpoint. */
					prodCount++;
					apiRetStatus = CyU3PDmaMultiChannelCommitBuffer (&glChHandleUVCStream, produced_buffer.count + CY_FX_UVC_MAX_HEADER, 0);
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						prodCount--;
						myCyU3PDebugPrint (4, "Error in multichannelcommitbuffer\r\n");
					}
				}

#else

				if (produced_buffer.count == CY_FX_UVC_BUF_FULL_SIZE)
				{
					CyFxUVCAddHeader (produced_buffer.buffer - CY_FX_UVC_MAX_HEADER, CY_FX_UVC_HEADER_FRAME);
				}
				else
				{
					/* If we have a partial buffer, this is guaranteed to be the end of the video frame for uncompressed images. */
					CyFxUVCAddHeader (produced_buffer.buffer - CY_FX_UVC_MAX_HEADER, CY_FX_UVC_HEADER_EOF);
					hitFV = CyTrue;
				}

				/* Commit the updated DMA buffer to the USB endpoint. */
				prodCount++;
				apiRetStatus = CyU3PDmaMultiChannelCommitBuffer (&glChHandleUVCStream, produced_buffer.count + CY_FX_UVC_MAX_HEADER, 0);
				if (apiRetStatus != CY_U3P_SUCCESS)
				{
					prodCount--;
					myCyU3PDebugPrint (4, "Error in multichannelcommitbuffer\r\n");
				}

#endif
			}

			/* If we have the end of frame signal and all of the committed data (including partial buffer)
			 * has been read by the USB host; we can reset the DMA channel and prepare for the next video frame.
			 */

#ifdef FRAME_TIMER
            if (((hitFV) && (prodCount == consCount)) || (UvcAbort))
#else
			if ((hitFV) && (prodCount == consCount))
#endif
			{
				prodCount = 0;
				consCount = 0;
				hitFV     = CyFalse;

#ifdef FRAME_TIMER
                UvcAbort  = CyFalse;
                UvcTimerStarted = CyFalse;
                CyU3PTimerStop (&UvcTimer);
#endif

#ifdef BACKFLOW_DETECT
				back_flow_detected = 0;
#endif

#ifdef DEBUG_PRINT_FRAME_COUNT
				CyU3PDebugPrint (4, "frame %d\r\n", frameCnt++);
#endif

				/* Toggle UVC header FRAME ID bit. Right also when we perform a still capture */
				glUVCHeader[1] ^= CY_FX_UVC_HEADER_FRAME_ID;

				/* Reset the DMA channel. */
				apiRetStatus = CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
				if (apiRetStatus != CY_U3P_SUCCESS)
				{
					myCyU3PDebugPrint (4, "DMA Channel Reset Failed \n");
				}

#ifdef STILL_IMAGE_CAPTURE_M2
				/* Host asked FX3 to initiate STILL IMAGE CAPTURE: we change the resolution and frame rate of the sensor and
				 * acquire the third frame */
				if (glStillCaptureStart && (glCaptureFrame == 0))
				{
					/* AEC/AGC manual mode */
					apiRetStatus = AEC_AGC_Auto(CyFalse);

#ifdef AUTOFOCUS
					/* Single Focus */
					apiRetStatus = AF_single_focus();
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint(4, "Single Focus failed!\r\n");
					}
					else
					{
						hold_focus = CyTrue;		/* hold_focus indicate that the focus is active */
					}
#endif

					/* Read back the preview's parameters */
					apiRetStatus = ReadPreviewRegs(preReg);
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint (4, "Reading preview registers failed!\n");
					}

					/* Turn off night mode for capture */
					apiRetStatus = Set_Night_Mode(CyFalse);
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint (4, "Reading preview registers failed!\n");
					}

					/* Change resolution to capture: 5Mp resolution */
					apiRetStatus = SensorScaling_5M();
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint (4, "Still Capture image sensor scaling \n");
					}

					/* Read back the capture's parameters, calculate the new ones and re-configure the camera */
					apiRetStatus = CaptureSettings(preReg);
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint (4, "Failed to re-configure the camera for capture!\n");
					}

					glUVCHeader[1] |= CY_FX_UVC_HEADER_STILL_IMAGE;
					glStillCaptureFinish = CyFalse;
					glDiscardBuffer = CyTrue;			/* Discard the buffers of the next 2 frames */
					glCaptureFrame++;
				}
				/* We must capture the 3rd frame */
				else if(glStillCaptureStart && (glCaptureFrame != 0))
				{
					glCaptureFrame++;
					if(glCaptureFrame == 3)
					{
						glStillCaptureFinish = CyTrue;
						glDiscardBuffer = CyFalse;		/* Get next buffers */
					}
				}
				/* We've done. Clear the STILL_IMAGE bit, release the focus and resume normal video streaming */
				else if(glStillCaptureFinish)
				{
					glCaptureFrame = 0;
					glStillCaptureStart = CyFalse;
					glStillCaptureFinish = CyFalse;
					glUVCHeader[1] &= ~CY_FX_UVC_HEADER_STILL_IMAGE;
					myCyU3PDebugPrint(4, "End still capture\n");

#ifdef AUTOFOCUS
					/* Release Focus */
					if (hold_focus == CyTrue)
					{
						apiRetStatus = AF_release_focus();
						if (apiRetStatus != CY_U3P_SUCCESS)
						{
							myCyU3PDebugPrint(4, "Error: Release Focus failed!\r\n");
						}
						else
						{
							hold_focus = CyFalse;
						}
					}
#endif
				}
				else
				{
					/* Nothing */
				}
#endif

				/* Start Channel Immediately */
				apiRetStatus = CyU3PDmaMultiChannelSetXfer (&glChHandleUVCStream, 0, 0);
				if (apiRetStatus != CY_U3P_SUCCESS)
				{
					myCyU3PDebugPrint (4, "DMA Channel Set Transfer Failed\n");
				}

				/* Jump to the start state of the GPIF state machine. 257 is used as an
				   arbitrary invalid state (> 255) number. */
				CyU3PGpifSMSwitch (257, 0, 257, 0, 2);

#ifdef FRAME_TIMER
                UvcAbort = CyFalse;
                CyU3PTimerModify (&UvcTimer, TIMER_PERIOD, 0);
#endif
			}
        }

    	/* If we have a stream abort request pending. */
        else if (CyU3PEventGet (&glFxUVCEvent, CY_FX_UVC_STREAM_ABORT_EVENT, CYU3P_EVENT_AND_CLEAR,
                        &flag, CYU3P_NO_WAIT) == CY_U3P_SUCCESS)
		{

#ifdef AUTOFOCUS
        	/* Release Focus */
        	if (hold_focus == CyTrue)
        	{
				apiRetStatus = AF_release_focus();
				if (apiRetStatus != CY_U3P_SUCCESS)
				{
					myCyU3PDebugPrint(4, "Error: Release Focus failed!\r\n");
				}
        	}
#endif

			/* Re-initialize following variables */
			hitFV     = CyFalse;
			gotPartial = CyFalse;
			prodCount = 0;
			consCount = 0;
			glCaptureFrame = 0;
			glStillCaptureStart = CyFalse;
			glStillCaptureFinish = CyFalse;
			glDiscardBuffer = CyFalse;
			isReadyToStream = CyFalse;
			gpif_initialized = CyFalse;

#ifdef FRAME_TIMER
			UvcAbort = CyFalse;
			UvcTimerStarted = CyFalse;
			CyU3PTimerStop(&UvcTimer);
#endif

			/* Cameras power-down */
			SensorPwdn_F(CyTrue);		// Power-down the front camera
			SensorPwdn_R(CyTrue);		// Power-down the rear camera

			if (clearFeatureRqtReceived)
			{
				apiRetStatus = CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
				if (apiRetStatus != CY_U3P_SUCCESS)
				{
					myCyU3PDebugPrint (4, "DMA channel reset error\r\n");
				}

			}

			clearFeatureRqtReceived = CyFalse;

		}

		else
		{
			/*  We are essentially idle at this point. Wait for the reception of a start streaming request. ABORT EVENT not necessary. */
			if ((CyU3PEventGet (&glFxUVCEvent, (CY_FX_FRONT_CAM_SEL_EVENT | CY_FX_REAR_CAM_SEL_EVENT | CY_FX_LP_MODE_EVENT
					| CY_FX_UVC_STREAM_EVENT), CYU3P_EVENT_OR, &flag, CYU3P_WAIT_FOREVER) == CY_U3P_SUCCESS) || GpioSwitch)
			{

				if (CyU3PEventGet (&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_AND, &flag, CYU3P_NO_WAIT) == CY_U3P_SUCCESS)
				{

					if (usbSpeed == CY_U3P_SUPER_SPEED)
					{
						CyU3PUsbSetLinkPowerState (CyU3PUsbLPM_U0);
						CyU3PBusyWait (200);
					}

					/* If not initialized, initialize gpif */
					if (gpif_initialized == CyFalse)
					{
						CyFxUvcAppGpifInit ();
						gpif_initialized = CyTrue;
					}

					CyU3PThreadSleep (10);	/* Wait 10 ms */

					/* Power-up the selected camera */
					if (EN_FRONT_CAM && (!EN_REAR_CAM))
					{
						SensorPwdn_F(CyFalse);		/* Power-up the front camera */
					}
					else if (EN_REAR_CAM /*&& (!EN_FRONT_CAM)*/)
					{
						SensorPwdn_R(CyFalse);		/* Power-up the rear camera */
					}
					else
					{
						myCyU3PDebugPrint (4, "Power-down mode!\r\n");
					}

					/* Selection of the video format based on glCommitCtrl byte 2 and 3. */
					if (usbSpeed == CY_U3P_SUPER_SPEED)
					{
						switch((glCommitCtrl[2] << 4) | glCommitCtrl[3])
						{
							case HD_1080p:
								/* Resolution: 1920x1080, Frame rate: 10 fps */
								apiRetStatus = SensorScaling_1080p_30fps();
								if (apiRetStatus != CY_U3P_SUCCESS)
								{
									myCyU3PDebugPrint (4, "Error in sensor video streaming configuration! \r\n");
								}
								break;
							case VGA_640x480:
								/* Resolution: 640x480, Frame rate: 30 fps */
								apiRetStatus = SensorScaling_VGA_30fps();
								if (apiRetStatus != CY_U3P_SUCCESS)
								{
									myCyU3PDebugPrint (4, "Error in sensor video streaming configuration! \r\n");
								}
								break;
							case _720p:
								/* Resolution: 1280x720, Frame rate: 25 fps */
								apiRetStatus = SensorScaling_720p_30fps();
								if (apiRetStatus != CY_U3P_SUCCESS)
								{
									myCyU3PDebugPrint (4, "Error in sensor video streaming configuration! \r\n");
								}
								break;

#ifdef STILL_IMAGE_CAPTURE_M2
							case _5Mp:
								/* Resolution: 2592x1944, Frame rate: 7.5 fps */
								apiRetStatus = SensorScaling_5M();
								if (apiRetStatus != CY_U3P_SUCCESS)
								{
									myCyU3PDebugPrint (4, "Error in sensor video streaming configuration! \r\n");
								}
								break;
#endif

							default:
								break;
						}
					}
					else
					{
						if (glCommitCtrl[2] == 0x01 && glCommitCtrl[3] == 0x01)
						{
							/* Resolution: 640x480, Frame rate: 30 fps */
							apiRetStatus = SensorScaling_VGA_30fps();
							if (apiRetStatus != CY_U3P_SUCCESS)
							{
								myCyU3PDebugPrint (4, "Error in sensor video streaming configuration! \r\n");
							}
						}
					}

					/* Mirror/Flip actions */
					if(EN_FRONT_CAM && (!EN_REAR_CAM))
					{
						/* For front camera Flip and Mirror action applied */
						Mirror_Flip(CyTrue, CyTrue);
					}
					else if(EN_REAR_CAM)
					{
						/* For rear camera Flip action applied */
						Mirror_Flip(CyFalse, CyTrue);
					}
					else
					{
					}

					/* AEC/AGC automatic mode */
					apiRetStatus = AEC_AGC_Auto(CyTrue);

#ifdef AUTOFOCUS
					/* Now Auto Focus firmware can be downloaded inside the MCU via I2C */
					apiRetStatus = AF_init();
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint(4, "Error: Writing AF firmware failed!\r\n");
					}

					CyU3PThreadSleep (5);	/* Wait 5 ms */

					/* Single Focus */
					apiRetStatus = AF_single_focus();
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint(4, "Error: Single Focus failed!\r\n");
					}
					else
					{
						hold_focus = CyTrue;		// hold_focus indicate that the focus is active
					}
#endif

#ifdef STILL_IMAGE_CAPTURE_M2
					gotPartial = CyFalse;				/* Reset the state of gotPartial */
#endif

					hitFV = CyFalse;
					isReadyToStream = CyTrue;			/* Camera is configured, active and ready to stream data */

					/* Reset the DMA channel. */
					apiRetStatus = CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint (4, "DMA Channel Reset Failed \n");
					}

					/* Set DMA Channel transfer size, first producer socket */
					apiRetStatus = CyU3PDmaMultiChannelSetXfer (&glChHandleUVCStream, 0, 0);
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint (4, "DMA Channel Set Transfer Failed \r\n");
					}

					// Jump to the start state of the GPIF state machine. 257 is used as an
					// arbitrary invalid state (> 255) number.
					CyU3PGpifSMSwitch (257, 0, 257, 0, 2);

#ifdef FRAME_TIMER
				/* Restart the frame timeout timer */
                UvcAbort = CyFalse;
                UvcTimerStarted = CyFalse;
                CyU3PTimerModify (&UvcTimer, TIMER_PERIOD, 0);
#endif
				}
			}
		}

        /* Allow other ready threads to run before proceeding. */
        CyU3PThreadRelinquish ();
    }
}



/*
 * Handler for control requests addressed to the Processing Unit.
 */
static void UVCHandleProcessingUnitRqts(void)
{

#ifdef BRIGHTNESS_CONTROL
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    uint16_t readCount;
#endif

    switch (wValue)
    {

#ifdef BRIGHTNESS_CONTROL
        case CY_FX_UVC_PU_BRIGHTNESS_CONTROL:		/*Brightness control on both front and rear camera*/
			switch (bRequest)
			{
				case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of brightness data = 1 byte. */
					glEp0Buffer[0] = 1;
					CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					break;
				case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
					glEp0Buffer[0] = SensorGetBrightness ();
					CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					break;
				case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum brightness level = 0 (level -4 --> see sensor5640.h) */
					glEp0Buffer[0] = 0;
					CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					break;
				case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum brightness level = 8 (level +4 --> see sensor5640.h) */
					glEp0Buffer[0] = 8;
					CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					break;
				case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
					glEp0Buffer[0] = 1;
					CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					break;
				case CY_FX_USB_UVC_GET_INFO_REQ: /* Both GET and SET requests are supported, auto modes not supported */
					glEp0Buffer[0] = 3;
					CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					break;
				case CY_FX_USB_UVC_GET_DEF_REQ: /* Default brightness level = 4 (level 0 --> see sensor5640.h) */
					glEp0Buffer[0] = 4;
					CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					break;
				case CY_FX_USB_UVC_SET_CUR_REQ: /* Update brightness level. */
					apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
							glEp0Buffer, &readCount);
					if (apiRetStatus == CY_U3P_SUCCESS)
					{
						apiRetStatus = SensorSetBrightness (glEp0Buffer[0]);
						if (apiRetStatus != CY_U3P_SUCCESS)
						{
							myCyU3PDebugPrint (4, "Error in Brightness configuration! \r\n");
						}
					}
					break;
				default:
					CyU3PUsbStall (0, CyTrue, CyFalse);
					break;
			}
        	break;
#endif
        default:
            /*
             * Only the brightness control is supported as of now. Add additional code here to support
             * other controls.
             */
            CyU3PUsbStall (0, CyTrue, CyFalse);
            break;
    }
}


/*
 * Handler for control requests addressed to the UVC Camera Terminal unit.
 */
static void UVCHandleCameraTerminalRqts(void)
{
#ifdef UVC_PTZ_SUPPORT
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    uint16_t readCount;
    uint16_t zoomVal;
    int32_t  panVal, tiltVal;
    CyBool_t sendData = CyFalse;
#endif

    switch (wValue)
    {
#ifdef UVC_PTZ_SUPPORT
        case CY_FX_UVC_CT_ZOOM_ABSOLUTE_CONTROL:
            switch (bRequest)
            {
                case CY_FX_USB_UVC_GET_INFO_REQ:
                    glEp0Buffer[0] = 3;                /* Support GET/SET queries. */
                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_CUR_REQ: /* Current zoom control value. */
                    zoomVal  = CyFxUvcAppGetCurrentZoom ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum zoom control value. */
                    zoomVal  = CyFxUvcAppGetMinimumZoom ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum zoom control value. */
                    zoomVal  = CyFxUvcAppGetMaximumZoom ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution is one unit. */
                    zoomVal  = CyFxUvcAppGetZoomResolution ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_DEF_REQ: /* Default zoom setting. */
                    zoomVal  = CyFxUvcAppGetDefaultZoom ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_SET_CUR_REQ:
                    apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                            glEp0Buffer, &readCount);
                    if (apiRetStatus == CY_U3P_SUCCESS)
                    {
                        zoomVal = (glEp0Buffer[0]) | (glEp0Buffer[1] << 8);
                        CyFxUvcAppModifyZoom (zoomVal);
                    }
                    break;
                default:
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                    break;
            }

            if (sendData)
            {
                /* Send the 2-byte data in zoomVal back to the USB host. */
                glEp0Buffer[0] = CY_U3P_GET_LSB (zoomVal);
                glEp0Buffer[1] = CY_U3P_GET_MSB (zoomVal);
                CyU3PUsbSendEP0Data (wLength, (uint8_t *)glEp0Buffer);
            }
            break;

        case CY_FX_UVC_CT_PANTILT_ABSOLUTE_CONTROL:
            switch (bRequest)
            {
                case CY_FX_USB_UVC_GET_INFO_REQ:
                    glEp0Buffer[0] = 3;                /* GET/SET requests supported for this control */
                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_CUR_REQ:
                    panVal   = CyFxUvcAppGetCurrentPan ();
                    tiltVal  = CyFxUvcAppGetCurrentTilt ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_MIN_REQ:
                    panVal   = CyFxUvcAppGetMinimumPan ();
                    tiltVal  = CyFxUvcAppGetMinimumTilt ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_MAX_REQ:
                    panVal   = CyFxUvcAppGetMaximumPan ();
                    tiltVal  = CyFxUvcAppGetMaximumTilt ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_RES_REQ:
                    panVal   = CyFxUvcAppGetPanResolution ();
                    tiltVal  = CyFxUvcAppGetTiltResolution ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_GET_DEF_REQ:
                    panVal   = CyFxUvcAppGetDefaultPan ();
                    tiltVal  = CyFxUvcAppGetDefaultTilt ();
                    sendData = CyTrue;
                    break;
                case CY_FX_USB_UVC_SET_CUR_REQ:
                    apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                            glEp0Buffer, &readCount);
                    if (apiRetStatus == CY_U3P_SUCCESS)
                    {
                        panVal = (glEp0Buffer[0]) | (glEp0Buffer[1]<<8) |
                            (glEp0Buffer[2]<<16) | (glEp0Buffer[2]<<24);
                        tiltVal = (glEp0Buffer[4]) | (glEp0Buffer[5]<<8) |
                            (glEp0Buffer[6]<<16) | (glEp0Buffer[7]<<24);

                        CyFxUvcAppModifyPan (panVal);
                        CyFxUvcAppModifyTilt (tiltVal);
                    }
                    break;
                default:
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                    break;
            }

            if (sendData)
            {
                /* Send the 8-byte PAN and TILT values back to the USB host. */
                glEp0Buffer[0] = CY_U3P_DWORD_GET_BYTE0 (panVal);
                glEp0Buffer[1] = CY_U3P_DWORD_GET_BYTE1 (panVal);
                glEp0Buffer[2] = CY_U3P_DWORD_GET_BYTE2 (panVal);
                glEp0Buffer[3] = CY_U3P_DWORD_GET_BYTE3 (panVal);
                glEp0Buffer[4] = CY_U3P_DWORD_GET_BYTE0 (tiltVal);
                glEp0Buffer[5] = CY_U3P_DWORD_GET_BYTE1 (tiltVal);
                glEp0Buffer[6] = CY_U3P_DWORD_GET_BYTE2 (tiltVal);
                glEp0Buffer[7] = CY_U3P_DWORD_GET_BYTE3 (tiltVal);
                CyU3PUsbSendEP0Data (wLength, (uint8_t *)glEp0Buffer);
            }
            break;
#endif

        default:
            CyU3PUsbStall (0, CyTrue, CyFalse);
            break;
    }
}


/*
 * Handler for UVC Interface control requests.
 */
static void UVCHandleInterfaceCtrlRqts(void)
{
    /* No requests supported as of now. Just stall EP0 to fail the request. */
    CyU3PUsbStall (0, CyTrue, CyFalse);
}


/*
 * Handler for control requests addressed to the Extension Unit.
 */
static void UVCHandleExtensionUnitRqts(void)
{
    /* No requests supported as of now. Just stall EP0 to fail the request. */
    CyU3PUsbStall (0, CyTrue, CyFalse);
}


/*
 * Handler for the video streaming control requests.
 */
static void UVCHandleVideoStreamingRqts(void)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    uint16_t readCount;

    switch (wValue)
    {

#ifdef STILL_IMAGE_CAPTURE_M2
    	case CY_FX_UVC_STILL_PROBE_CONTROL:
    		switch (bRequest)
    		{
				case CY_FX_USB_UVC_GET_INFO_REQ:
					glEp0Buffer[0] = 3;                /* GET/SET requests are supported: D0 and D1 each set to one, and the remaining bits set to zero */
					CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					break;
				case CY_FX_USB_UVC_GET_LEN_REQ:
					glEp0Buffer[0] = CY_FX_UVC_MAX_STILL_PROBE_SETTING;
					CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					break;
				case CY_FX_USB_UVC_GET_CUR_REQ:
				case CY_FX_USB_UVC_GET_MIN_REQ:
				case CY_FX_USB_UVC_GET_MAX_REQ:
				case CY_FX_USB_UVC_GET_DEF_REQ:
					if (usbSpeed == CY_U3P_SUPER_SPEED)		/* Still Image Capture only for SuperSpeed. */
					{
						apiRetStatus = CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_STILL_PROBE_SETTING, glStillProbeCtrl);
						if (apiRetStatus != CY_U3P_SUCCESS)
						{
							myCyU3PDebugPrint (4, "GET_DEF Still Probe/Commit error\n");
						}
					}
					else
					{
						CyU3PUsbStall (0, CyTrue, CyFalse);
					}
					break;
				case CY_FX_USB_UVC_SET_CUR_REQ:
					/* Get the UVC STILL probe/commit control data from EP0 */
					apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_STILL_PROBE_SETTING_ALIGNED, glStillCommitCtrl, &readCount);
					if (apiRetStatus == CY_U3P_SUCCESS)
					{
						if (usbSpeed == CY_U3P_SUPER_SPEED)
						{
							/* Copy the relevant settings from the host provided data into the
						   	   active data structure. */
							glStillProbeCtrl[0] = glStillCommitCtrl[0];
							glStillProbeCtrl[1] = glStillCommitCtrl[1];
							glStillProbeCtrl[2] = glStillCommitCtrl[2];
							glStillProbeCtrl[3] = glStillCommitCtrl[3];
							glStillProbeCtrl[4] = glStillCommitCtrl[4];
							glStillProbeCtrl[5] = glStillCommitCtrl[5];
							glStillProbeCtrl[6] = glStillCommitCtrl[6];
						}
					}
					else
					{
						myCyU3PDebugPrint (4, "GetEP0Data (Still) Error\n");
					}
					break;
				default:
				   CyU3PUsbStall (0, CyTrue, CyFalse);
				   break;
			}
			break;

    	case CY_FX_UVC_STILL_COMMIT_CONTROL:
		   switch (bRequest)
		   {
				case CY_FX_USB_UVC_GET_INFO_REQ:
					glEp0Buffer[0] = 3;                /* GET/SET requests are supported: D0 and D1 each set to one, and the remaining bits set to zero */
					CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					break;
				case CY_FX_USB_UVC_GET_LEN_REQ:
					glEp0Buffer[0] = CY_FX_UVC_MAX_STILL_PROBE_SETTING;
					CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					break;
				case CY_FX_USB_UVC_GET_CUR_REQ:
				case CY_FX_USB_UVC_GET_MIN_REQ:
				case CY_FX_USB_UVC_GET_MAX_REQ:
				case CY_FX_USB_UVC_GET_DEF_REQ: 	/* There is only one setting per USB speed. */
					apiRetStatus = CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_STILL_PROBE_SETTING, glStillProbeCtrl);
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint (4, "GET_CUR Still Probe/Commit Error\n");
					}
					break;
				case CY_FX_USB_UVC_SET_CUR_REQ:
					/* Get the UVC STILL probe/commit control data from EP0 */
					apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_STILL_PROBE_SETTING_ALIGNED, glStillCommitCtrl, &readCount);
					if (apiRetStatus != CY_U3P_SUCCESS)
					{
						myCyU3PDebugPrint (4, "GetEP0Data (Still) Error\n");
					}
					else
					{
						glCurrentStillFrameIndex = glStillCommitCtrl[1];		/* Update Still Image Size Patterns selected. */
						myCyU3PDebugPrint(4, "STILL Control Error\n");
					}
					break;
				default:
				   CyU3PUsbStall (0, CyTrue, CyFalse);
				   break;
			}
			break;

	   case CY_FX_UVC_STILL_IMAGE_TRIGGER:
		   switch(bRequest)
		   {
				case CY_FX_USB_UVC_SET_CUR_REQ:
					apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_STILL_IMAGE_TRIGGER_ALIGNED, &glStillTriggerCtrl, &readCount);
					if (apiRetStatus == CY_U3P_SUCCESS)
					{
						if(glStillTriggerCtrl == CY_FX_UVC_STILL_IMAGE)
						{
							/* Now we have to start capture a still image using the glCurrentStillFrameIndex
							 * Still Image Size Patterns selected (Still Image Frame descriptor). */
							glCaptureFrame = 0;
							glStillCaptureFinish = CyFalse;
							glDiscardBuffer = CyFalse;
							glStillCaptureStart = CyTrue;
						}
						else
							myCyU3PDebugPrint(4, "STILL_TRIGGER SET_CUR Error\n");
					}
					else
					{
						myCyU3PDebugPrint (4, "SET_CUR (Still Trigger) Error\n");
					}
					break;
				default:
				   CyU3PUsbStall (0, CyTrue, CyFalse);
				   break;
			}
			break;
#endif

        case CY_FX_UVC_PROBE_CTRL:
            switch (bRequest)
            {
                case CY_FX_USB_UVC_GET_INFO_REQ:
                    glEp0Buffer[0] = 3;                /* GET/SET requests are supported. */
                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_LEN_REQ:
                    glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_CUR_REQ:
                case CY_FX_USB_UVC_GET_MIN_REQ:
                case CY_FX_USB_UVC_GET_MAX_REQ:
                case CY_FX_USB_UVC_GET_DEF_REQ: /* There is only one setting per USB speed. */
                    if (usbSpeed == CY_U3P_SUPER_SPEED)
                    {
                        CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl);
                    }
                    else
                    {
                        CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl20);
                    }
                    break;
                case CY_FX_USB_UVC_SET_CUR_REQ:
                    apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                            glCommitCtrl, &readCount);
                    if (apiRetStatus == CY_U3P_SUCCESS)
                    {
                        if (usbSpeed == CY_U3P_SUPER_SPEED)
                        {
                            /* Copy the relevant settings from the host provided data into the
                               active data structure. */
                            glProbeCtrl[2] = glCommitCtrl[2];
                            glProbeCtrl[3] = glCommitCtrl[3];
                            glProbeCtrl[4] = glCommitCtrl[4];
                            glProbeCtrl[5] = glCommitCtrl[5];
                            glProbeCtrl[6] = glCommitCtrl[6];
                            glProbeCtrl[7] = glCommitCtrl[7];
                        }
                    }
                    break;
                default:
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                    break;
            }
            break;

        case CY_FX_UVC_COMMIT_CTRL:
            switch (bRequest)
            {
                case CY_FX_USB_UVC_GET_INFO_REQ:
                    glEp0Buffer[0] = 3;                        /* GET/SET requests are supported. */
                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_LEN_REQ:
                    glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
                    break;
                case CY_FX_USB_UVC_GET_CUR_REQ:
                    if (usbSpeed == CY_U3P_SUPER_SPEED)
                    {
                        CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl);
                    }
                    else
                    {
                        CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl20);
                    }
                    break;
                case CY_FX_USB_UVC_SET_CUR_REQ:
                    /* The host has selected the parameters for the video stream. Check the desired
                    resolution settings, configure the sensor and start the video stream.*/
                    apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED, glCommitCtrl, &readCount);
                    if (apiRetStatus == CY_U3P_SUCCESS)
                    {

						/* Set the STREAM EVENT */
						apiRetStatus = CyU3PEventSet (&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_OR);
						if (apiRetStatus != CY_U3P_SUCCESS)
						{
							myCyU3PDebugPrint (4, "Set CY_FX_UVC_STREAM_EVENT failed \n");
						}

                    }
                    break;

                default:
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                    break;
            }
            break;

        default:
            CyU3PUsbStall (0, CyTrue, CyFalse);
            break;
    }
}


/*
 * Entry function for the UVC control request processing thread.
 */
void UVCAppEP0Thread_Entry(uint32_t input)
{
    uint32_t eventMask = (CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT | CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT);
    uint32_t eventFlag;

    INUTILE(input);

#ifdef USB_DEBUG_INTERFACE
    CyU3PReturnStatus_t apiRetStatus;
    CyU3PDmaBuffer_t    dmaInfo;

    eventMask |= CY_FX_USB_DEBUG_CMD_EVENT;
#endif

    for (;;)
    {
        /* Wait for a Video control or streaming related request on the control endpoint. */
        if (CyU3PEventGet (&glFxUVCEvent, eventMask, CYU3P_EVENT_OR_CLEAR, &eventFlag,
        		CYU3P_NO_WAIT) == CY_U3P_SUCCESS)
        {
            /* If this is the first request received during this connection, query the connection speed. */
            if (!isUsbConnected)
            {
                usbSpeed = CyU3PUsbGetSpeed ();
                if (usbSpeed != CY_U3P_NOT_CONNECTED)
                {
                    isUsbConnected = CyTrue;
                }
            }

            if (eventFlag & CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT)
            {
                switch ((wIndex >> 8))
                {
                    case CY_FX_UVC_PROCESSING_UNIT_ID:
                        UVCHandleProcessingUnitRqts ();
                        break;

                    case CY_FX_UVC_CAMERA_TERMINAL_ID:
                        UVCHandleCameraTerminalRqts ();
                        break;

                    case CY_FX_UVC_INTERFACE_CTRL:
                        UVCHandleInterfaceCtrlRqts ();
                        break;

                    case CY_FX_UVC_EXTENSION_UNIT_ID:
                        UVCHandleExtensionUnitRqts ();
                        break;

                    default:
                        /* Unsupported request. Fail by stalling the control endpoint. */
                        CyU3PUsbStall (0, CyTrue, CyFalse);
                        break;
                }
            }

            if (eventFlag & CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT)
            {
                if (wIndex != CY_FX_UVC_STREAM_INTERFACE)
                {
                    CyU3PUsbStall (0, CyTrue, CyFalse);
                }
                else
                {
                    UVCHandleVideoStreamingRqts ();
                }
            }

            else
            {

            }

#ifdef USB_DEBUG_INTERFACE
            if (eventFlag & CY_FX_USB_DEBUG_CMD_EVENT)
            {
                /* Get the command buffer */
                apiRetStatus = CyU3PDmaChannelGetBuffer (&glDebugCmdChannel, &dmaInfo, CYU3P_WAIT_FOREVER);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    myCyU3PDebugPrint (4, "Failed to receive debug command \r\n");
                    CyFxAppErrorHandler (apiRetStatus);
                }

                /* Decode the command from the command buffer, error checking is not implemented,
                 * so the command is expected to be correctly sent from the host application. First byte indicates
                 * read (0x00) or write (0x01) command. Second and third bytes are register address high byte and
                 * register address low byte. For read commands the fourth byte (optional) can be N>0, to read N
                 * registers in sequence. Response first byte is status (0=Pass, !0=Fail) followed by N pairs of
                 * register value high byte and register value low byte.
                 */
                if (dmaInfo.buffer[0] == 0)
                {
                    if (dmaInfo.count == 3)
                    {
                        glDebugRspBuffer[0] = SensorRead2B (SENSOR_ADDR_RD, dmaInfo.buffer[1], dmaInfo.buffer[2],
                        		(glDebugRspBuffer+1));
                        dmaInfo.count = 3;
                    }
                    else if (dmaInfo.count == 4)
                    {
                        if (dmaInfo.buffer[3] > 0)
                        {
                                glDebugRspBuffer[0] = SensorRead (SENSOR_ADDR_RD, dmaInfo.buffer[1], dmaInfo.buffer[2],
                                		(dmaInfo.buffer[3]*2), (glDebugRspBuffer+1));
                        }
                        dmaInfo.count = dmaInfo.buffer[3]*2+1;
                    }
                }
                /*  For write commands, the register address is followed by N pairs (N>0) of register value high byte
                 *  and register value low byte to write in sequence. Response first byte is status (0=Pass, !0=Fail)
                 *  followed by N pairs of register value high byte and register value low byte after modification.
                 */
                else if (dmaInfo.buffer[0] == 1)
                {
                        glDebugRspBuffer[0] = SensorWrite (SENSOR_ADDR_WR, dmaInfo.buffer[1], dmaInfo.buffer[2],
                        		(dmaInfo.count-3), (dmaInfo.buffer+3));
                        if (glDebugRspBuffer[0] != CY_U3P_SUCCESS)
                        	break;
                        glDebugRspBuffer[0] = SensorRead (SENSOR_ADDR_RD, dmaInfo.buffer[1], dmaInfo.buffer[2],
                        		(dmaInfo.count-3), (glDebugRspBuffer+1));
                        if (glDebugRspBuffer[0] != CY_U3P_SUCCESS)
                        	break;
                    dmaInfo.count -= 2;
                }
                /* Default case, prepare buffer for loop back command in response */
                else
                {
                   /* For now, we just copy the command into the response buffer; and send it back to the
                      USB host. This can be expanded to include I2C transfers. */
                    CyU3PMemCopy (glDebugRspBuffer, dmaInfo.buffer, dmaInfo.count);
                }

                dmaInfo.buffer = glDebugRspBuffer;
                dmaInfo.size   = 1024;
                dmaInfo.status = 0;

                /* Free the command buffer to receive the next command. */
                apiRetStatus = CyU3PDmaChannelDiscardBuffer (&glDebugCmdChannel);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    myCyU3PDebugPrint (4, "Failed to free up command OUT EP buffer \r\n");
                    CyFxAppErrorHandler (apiRetStatus);
                }

                /* Wait until the response has gone out. */
                CyU3PDmaChannelWaitForCompletion (&glDebugRspChannel, CYU3P_WAIT_FOREVER);

                apiRetStatus = CyU3PDmaChannelSetupSendBuffer (&glDebugRspChannel, &dmaInfo);
                if (apiRetStatus != CY_U3P_SUCCESS)
                {
                    myCyU3PDebugPrint (4, "Failed to send debug response \r\n");
                    CyFxAppErrorHandler (apiRetStatus);
                }
            }
#endif
        }

        /* Allow other ready threads to run. */
        CyU3PThreadRelinquish ();
    }
}


/*
 * This function is called by the FX3 framework once the ThreadX RTOS has started up.
 * The application specific threads and other OS resources are created and initialized here.
 */
void CyFxApplicationDefine(void)
{
    void *ptr1, *ptr2;
    uint32_t retThrdCreate;

    /* Allocate the memory for the thread stacks. */
    ptr1 = CyU3PMemAlloc (UVC_APP_THREAD_STACK);
    ptr2 = CyU3PMemAlloc (UVC_APP_THREAD_STACK);
    if ((ptr1 == 0) || (ptr2 == 0))
        goto fatalErrorHandler;

    /* Create the UVC application thread. */
    retThrdCreate = CyU3PThreadCreate (&uvcAppThread,   /* UVC Thread structure */
            "30:UVC App Thread",                        /* Thread Id and name */
            UVCAppThread_Entry,                         /* UVC Application Thread Entry function */
            0,                                          /* No input parameter to thread */
            ptr1,                                       /* Pointer to the allocated thread stack */
            UVC_APP_THREAD_STACK,                       /* UVC Application Thread stack size */
            UVC_APP_THREAD_PRIORITY,                    /* UVC Application Thread priority */
            UVC_APP_THREAD_PRIORITY,                    /* Threshold value for thread pre-emption. */
            CYU3P_NO_TIME_SLICE,                        /* No time slice for the application thread */
            CYU3P_AUTO_START                            /* Start the Thread immediately */
            );
    if (retThrdCreate != 0)
    {
        goto fatalErrorHandler;
    }

    /* Create the control request handling thread. */
    retThrdCreate = CyU3PThreadCreate (&uvcAppEP0Thread,        /* UVC Thread structure */
            "31:UVC App EP0 Thread",                            /* Thread Id and name */
            UVCAppEP0Thread_Entry,                              /* UVC Application EP0 Thread Entry function */
            0,                                                  /* No input parameter to thread */
            ptr2,                                               /* Pointer to the allocated thread stack */
            UVC_APP_EP0_THREAD_STACK,                           /* UVC Application Thread stack size */
            UVC_APP_EP0_THREAD_PRIORITY,                        /* UVC Application Thread priority */
            UVC_APP_EP0_THREAD_PRIORITY,                        /* Threshold value for thread pre-emption. */
            CYU3P_NO_TIME_SLICE,                                /* No time slice for the application thread */
            CYU3P_AUTO_START                                    /* Start the Thread immediately */
            );
    if (retThrdCreate != 0)
    {
        goto fatalErrorHandler;
    }

    return;

fatalErrorHandler:
    /* Add custom recovery or debug actions here */
    /* Loop indefinitely */
    while (1);
}


/* Main entry point for the C code. We perform device initialization and start
 * the ThreadX RTOS here.
 */
int main(void)
{
    CyU3PReturnStatus_t apiRetStatus;
    CyU3PIoMatrixConfig_t io_cfg;

    /* Initialize the device */
    apiRetStatus = CyU3PDeviceInit (0);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Turn on instruction cache to improve firmware performance. Use Release build to improve it further.
     * D-cache not enabled. */
    apiRetStatus = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);

    /* Configure the IO matrix for the device. */
    io_cfg.isDQ32Bit        = CyFalse;		/* GPIF is 16 bit wide. */
    io_cfg.lppMode          = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = 0;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    io_cfg.useUart          = CyFalse;  	/* Uart is disabled. */
    io_cfg.useI2C           = CyTrue;   	/* I2C is used for camera interface. */
    io_cfg.useI2S           = CyFalse;
    io_cfg.useSpi           = CyFalse;

    apiRetStatus = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();

    /* Dummy return to make the compiler happy */
    return 0;

handle_fatal_error:
    /* Cannot recover from this error. */
    while (1);
}

