/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_usbx_device.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_usbx_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_otg.h"
#include "ux_api.h"
#include "ux_system.h"
#include "ux_utility.h"
#include "ux_device_stack.h"
#include "ux_dcd_stm32.h"
#include "ux_device_audio.h"
#include "ux_device_class_audio.h"
#include "ux_device_class_audio20.h"
#include "ux_device_descriptors.h"
#include "stm32f4_discovery_audio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USBX_APP_STACK_SIZE             2048
#define USBX_MEMORY_SIZE                (40 * 1024)
#define FRAME_BUFFER_NUNMBER            2
#define AUDIO_OUT_INSTANCE              1

#define VOLUME_SPEAKER_RES              1
#define VOLUME_SPEAKER_MAX              100
#define VOLUME_SPEAKER_MIN              0
#define VOLUME_SPEAKER_DEFAULT          70
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
AUDIO_ProcessTypdef haudio;
UX_DEVICE_CLASS_AUDIO                  *audio;
UX_DEVICE_CLASS_AUDIO_STREAM           *stream_read;
UX_DEVICE_CLASS_AUDIO_PARAMETER         audio_parameter;
UX_DEVICE_CLASS_AUDIO_STREAM_PARAMETER  audio_stream_parameter[1];

UX_DEVICE_CLASS_AUDIO20_CONTROL         audio_control[1];
UX_DEVICE_CLASS_AUDIO20_CONTROL_GROUP   group;

TX_THREAD       ux_app_thread;
TX_THREAD       ux_audio_play_thread;
TX_QUEUE        ux_app_MsgQueue;

ULONG device_framework_fs_length;
ULONG string_framework_length;
ULONG languge_id_framework_length;
UCHAR *device_framework_full_speed;
UCHAR *string_framework;
UCHAR *language_id_framework;
CHAR  *pointer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID  usbx_app_thread_entry(ULONG arg);
/* USER CODE END PFP */
/**
  * @brief  Application USBX Device Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_USBX_Device_Init(VOID *memory_ptr)
{
  UINT ret = UX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN MX_USBX_Device_MEM_POOL */

  /* USER CODE END MX_USBX_Device_MEM_POOL */

  /* USER CODE BEGIN MX_USBX_Device_Init */

  /* Allocate the USBX_MEMORY_SIZE. */
  tx_byte_allocate(byte_pool, (VOID **) &pointer,
                   USBX_MEMORY_SIZE, TX_NO_WAIT);

  /* Initialize USBX Memory */
  ret = ux_system_initialize(pointer, USBX_MEMORY_SIZE, UX_NULL, 0);

  if(ret != UX_SUCCESS) {
	  Error_Handler();
  }

  /* Get_Device_Framework_Full_Speed and get the length */
  device_framework_full_speed = USBD_Get_Device_Framework_Speed(USBD_FULL_SPEED,
                                                                &device_framework_fs_length);

  /* Get_String_Framework and get the length */
  string_framework = USBD_Get_String_Framework(&string_framework_length);

  /* Get_Language_Id_Framework and get the length */
  language_id_framework = USBD_Get_Language_Id_Framework(&languge_id_framework_length);



  /* Set the parameters for Audio streams.  */
  audio_stream_parameter[0].ux_device_class_audio_stream_parameter_callbacks.ux_device_class_audio_stream_change     = Audio_ReadChange;
  audio_stream_parameter[0].ux_device_class_audio_stream_parameter_callbacks.ux_device_class_audio_stream_frame_done = Audio_ReadDone;
  audio_stream_parameter[0].ux_device_class_audio_stream_parameter_thread_stack_size = UX_THREAD_STACK_SIZE;
  audio_stream_parameter[0].ux_device_class_audio_stream_parameter_max_frame_buffer_nb   = FRAME_BUFFER_NUNMBER;
//  audio_stream_parameter[0].ux_device_class_audio_stream_parameter_max_frame_buffer_size = USBD_AUDIO_EPIN_FS_MPS;
  audio_stream_parameter[0].ux_device_class_audio_stream_parameter_max_frame_buffer_size = 196;
  audio_stream_parameter[0].ux_device_class_audio_stream_parameter_thread_entry = ux_device_class_audio_read_thread_entry;

  /* Set the parameters for Audio device.  */
  audio_parameter.ux_device_class_audio_parameter_streams_nb  = 1;
  audio_parameter.ux_device_class_audio_parameter_streams     = audio_stream_parameter;
  audio_parameter.ux_device_class_audio_parameter_callbacks.ux_slave_class_audio_instance_activate   = Audio_Init;
  audio_parameter.ux_device_class_audio_parameter_callbacks.ux_slave_class_audio_instance_deactivate = Audio_DeInit;
  audio_parameter.ux_device_class_audio_parameter_callbacks.ux_device_class_audio_control_process    = Audio_Control;

  //  /* Initialize audio 2.0 control values.  */
	audio_control[0].ux_device_class_audio20_control_cs_id                = USB_AUDIO_CONFIG_PLAY_CLOCK_SOURCE_ID;
	audio_control[0].ux_device_class_audio20_control_sampling_frequency   = USB_AUDIO_CONFIG_PLAY_SAMPLING_FREQUENCY;
	audio_control[0].ux_device_class_audio20_control_fu_id                = USB_AUDIO_CONFIG_PLAY_UNIT_FEATURE_ID;
	audio_control[0].ux_device_class_audio20_control_mute[0]              = 0;
	audio_control[0].ux_device_class_audio20_control_volume_min[0]        = VOLUME_SPEAKER_MIN;
	audio_control[0].ux_device_class_audio20_control_volume_max[0]        = VOLUME_SPEAKER_MAX;
	audio_control[0].ux_device_class_audio20_control_volume_res[0]        = VOLUME_SPEAKER_RES;
	audio_control[0].ux_device_class_audio20_control_volume[0]            = VOLUME_SPEAKER_DEFAULT;

	group.ux_device_class_audio20_control_group_controls_nb = 1;
	group.ux_device_class_audio20_control_group_controls    = audio_control;



  /* Allocate the stack for main_usbx_app_thread_entry.  */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, USBX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the main thread.  */
  if (tx_thread_create(&ux_app_thread, "main_usbx_app_thread_entry",
                       usbx_app_thread_entry, 0, pointer, USBX_APP_STACK_SIZE,
                       20, 20, 1, TX_AUTO_START) != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Allocate the stack for main_usbx_app_thread_entry.  */
  if(tx_byte_allocate(byte_pool, (VOID **) &pointer, USBX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the main thread.  */
  if(tx_thread_create(&ux_audio_play_thread, "usbx_audio_app_thread_entry",
                      usbx_audio_play_app_thread, 0, pointer, USBX_APP_STACK_SIZE,
                      20, 20, 1, TX_AUTO_START) != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Allocate Memory for the Queue */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
                       APP_QUEUE_SIZE * sizeof(ULONG), TX_NO_WAIT) != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create the MsgQueue */
  if (tx_queue_create(&ux_app_MsgQueue, "Message Queue app", TX_1_ULONG,
                      pointer, APP_QUEUE_SIZE * sizeof(ULONG)) != TX_SUCCESS)
  {
    Error_Handler();
  }
 
//  AudioPlayInit.Device = AUDIO_OUT_DEVICE_HEADPHONE;
//  AudioPlayInit.ChannelsNbr = 2;
//  AudioPlayInit.SampleRate = AUDIO_FREQUENCY_48K;
//  AudioPlayInit.BitsPerSample = AUDIO_RESOLUTION_16B;
//  AudioPlayInit.Volume = 70;
//
  /* USER CODE END MX_USBX_Device_Init */

  return ret;
}

/* USER CODE BEGIN 1 */
/**
  * @brief  Function implementing usbx_app_thread_entry.
  * @param arg: Not used
  * @retval None
  */
void usbx_app_thread_entry(ULONG arg)
{


//  if( BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 70, AUDIO_FREQUENCY_48K) != AUDIO_OK)
//  {
//    Error_Handler();
//  }
	  /* The code below is required for installing the device portion of USBX.
	  In this application */
	  if (ux_device_stack_initialize(NULL,
	                                 0,
	                                 device_framework_full_speed,
	                                 device_framework_fs_length,
	                                 string_framework,
	                                 string_framework_length,
	                                 language_id_framework,
	                                 languge_id_framework_length,
	                                 UX_NULL) != UX_SUCCESS)
	  {
		  Error_Handler();
	  }

	  /* Initialize the device Audio class. This class owns interfaces starting with 0. */
	    if( ux_device_stack_class_register(_ux_system_slave_class_audio_name, ux_device_class_audio_entry,
	                                       1, 0, &audio_parameter) != UX_SUCCESS)
	    {
	      Error_Handler();
	    }

  /* Initialization of USB device */
  MX_USB_Device_Init();
}

/**
  * @brief MX_USB_Device_Init
  *        Initialization of USB device.
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_Device_Init(void)
{
	  /* USER CODE BEGIN USB_Device_Init_PreTreatment_0 */
	  /* USER CODE END USB_Device_Init_PreTreatment_0 */

	  MX_USB_OTG_FS_PCD_Init();

	  /* USER CODE BEGIN USB_Device_Init_PreTreatment_1 */
	  /* Set Rx FIFO */
//	  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x200);
//	  /* Set Tx FIFO 0 */
//	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
//	  /* Set Tx FIFO 1 */
//	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x100);
	  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0xf0);
	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x10);
	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x10);

//	  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);
//	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
//	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x80);

	  /* Set Rx FIFO */
//	  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0xf0);
//	  /* Set Tx0 FIFO (for EP0 IN) */
//	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
//	  /* Set Tx1 FIFO (for EP1 IN) */
//	  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x10);
	  /* USER CODE END USB_Device_Init_PreTreatment_1 */

	  /* initialize the device controller driver*/
	  _ux_dcd_stm32_initialize((ULONG)USB_OTG_FS, (ULONG)&hpcd_USB_OTG_FS);

	  HAL_PCD_Start(&hpcd_USB_OTG_FS);

	  /* USER CODE BEGIN USB_Device_Init_PostTreatment */
	  /* USER CODE END USB_Device_Init_PostTreatment */
}
/* USER CODE END 1 */
