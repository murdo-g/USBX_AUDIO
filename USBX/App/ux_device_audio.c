/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_audio.c
  * @author  MCD Application Team
  * @brief   USBX Device AUDIO applicative source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2022 STMicroelectronics.
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
#include "ux_device_audio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ux_api.h"
#include "ux_device_audio.h"
#include "ux_device_descriptors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define VOLUME_SPEAKER_RES              1
//#define VOLUME_SPEAKER_MAX              100
//#define VOLUME_SPEAKER_MIN              0
//#define VOLUME_SPEAKER_DEFAULT          70
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

extern UX_DEVICE_CLASS_AUDIO            *audio;
extern UX_DEVICE_CLASS_AUDIO_STREAM     *stream_read;
extern TX_QUEUE                         ux_app_MsgQueue;

extern   UX_DEVICE_CLASS_AUDIO20_CONTROL         audio_control[1];
extern   UX_DEVICE_CLASS_AUDIO20_CONTROL_GROUP   group;

/* Double BUFFER for Output Audio stream */
/* as Buffer location should be aligned to cache line size (32 bytes) */
#if defined ( __ICCARM__ )  /* !< ICCARM Compiler */
#pragma location = 0x24050000
/* Buffer location should aligned to cache line size (32 bytes) */
ALIGN_32BYTES (static AUDIO_OUT_BufferTypeDef  BufferCtl);

#endif
static int play_count = 0;
static AUDIO_OUT_BufferTypeDef  __attribute__ ((aligned (32))) BufferCtl;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

VOID Audio_Init(VOID *audio_instance)
{
  /* Save the Audio instance.  */
  audio = (UX_DEVICE_CLASS_AUDIO *)audio_instance;
  
  /* Get the streams instances.  */
  ux_device_class_audio_stream_get(audio, 0, &stream_read); 
  
              
}

VOID Audio_DeInit(VOID *audio_instance)
{
  /* Reset the Audio instance.  */
  audio = UX_NULL;

  /* Reset the Audio streams.  */
  stream_read = UX_NULL;
       
}

/**
  * @brief  Function implementing usbx_app_thread_entry.
  * @param arg: Not used
  * @retval None
  */
VOID usbx_audio_play_app_thread(ULONG arg)
{

  
  while (1)
  {
//    /* Wait for a buffer of audio to be  */
//    if (tx_queue_receive(&ux_app_MsgQueue, &BufferCtl.state, TX_WAIT_FOREVER)!= TX_SUCCESS)
//    {
//      Error_Handler();
//    }
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      tx_thread_sleep(20);

//    switch(BufferCtl.state)
//    {
//
//      case PLAY_BUFFER_OFFSET_NONE:
//        /*DMA stream from output double buffer to codec in Circular mode launch*/
//        if (BSP_AUDIO_OUT_Play((uint16_t*)&BufferCtl.buff[0],
//                               AUDIO_TOTAL_BUF_SIZE) != AUDIO_OK)
//        {
//          Error_Handler();
//        }
//
//        break;
//
//    default:
//      tx_thread_sleep(1);
//      break;
//    }
  }
}

UINT Audio_Control(UX_DEVICE_CLASS_AUDIO *audio, UX_SLAVE_TRANSFER *transfer)
{
  static uint8_t vol;
  UINT                                    status;
//  UX_DEVICE_CLASS_AUDIO20_CONTROL         audio_control[1];
//  UX_DEVICE_CLASS_AUDIO20_CONTROL_GROUP   group;

//  /* Initialize audio 2.0 control values.  */
//  audio_control[0].ux_device_class_audio20_control_cs_id                = USB_AUDIO_CONFIG_PLAY_CLOCK_SOURCE_ID;
//  audio_control[0].ux_device_class_audio20_control_sampling_frequency   = USB_AUDIO_CONFIG_PLAY_SAMPLING_FREQUENCY;
//  audio_control[0].ux_device_class_audio20_control_fu_id                = USB_AUDIO_CONFIG_PLAY_UNIT_FEATURE_ID;
//  audio_control[0].ux_device_class_audio20_control_mute[0]              = 0;
//  audio_control[0].ux_device_class_audio20_control_volume_min[0]        = VOLUME_SPEAKER_MIN;
//  audio_control[0].ux_device_class_audio20_control_volume_max[0]        = VOLUME_SPEAKER_MAX;
//  audio_control[0].ux_device_class_audio20_control_volume_res[0]        = VOLUME_SPEAKER_RES;
//  audio_control[0].ux_device_class_audio20_control_volume[0]            = VOLUME_SPEAKER_DEFAULT;
//
//  group.ux_device_class_audio20_control_group_controls_nb = 1;
//  group.ux_device_class_audio20_control_group_controls    = audio_control;


  status = ux_device_class_audio20_control_process(audio, transfer, &group);
  if (status == UX_SUCCESS)
  {

    switch(audio_control[0].ux_device_class_audio20_control_changed)
    {
      case UX_DEVICE_CLASS_AUDIO20_CONTROL_MUTE_CHANGED:

//        if(BSP_AUDIO_OUT_SetMute(0) != AUDIO_OK) {
//        	Error_Handler();
//        }

        break;

      case UX_DEVICE_CLASS_AUDIO20_CONTROL_VOLUME_CHANGED:

        vol = audio_control[0].ux_device_class_audio20_control_volume[0];

//        if(BSP_AUDIO_OUT_SetVolume(vol) != AUDIO_OK) {
//        	Error_Handler();
//        }

        break;
      default:
        break;
    }
  }
  return(status);

  return(UX_SUCCESS);
}

VOID Audio_ReadChange(UX_DEVICE_CLASS_AUDIO_STREAM *stream, ULONG alternate_setting)
{
  /* Do nothing if alternate setting is 0 (stream closed).  */
  if (alternate_setting == 0)
  {    
    return;
  }

  BufferCtl.state = PLAY_BUFFER_OFFSET_UNKNOWN;
  
  /* Start reception (stream opened).  */
  ux_device_class_audio_reception_start(stream);
}

VOID __attribute__((optimize("O0"))) Audio_ReadDone(UX_DEVICE_CLASS_AUDIO_STREAM *stream, ULONG length)
{

  UCHAR         *frame_buffer;
  ULONG         frame_length;
  
  /* Get access to first audio input frame.  */
  ux_device_class_audio_read_frame_get(stream, &frame_buffer, &frame_length);

  if (length)
  {
    BufferCtl.wr_ptr += frame_length;

    if (BufferCtl.wr_ptr == AUDIO_TOTAL_BUF_SIZE)
    {
      /* All buffers are full: roll back */
      BufferCtl.wr_ptr = 0U;

      if (BufferCtl.state == PLAY_BUFFER_OFFSET_UNKNOWN)
      {

        /* Start BSP play */
        BufferCtl.state = PLAY_BUFFER_OFFSET_NONE;

        /* Put a message queue  */
        if(tx_queue_send(&ux_app_MsgQueue, &BufferCtl.state, TX_NO_WAIT) != TX_SUCCESS)
        {
          Error_Handler();
        }

      }
    }

    if (BufferCtl.rd_enable == 0U)
    {
      if (BufferCtl.wr_ptr == (AUDIO_TOTAL_BUF_SIZE / 2U))
      {
        BufferCtl.rd_enable = 1U;
      }
    }

    ux_utility_memory_copy(&BufferCtl.buff[BufferCtl.wr_ptr], frame_buffer, frame_length);

    play_count++;
  }
  /* Re-free the first audio input frame for transfer.  */
  ux_device_class_audio_read_frame_free(stream);
}

//static void Audio_ReadDone (UX_DEVICE_CLASS_AUDIO_STREAM * p_stream, ULONG actual_length)
//{
//    UINT    ux_err;
//    UCHAR * p_buffer;
//    ULONG   length;
//    UINT    i;
//    FSP_PARAMETER_NOT_USED(actual_length);
//    if (USB_APL_ON == g_read_alternate_setting)
//    {
//        ux_err = ux_device_class_audio_read_frame_get(p_stream, &p_buffer, &length);
//        if (UX_SUCCESS == ux_err)
//        {
//            for (i = 0; i < length; i++)
//            {
//                g_read_buf[g_read_wp][i] = *(p_buffer + i);
//            }
//            g_read_wp++;
//            g_read_wp %= NUM_OF_FRAME;
//            ux_device_class_audio_read_frame_free(p_stream);
//        }
//    }
//}

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
