/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/

/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** USBX Component                                                        */
/**                                                                       */
/**   Device Audio Class                                                    */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define UX_SOURCE_CODE


/* Include necessary system files.  */

#include "ux_api.h"
#include "ux_device_class_audio.h"
#include "ux_device_class_audio10.h"
#include "ux_device_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_device_class_audio10_control_process            PORTABLE C      */
/*                                                           6.1.6        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function manages the based sent by the host on the control     */
/*    endpoints with a CLASS or VENDOR SPECIFIC type.                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    audio                                 Address of audio class        */
/*                                            instance                    */
/*    transfer                              Address of transfer request   */
/*                                            instance                    */
/*    group                                 Request process data          */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_utility_short_get                 Get 2-byte value from buffer  */
/*    _ux_utility_short_put                 Put 2-byte value to buffer    */
/*    _ux_device_stack_transfer_request     Issue a transfer request      */
/*    _ux_device_stack_endpoint_stall       Endpoint stall                */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    ThreadX                                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*  09-30-2020     Chaoqiong Xiao           Modified comment(s),          */
/*                                            resulting in version 6.1    */
/*  04-02-2021     Chaoqiong Xiao           Modified comment(s),          */
/*                                            added volume RES support,   */
/*                                            resulting in version 6.1.6  */
/*                                                                        */
/**************************************************************************/
UINT _ux_device_class_audio10_control_process(UX_DEVICE_CLASS_AUDIO *audio,
    UX_SLAVE_TRANSFER *transfer, UX_DEVICE_CLASS_AUDIO10_CONTROL_GROUP *group)
{

UX_SLAVE_ENDPOINT                   *endpoint;
UX_DEVICE_CLASS_AUDIO10_CONTROL     *control;
UCHAR                               request;
UCHAR                               unit_id;
UCHAR                               control_selector;
UCHAR                               channel_number;
ULONG                               request_length;
ULONG                               i;


    /* Get instances.  */
    endpoint = &audio -> ux_device_class_audio_device -> ux_slave_device_control_endpoint;
    transfer = &endpoint -> ux_slave_endpoint_transfer_request;

    /* Extract all necessary fields of the request.  */
    request          = *(transfer -> ux_slave_transfer_request_setup + UX_DEVICE_CLASS_AUDIO_REQUEST_REQUEST);
    unit_id          = *(transfer -> ux_slave_transfer_request_setup + UX_DEVICE_CLASS_AUDIO_REQUEST_ENEITY_ID);
    control_selector = *(transfer -> ux_slave_transfer_request_setup + UX_DEVICE_CLASS_AUDIO_REQUEST_CONTROL_SELECTOR);
    channel_number   = *(transfer -> ux_slave_transfer_request_setup + UX_DEVICE_CLASS_AUDIO_REQUEST_CHANNEL_NUMBER);
    request_length   = _ux_utility_short_get(transfer -> ux_slave_transfer_request_setup + UX_SETUP_LENGTH);

    /* Process controls one by one.  */
    for (i = 0; i < group -> ux_device_class_audio10_control_group_controls_nb; i ++)
    {
        control = &group -> ux_device_class_audio10_control_group_controls[i];

        /* Reset change map.  */
        control -> ux_device_class_audio10_control_changed = 0;

        /* We handle feature unit requests.  */
        if (unit_id == control -> ux_device_class_audio10_control_fu_id)
        {

            /* Handle the request.  */
            switch(request)
            {
            case UX_DEVICE_CLASS_AUDIO10_SET_CUR:

                /* We only support master channel, so channel number must be 0 or 0xFF.  */
                if (channel_number != 0 && channel_number != 0xFF)
                    break;

                /* Set request.  */
                switch(control_selector)
                {
                case UX_DEVICE_CLASS_AUDIO10_FU_MUTE_CONTROL:

                    if (transfer -> ux_slave_transfer_request_actual_length < 1)

                        /* No change applied.  */
                        break;

                    if (control -> ux_device_class_audio10_control_mute[0] != transfer -> ux_slave_transfer_request_data_pointer[0])
                    {
                        control -> ux_device_class_audio10_control_changed = UX_DEVICE_CLASS_AUDIO10_CONTROL_MUTE_CHANGED;
                        control -> ux_device_class_audio10_control_mute[0] = transfer -> ux_slave_transfer_request_data_pointer[0];

                    }

                    /* Done success.  */
                    return(UX_SUCCESS);

                case UX_DEVICE_CLASS_AUDIO10_FU_VOLUME_CONTROL:

                    if (transfer -> ux_slave_transfer_request_actual_length < 2)

                        /* No change applied.  */
                        break;

                    if (control -> ux_device_class_audio10_control_volume[0] != (SHORT)_ux_utility_short_get(transfer -> ux_slave_transfer_request_data_pointer))
                    {
                        control -> ux_device_class_audio10_control_changed = UX_DEVICE_CLASS_AUDIO10_CONTROL_VOLUME_CHANGED;
                        control -> ux_device_class_audio10_control_volume[0] = (SHORT)_ux_utility_short_get(transfer -> ux_slave_transfer_request_data_pointer);
                    }

                    /* Done success.  */
                    return(UX_SUCCESS);


                default:

                    /* No change applied.  */
                    break;
                }

                /* Request or parameter problem.  */
                break;

            case UX_DEVICE_CLASS_AUDIO10_GET_MIN:
            case UX_DEVICE_CLASS_AUDIO10_GET_MAX:
            case UX_DEVICE_CLASS_AUDIO10_GET_RES:
            case UX_DEVICE_CLASS_AUDIO10_GET_CUR:

                /* We only support master channel, so channel number must be 0 or 0xFF.  */
                if (channel_number != 0 && channel_number != 0xFF)
                    break;

                /* Get request.  */
                switch(control_selector)
                {
                case UX_DEVICE_CLASS_AUDIO10_FU_MUTE_CONTROL:

                    /* We only support _CUR.  */
                    if (request != UX_DEVICE_CLASS_AUDIO10_GET_CUR)
                        break;

                    /* Not enough buffer for data.  */
                    if (request_length < 1)
                        break;

                    /* Send mute status.  */
                    transfer -> ux_slave_transfer_request_data_pointer[0] = (UCHAR)control -> ux_device_class_audio10_control_mute[0];
                    _ux_device_stack_transfer_request(transfer, 1, request_length);

                    /* Done success.  */
                    return(UX_SUCCESS);

                case UX_DEVICE_CLASS_AUDIO10_FU_VOLUME_CONTROL:

                    /* Not enough buffer for data.  */
                    if (request_length < 2)
                        break;

                    /* Send volume value.  */
                    _ux_utility_short_put(transfer -> ux_slave_transfer_request_data_pointer, (USHORT)
                                          (request == UX_DEVICE_CLASS_AUDIO10_GET_MIN ? control -> ux_device_class_audio10_control_volume_min[0] :
                                           (request == UX_DEVICE_CLASS_AUDIO10_GET_MAX ? control -> ux_device_class_audio10_control_volume_max[0] :
                                            (request == UX_DEVICE_CLASS_AUDIO10_GET_RES ? UX_MAX(1, control -> ux_device_class_audio10_control_volume_res[0]) :
                                              control -> ux_device_class_audio10_control_volume[0]))));
                    _ux_device_stack_transfer_request(transfer, 2, request_length);

                    /* Done success.  */
                    return(UX_SUCCESS);

                default:
                    break;
                }

            default:
                break;
            }

            /* We are here when there is error, break the loop.  */
            break;
        }

        /* Now try next.  */
    }


    /* Request or parameter not supported.  */
    _ux_device_stack_endpoint_stall(endpoint);

    /* Done error.  */
    return(UX_ERROR);
}
