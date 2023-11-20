# USBX_AUDIO

This project aims to develop a functional USB Audio Class 2 device on STM32 MCUs.

Initial development stage is loosely based on this [example](https://github.com/STMicroelectronics/x-cube-azrtos-h7/tree/dev/usbx/Projects/STM32H743I-EVAL/Applications/USBX/Ux_Device_Audio) running on a H7 core. The F4 Discovery board provides only the Full speed USB port, restricting bandwidth significantly.

## Milestones

- [X] UAC2 correct enumeration
- [ ] Control requests (Volume, mute etc)
- [X] Data reception (Host -> Device)
- [X] Audio playback from DAC
- [ ] Multi samplerate support
- [ ] 16-bit/24-bit support
- [ ] Clock synchronisation (via iso async feedback)
- [ ] Audio capture (Device -> Host) [1]
- [ ] High speed implementation [2]

[1] Will likely send test tone/DC button push

[2] Requires a platform with high speed USB port available

