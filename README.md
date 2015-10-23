#Atmel-JFET-Tester

JFET Tester based on Atmel Hardware (Currently AVR8/ATTiny)

Current ADCs supported:
* LTC2418 (24 bit SD, SPI 32b) ** Mostly written
* LTC2439 (16 bit SD, SPI 19b) ** Written/working 100%

Future ADCs to support:
* On-board if Cortex-M0+ and enough ADC pins

Current microcontrollers supported:
* ATTiny1634

Possible future uCs to support:
* ATMega16U/32U
* ATMega328(P)
* ATTiny84/85/87/167/861
* ATSAMD Cortex M0+ controllers
* ATSAM4LC2A Cortex M4 
* ATSAM3N* Cortex M3

Current hardware issues:
* Multiple board errata in 7.0.8-RC1 that require cutting traces, extra components
* Uncorrectable errata in board traces not allowing MOSFETs to switch to Idss mode
* Can only test N-fets

Current software issues:
* LTC2418 code NYI
* RAM usage higher than necessary
* Command prompt gets a new line (it shouldn't)

Planned Hardware+Software Upgrades for 7.1:
* Support for multiple ADCs
* Support for DAC output for AREF
* Support for Id/Gain vs Gate voltage
* Status LEDs
* Read JFET Vcc voltage with on-board ADC
* Calibration data in EEPROM

Planned Hardware+Software upgrades for 7.2:
* P-channel FET support
* Generation and manipulation of JFET Vcc (buck/boost converter on-board)
* Onboard FT232RL and USB port

Requirements: 
* D&W AG JFET Test Board 7.0.8+
* FTDI 5v TTL <-> USB cable (6 pin)
* ICSP device (ie AVRISP mk II)
  -- OR --
* Custom hardware/breadboard designed around this software
** Micro with 4kB+ flash, 512+ bytes of SRAM
** ADC hooked to any pins you want
** JFET test circuit implemented 
** 1 Pin to activate transistor to switch Idss/Vp mode globally
** Serial implementation on USART0 (Bluetooth?!)