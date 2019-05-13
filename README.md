# SJTU-USATL-PX4-v1.8.0 STABLE
上海交大无人系统实验室 PX4稳定固件-光流版
每次需要修改三个参数 AID_MASK(optical_flow); HGT(range_sensor); ll40ls(i2c);

pxihawk,pixhack v3 /dev/ttyS6,pixhawk4 /dev/ttyS3
需要在pixhawk4 中使用时，应在rcs中禁用ttyS3 mavlink内容
```
# UART mapping on OMNIBUSF4SD:
#
# USART1		/dev/ttyS0		SerialRX
# USART4		/dev/ttyS1		TELEM1
# USART6		/dev/ttyS2		GPS
#
# UART mapping on FMUv2/3/4:
#
# UART1			/dev/ttyS0		IO debug (except v4, there ttyS0 is the wifi)
# USART2		/dev/ttyS1		TELEM1 (flow control)
# USART3		/dev/ttyS2		TELEM2 (flow control)
# UART4
# UART7							CONSOLE
# UART8							SERIAL4
#
#
# UART mapping on FMUv5:
#
# UART1			/dev/ttyS0		GPS
# USART2		/dev/ttyS1		TELEM1 (flow control)
# USART3		/dev/ttyS2		TELEM2 (flow control)
# UART4			/dev/ttyS3		TELEM4
# USART6		/dev/ttyS4		TELEM3 (flow control)
# UART7			/dev/ttyS5		?
# UART8			/dev/ttyS6		CONSOLE
```


* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/Firmware/blob/master/LICENSE))
* [Supported airframes](https://docs.px4.io/en/airframes/airframe_reference.html) ([portfolio](http://px4.io/#airframes)):
  * [Multicopters](https://docs.px4.io/en/airframes/airframe_reference.html#copter)
  * [Fixed wing](https://docs.px4.io/en/airframes/airframe_reference.html#plane)
  * [VTOL](https://docs.px4.io/en/airframes/airframe_reference.html#vtol)
  * many more experimental types (Rovers, Blimps, Boats, Submarines, etc)
* Releases: [Downloads](https://github.com/PX4/Firmware/releases)


## PX4 Users

The [PX4 User Guide](https://docs.px4.io/en/) explains how to assemble [supported vehicles](https://docs.px4.io/en/airframes/airframe_reference.html) and fly drones with PX4. 
See the [forum and chat](https://docs.px4.io/en/#support) if you need help!


## PX4 Developers

This [Developer Guide](https://dev.px4.io/) is for software developers who want to modify the flight stack and middleware (e.g. to add new flight modes), hardware integrators who want to support new flight controller boards and peripherals, and anyone who wants to get PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the [Guide for Contributions](https://dev.px4.io/en/contribute/).
See the [forum and chat](https://dev.px4.io/en/#support) if you need help!


### Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://dev.px4.io/en/contribute/#dev_call).

> **Note** The dev call is open to all interested developers (not just the core dev team). This is a great opportunity to meet the team and contribute to the ongoing development of the platform. It includes a QA session for newcomers. All regular calls are listed in the [Dronecode calendar](https://www.dronecode.org/calendar/).


## Maintenance Team

  * Project: Founder - [Lorenz Meier](https://github.com/LorenzMeier), Architecture: [Daniel Agar](https://github.com/dagar)
    * [Dev Call](https://github.com/PX4/Firmware/labels/devcall) - [Ramon Roche](https://github.com/mrpollo)
  * Communication Architecture
    * [Beat Kueng](https://github.com/bkueng)
    * [Julian Oes](https://github.com/JulianOes)
  * UI / UX
    * [Donald Gagne](https://github.com/DonLakeFlyer)
    * [Gus Grubba](https://github.com/dogmaphobic)
  * [Multicopter Flight Control](https://github.com/PX4/Firmware/labels/multicopter)
    * [Dennis Mannhart](https://github.com/Stifael)
    * [Matthias Grob](https://github.com/MaEtUgR)
  * [VTOL Flight Control](https://github.com/PX4/Firmware/labels/vtol)
    * [Daniel Agar](https://github.com/dagar)
    * [Mathieu Bresciani](https://github.com/bresch)
    * [Sander Smeets](https://github.com/sanderux)
    * [Roman Bapst](https://github.com/tumbili)
    * [Andreas Antener](https://github.com/AndreasAntener)
  * [Fixed Wing Flight Control](https://github.com/PX4/Firmware/labels/fixedwing)
    * [Daniel Agar](https://github.com/dagar)
    * [Paul Riseborough](https://github.com/priseborough)
  * Racers - [Matthias Grob](https://github.com/MaEtUgR)
  * OS / drivers - [David Sidrane](https://github.com/davids5)
  * [UAVCAN](https://github.com/PX4/Firmware/labels/uavcan) / Industrial - [Pavel Kirienko](https://github.com/pavel-kirienko)
  * [State Estimation](https://github.com/PX4/Firmware/issues?q=is%3Aopen+is%3Aissue+label%3A%22state+estimation%22) - [James Goppert](https://github.com/jgoppert), [Paul Riseborough](https://github.com/priseborough)
  * Vision based navigation
    * [Christoph Tobler](https://github.com/ChristophTobler)
    * [Mohammed Kabir](https://github.com/mhkabir)
  * Obstacle Avoidance - [Martina Rivizzigno](https://github.com/mrivi)
  * [Snapdragon](https://github.com/PX4/Firmware/labels/snapdragon)
    * [Christoph Tobler](https://github.com/ChristophTobler)
  * [Intel Aero](https://github.com/PX4/Firmware/labels/intel%20aero)
    * [Sugnan Prabhu](https://github.com/sugnanprabhu)
    * [José Roberto de Souza](https://github.com/zehortigoza)
  * [Raspberry Pi / Navio](https://github.com/PX4/Firmware/labels/raspberry_pi) - [Beat Kueng](https://github.com/bkueng)
  * [Airmind MindPX / MindRacer](https://github.com/PX4/Firmware/labels/mindpx) - [Henry Zhang](https://github.com/iZhangHui)
  * RTPS/ROS2 Interface - [Vicente Monge](https://github.com/vicenteeprosima)

See also [About Us](http://px4.io/about-us/#development_team) (px4.io) and the [contributors list](https://github.com/PX4/Firmware/graphs/contributors) (Github).

## Supported Hardware

This repository contains code supporting these boards:
  * [Snapdragon Flight](https://docs.px4.io/en/flight_controller/snapdragon_flight.html)
  * [Intel Aero](https://docs.px4.io/en/flight_controller/intel_aero.html)
  * [Raspberry PI with Navio 2](https://docs.px4.io/en/flight_controller/raspberry_pi_navio2.html)
  * [Parrot Bebop 2](https://dev.px4.io/en/advanced/parrot_bebop.html)
  * FMUv2.x
    * [Pixhawk](https://docs.px4.io/en/flight_controller/pixhawk.html)
    * [Pixhawk Mini](https://docs.px4.io/en/flight_controller/pixhawk_mini.html)
    * [Pixfalcon](https://docs.px4.io/en/flight_controller/pixfalcon.html)
  * FMUv3.x [Pixhawk 2](https://pixhawk.org/modules/pixhawk2)
  * FMUv4.x
    * [Pixracer](https://docs.px4.io/en/flight_controller/pixracer.html)
    * [Pixhawk 3 Pro](https://docs.px4.io/en/flight_controller/pixhawk3_pro.html)
  * FMUv5.x (ARM Cortex M7, future Pixhawk)
  * [STM32F4Discovery](http://www.st.com/en/evaluation-tools/stm32f4discovery.html) (basic support) [Tutorial](https://pixhawk.org/modules/stm32f4discovery)
  * [Gumstix AeroCore](https://www.gumstix.com/aerocore-2/) (only v2)
  * [Airmind MindPX V2.8](http://www.mindpx.net/assets/accessories/UserGuide_MindPX.pdf)
  * [Airmind MindRacer V1.2](http://mindpx.net/assets/accessories/mindracer_user_guide_v1.2.pdf)
  * [Bitcraze Crazyflie 2.0](https://docs.px4.io/en/flight_controller/crazyflie2.html)

Additional information about supported hardware can be found in [PX4 user Guide > Autopilot Hardware](https://docs.px4.io/en/flight_controller/).

## Project Roadmap

A high level project roadmap is available [here](https://www.dronecode.org/roadmap/).
