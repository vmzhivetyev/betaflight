
# What is this

This fork is maintained to include all actual features from [Betaflight master branch](https://github.com/betaflight/betaflight).

I wanted to implement some features I need myself but I don't have enough time to make PRs to the Betaflight repo. Btw I'm pretty sure some of the modifications are too niche to be merged anyways.

# Features and changes in this fork 

## Show used Blackbox percent in OSD while armed

- Shows used percent (up to tenths) of blackbox memory **in realtime**. 
- Enable `Blackbox Log Status` OSD element to see the percent.
- This feature can be disabled with `set osd_show_blackbox_percent=OFF`.

> [!WARNING] 
> This is tested only with Flash blackbox memory, current implementation is most probably very slow and bad if you have an SD card blackbox storage. Disable this feature in such case.

## Auto-exit mass storage mode

- Auto-exit Mass Storage Mode if nothing was read from the storage during the last 10 seconds
- This feature can not be disabled.

## Configurable sampling rate for vbat and current

- You can configure the sampling rate of ADC for both battery voltage and current readings.
- `set current_meter_adc_hz=1000` for example.
- Default value is 50Hz which is the same as it was without this feature.

## Proper processing of EDT + Alarms

[What is **extended DSHOT telemetry**?](https://github.com/bird-sanctuary/extended-dshot-telemetry)

Extended DSHOT telemtry is properly parsed (based on https://github.com/betaflight/betaflight/pull/13855).

We are especially interested in the "Max Stress Level" reported by the ESC which represent how well the commutation of the motor is going on. Higher values = worse commutation (more heat, less torque, higher risk of desyncs). This value is reported by the ESC once a second.

Includes mutiple new debug modes. Proper interpretation of such debug data is not supported by the Blackbox Explorer.

New OSD element is added to show current values of "Max Stress Level" for each motor (4 numbers in 4 lines one under another just like rpm data). It's also colored (yellow when ESC reports demag timeouts and red when ESC reports desync). To enable it check the "Unknown 1" OSD Element in the list of elements in OSD tab of Betaflight Configurator.

> [!NOTE]  
> In Bluejay 0.21.0 the reported "Max Stress Level" is the maximum value since the quad was armed (not the max during the last second). This can be fixed with [custom Bluejay build](https://github.com/bird-sanctuary/bluejay/commit/1b61ea2345dc435f9e0b7a994e29d7e772325f71).

> [!IMPORTANT]
> Enable extended DSHOT telemtry with `set dshot_edt = ON`.

**Alarms**

Added configurable alarm for the "Max Stress Level" which can be from 0 to 15 (`set osd_esc_stress_alarm=9`, set to `0` to disable the alarm, it's also configurable from OSD menu). 

If your alarm kicks in it will never go away until you disarm (See the note above about Bluejay 0.21.0). 

The alarm text is `ESC <MOTOR_ID><ALARM_TYPE>`:

- `MOTOR_ID` is the number of the motor (1-4). 
- `ALARM_TYPE` can be 
	- `X` - stress level threshold is reached
	- `E` - the ESC is reporting that the motor is stalled. 

Alarm can include different flags for different motors at the same time. 

## Altitude hold

> [!WARNING]  
> * This is NOT a position hold.
> * Current implementation ONLY uses BARO. 
> * Current implementation MAY lack safety checks.
> * Current implementation is DIFFERENT and REPLACES the [implementation in Betaflight's master](https://github.com/betaflight/betaflight/pull/13816).
> * BEFORE disabling the ALTHOLD mode make sure you LOWER the throttle and CENTER the roll-pitch stick!

* Works reliably based on BARO only.
* Doesn't burn your motors even you have non-protected baro.
* Allows for extensive configuration and tuning.
* When ALTHOLD is active throttle position controls target altitude, throttle at zero lowers the target, throttle at max increases the target. Keep the throttle around 50% to not change the target altitude.
* Target altitude is displayed in the OSD element which shows current altitude (appended on the right).
* Tuning can be hard, recommended parameters based on my tests:
```python
set althold_pid_p = 60
set althold_pid_d = 40
set althold_pid_i = 5

# limits how much I can be accumulated (10 = 10% of throttle)
set althold_pid_imax = 10

# low pass filter for d-term, 10 == 1 Hz
set althold_pid_d_cutoff = 15

# low pass filter for motors control, 10 == 1 Hz
set althold_throttle_cutoff = 255

# low pass filter for baro data, 10 == 1 Hz
set althold_altitude_cutoff = 20

# min throttle % that the ALTHOLD can send to motors
set althold_throttle_min = 15

# max throttle % that the ALTHOLD can send to motors
set althold_throttle_max = 50

# set this to % of throttle your quad hovers at
set althold_throttle_hover = 28

# limit max target altitude
# has no effect if set to 0 or current target altitude is greater than the limit (you engaged ALTHOLD mode while higher than the limit)
set althold_max_altitude = 100

# 1 = 0.1s, how much time it takes to fade controls from your stick inputs to ALTHOLD algorithm
set althold_enter_fade_deciseconds = 1

# 1 = 0.1s, how much time it takes to fade controls from ALTHOLD algorithm to your stick inputs
set althold_exit_fade_deciseconds = 1  
```
 


___

# Original README

![Betaflight](images/bf_logo.png)

[![Latest version](https://img.shields.io/github/v/release/betaflight/betaflight)](https://github.com/betaflight/betaflight/releases) [![Build](https://img.shields.io/github/actions/workflow/status/betaflight/betaflight/nightly.yml?branch=master)](https://github.com/betaflight/betaflight/actions/workflows/nightly.yml) [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![Join us on Discord!](https://img.shields.io/discord/868013470023548938)](https://discord.gg/n4E6ak4u3c)

Betaflight is flight controller software (firmware) used to fly multi-rotor craft and fixed wing craft.

This fork differs from Baseflight and Cleanflight in that it focuses on flight performance, leading-edge feature additions, and wide target support.

## Events

| Date  | Event |
| - | - |
| 28-04-2024 | Firmware 4.5 Release |


## News

### Requirements for the submission of new and updated targets

The following new requirements for pull requests adding new targets or modifying existing targets are put in place from now on:

1. Read the [hardware specification](https://betaflight.com/docs/development/manufacturer/manufacturer-design-guidelines)

2. No new F3 based targets will be accepted;

3. For any new target that is to be added, only a Unified Target config into https://github.com/betaflight/unified-targets/tree/master/configs/default needs to be submitted. See the [instructions](https://betaflight.com/docs/manufacturer/creating-an-unified-target) for how to create a Unified Target configuration. If there is no Unified Target for the MCU type of the new target (see instructions above), then a 'legacy' format target definition into `src/main/target/` has to be submitted as well;

4. For changes to existing targets, the change needs to be applied to the Unified Target config in https://github.com/betaflight/unified-targets/tree/master/configs/default. If no Unified Target configuration for the target exists, a new Unified Target configuration will have to be created and submitted. If there is no Unified Target for the MCU type of the new target (see instructions above), then an update to the 'legacy' format target definition in `src/main/target/` has to be submitted alongside the update to the Unified Target configuration.


## Features

Betaflight has the following features:

* Multi-color RGB LED strip support (each LED can be a different color using variable length WS2811 Addressable RGB strips - use for Orientation Indicators, Low Battery Warning, Flight Mode Status, Initialization Troubleshooting, etc)
* DShot (150, 300 and 600), Multishot, Oneshot (125 and 42) and Proshot1000 motor protocol support
* Blackbox flight recorder logging (to onboard flash or external microSD card where equipped)
* Support for targets that use the STM32 F4, G4, F7 and H7 processors
* PWM, PPM, SPI, and Serial (SBus, SumH, SumD, Spektrum 1024/2048, XBus, etc) RX connection with failsafe detection
* Multiple telemetry protocols (CRSF, FrSky, HoTT smart-port, MSP, etc)
* RSSI via ADC - Uses ADC to read PWM RSSI signals, tested with FrSky D4R-II, X8R, X4R-SB, & XSR
* OSD support & configuration without needing third-party OSD software/firmware/comm devices
* OLED Displays - Display information on: Battery voltage/current/mAh, profile, rate profile, mode, version, sensors, etc
* In-flight manual PID tuning and rate adjustment
* PID and filter tuning using sliders
* Rate profiles and in-flight selection of them
* Configurable serial ports for Serial RX, Telemetry, ESC telemetry, MSP, GPS, OSD, Sonar, etc - Use most devices on any port, softserial included
* VTX support for Unify Pro and IRC Tramp
* and MUCH, MUCH more.

## Installation & Documentation

See: https://betaflight.com/docs/wiki

## Support and Developers Channel

There's a dedicated Discord server here:

https://discord.gg/n4E6ak4u3c

We also have a Facebook Group. Join us to get a place to talk about Betaflight, ask configuration questions, or just hang out with fellow pilots.

https://www.facebook.com/groups/betaflightgroup/

Etiquette: Don't ask to ask and please wait around long enough for a reply - sometimes people are out flying, asleep or at work and can't answer immediately.

## Configuration Tool

To configure Betaflight you should use the Betaflight-configurator GUI tool (Windows/OSX/Linux) which can be found here:

https://github.com/betaflight/betaflight-configurator/releases/latest

## Contributing

Contributions are welcome and encouraged. You can contribute in many ways:

* implement a new feature in the firmware or in configurator (see [below](#Developers));
* documentation updates and corrections;
* How-To guides - received help? Help others!
* bug reporting & fixes;
* new feature ideas & suggestions;
* provide a new translation for configurator, or help us maintain the existing ones (see [below](#Translators)).

The best place to start is the Betaflight Discord (registration [here](https://discord.gg/n4E6ak4u3c)). Next place is the github issue tracker:

https://github.com/betaflight/betaflight/issues
https://github.com/betaflight/betaflight-configurator/issues

Before creating new issues please check to see if there is an existing one, search first otherwise you waste people's time when they could be coding instead!

If you want to contribute to our efforts financially, please consider making a donation to us through [PayPal](https://paypal.me/betaflight).

If you want to contribute financially on an ongoing basis, you should consider becoming a patron for us on [Patreon](https://www.patreon.com/betaflight).

## Developers

Contribution of bugfixes and new features is encouraged. Please be aware that we have a thorough review process for pull requests, and be prepared to explain what you want to achieve with your pull request.
Before starting to write code, please read our [development guidelines](https://betaflight.com/docs/development) and [coding style definition](https://betaflight.com/docs/development/CodingStyle).

GitHub actions are used to run automatic builds

## Translators

We want to make Betaflight accessible for pilots who are not fluent in English, and for this reason we are currently maintaining translations into 21 languages for Betaflight Configurator: Català, Dansk, Deutsch, Español, Euskera, Français, Galego, Hrvatski, Bahasa Indonesia, Italiano, 日本語, 한국어, Latviešu, Português, Português Brasileiro, polski, Русский язык, Svenska, 简体中文, 繁體中文.
We have got a team of volunteer translators who do this work, but additional translators are always welcome to share the workload, and we are keen to add additional languages. If you would like to help us with translations, you have got the following options:
- if you help by suggesting some updates or improvements to translations in a language you are familiar with, head to [crowdin](https://crowdin.com/project/betaflight-configurator) and add your suggested translations there;
- if you would like to start working on the translation for a new language, or take on responsibility for proof-reading the translation for a language you are very familiar with, please head to the Betaflight Discord chat (registration [here](https://discord.gg/n4E6ak4u3c)), and join the ['translation'](https://discord.com/channels/868013470023548938/1057773726915100702) channel - the people in there can help you to get a new language added, or set you up as a proof reader.

## Hardware Issues

Betaflight does not manufacture or distribute their own hardware. While we are collaborating with and supported by a number of manufacturers, we do not do any kind of hardware support.
If you encounter any hardware issues with your flight controller or another component, please contact the manufacturer or supplier of your hardware, or check RCGroups https://rcgroups.com/forums/showthread.php?t=2464844 to see if others with the same problem have found a solution.

## Betaflight Releases

https://github.com/betaflight/betaflight/releases

## Open Source / Contributors

Betaflight is software that is **open source** and is available free of charge without warranty to all users.

Betaflight is forked from Cleanflight, so thanks goes to all those who have contributed to Cleanflight and its origins.

Origins for this fork (Thanks!):
* **Alexinparis** (for MultiWii),
* **timecop** (for Baseflight),
* **Dominic Clifton** (for Cleanflight),
* **borisbstyle** (for Betaflight), and
* **Sambas** (for the original STM32F4 port).

The Betaflight Configurator is forked from Cleanflight Configurator and its origins.

Origins for Betaflight Configurator:
* **Dominic Clifton** (for Cleanflight configurator), and
* **ctn** (for the original Configurator).

Big thanks to current and past contributors:
* Budden, Martin (martinbudden)
* Bardwell, Joshua (joshuabardwell)
* Blackman, Jason (blckmn)
* ctzsnooze
* Höglund, Anders (andershoglund)
* Ledvina, Petr (ledvinap) - **IO code awesomeness!**
* kc10kevin
* Keeble, Gary (MadmanK)
* Keller, Michael (mikeller) - **Configurator brilliance**
* Kravcov, Albert (skaman82) - **Configurator brilliance**
* MJ666
* Nathan (nathantsoi)
* ravnav
* sambas - **bringing us the F4**
* savaga
* Stålheim, Anton (KiteAnton)

And many many others who haven't been mentioned....
