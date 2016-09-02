# nRF52 Rocket

#### Table of Contents
* [Introduction](#introduction)
* [Hardware](#hardware)
* [Firmware](#firmware)

This repository contains the firmware and hardware files for the first launched BLE rocket prototype.

## Introduction

As the BLE standard (namely Bluetooth 5.0) enables higher TX power and thusly communication range, it becomes more and more viable to use Bluetooth LE for mid-range RC toys in the proximity of around 100 - 500 m. BLE provides the benefits of high interoperability amongst different manufacturers due to rigorous standardization compared to other frequencies, yet less power consuming than Wifi.

The model rocketry industry drives a very popular hobby, especially in the U.S., that inspires youth and adults to engage in science and engineering. Yet, through some web research it appears that there is a gap between really simple beginner rockets with less than 200 m of flight range which are completely made out of cardboard and wood with no electronic flight telemetry or electronic parachute deployment; and the advanced heavy duty rockets with expensive electronics that can go up to kilometers.

A BLE enabled model rocket can serve the hobbyists who want to be a little more serious with their rocket flying and track their rockets performance in real time and have smart parachute deployment, but do not have the funds or the skills to buy/build a super hardcore rocket that costs thousands of dollars.


BOTTOM SECTIONS INCOMPLETE STILL...


## Hardware

### Rocket:
#### Electronics
Componenet Type | Part Number | Function
---:|---:|---:|
Processor/RF| nRF52 | Flight Control, launch control, telemetry, communication with ground station
Radio Amplifier (PA + LNA)| Skyworks 66112-11 | Boost RF range
IMU | MPU9250 or ICM-20948 | Orientation tracking, parachute deployment upon flight peak
High G Accelerometer | ADXL377 (200 g) | Acceleration and Speed tracking during launch
Pressure | MPL3115A2 | Altitude tracking based on atmospheric pressure, parachute tracking upon flight peak
Humidity/Temperature | HTU21D | Environment monitoring
Camera Module | ... | Video Capturing
GPS |...| Location tracking
Servo Motors | ... | Fin rotation, parachute release
Relay | ... | Various triggering mechanisms: parachute, multistage booster
6 or 9V Boost Converter  | ... | Ignition Power
Li-ion Battery | ... | Power source
NFC Antenna | ... | Touch to Pair
LEDs | ... | UI indication

#### Mechanical
Component Type | Part Number | Function
---:|---:|---:|
Rocket Motor| Klima D9-P | Propulsion
Electronic Starters | Klima | Ignition mechanism
Alligator clips | ... | Ignition mechanism
Parachute | ... with Nordic Logo | Rocket recovery
Body Tube |From Biltema | Body construction
Nose Cone | 3D Printed | Body construction
Engine Mount + Fins | 3D Printed | Body construction
Camera Mount  | 3D Printed | Body construction

### Ground Station

#### Electronics
Componenet Type | Part Number | Function
---:|---:|---:|
Processor/RF| nRF52 | Launch control, communication with mobile
Radio Amplifier (PA + LNA)| Skyworks 66112-11 | Boost RF range
Patch Antenna | ... | Boost RF range, points up into the sky
LEDs | ... | UI indication
Push Buttons | ... | Launch security
9V battery | ... | Ignition power
Alligator clips | ... | Ignition mechanism

#### Mechanical
Componenet Type | Part Number | Function
---:|---:|---:|
Casing | 3D Printed | Casing for ground station electronics
Launching Guide | Some sort of metal rod | Guides the rocket vertically up
Launching Guide Base | Tripod of some sort | Base support for the metal rod

## Basic Goal - NEVER LOSE LINE OF SIGHT
* Component Tests
    * Vacuum or Drone test of Altimeter
    * Freefall detection test of Accelerometer
    * Parachute hatch opening
    * Parachute deployment
    * Motor ignition
    * Telemetry range test (with real time altitude and acceleration, attitude)

* First Rocket Test - Launchable Rocket
    * Single stage small rocket motor first (B, or C class)
    * Altimeter and freefall timer parachute release, BLE as back up.
    * Launch with phone

### Stretch Goals
* Servo controlled fins
* Multistage launch
* Logging of data onboard
* Launch with ground station (Better range)
