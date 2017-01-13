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

## Hardware

### Rocket:
#### Electronics
Componenet Type | Part Number | Function
---:|---:|---:|
Processor/RF| nRF52 | Flight Control, launch control, telemetry, communication with ground station
Radio Amplifier (PA + LNA)| Skyworks 66112-11 | Boost RF range
IMU | MPU9250 | Orientation tracking, parachute deployment upon flight peak
Pressure | MPL3115A2 | Altitude tracking based on atmospheric pressure, parachute release timing upon flight peak
Humidity/Temperature | HTU21D | Environment monitoring
Servo Motors | Tower Pro SG90 | Parachute release mechanism
5V Boost Converter  | MIC2875 | Ignition and servo power
3F Supercapacitor | PM-5R0H305-R | High current ignition reservoir
3.3V LDO | AP7333-33SAG-7 | nRF52/sensor power
USB 4.2V LiPo Charger | AAT3681A | Charging

![Alt text](/hw/hw_screenshots/PCB_3d_view.png?raw=true "3D View")
![Alt text](/hw/hw_screenshots/PCB_stacked_view.png?raw=true "Stacked View")

#### Mechanical
Component Type | Part Number | Function
---:|---:|---:|
Rocket Motor| Klima D9-P | Propulsion
Electronic Starters | Klima | Ignition mechanism
Parachute | Klima | Rocket recovery
Nose Cone | 3D Printed | Body construction
Engine Mount + Fins | 3D Printed | Body construction

![Alt text](/hw/hw_screenshots/Rocket_assembly.png?raw=true "Rocket CAD Assembly")

[OnShape 3D CAD Files and Assemblies](https://cad.onshape.com/documents/a4d45ed022dbb8a2dc83f180/w/58fca6c234a131fe917f28c8/e/50b00e1e04cebf07ed807ff0 "Rocket CAD Files")


##Firmware

(Coming Soon!)
