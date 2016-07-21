# nRF52-Strato v0.5 HW Bring Up

## Basic Power - Bare Board
* Check for shorts between power rails and ground via test points
    * VDD and GND (TP3 & TP4)
    * VBOOST_5V and GND (TP1 & TP4)
    * VBAT and GND (TP 2 & TP4)


* Using the desktop power supplies, apply the corresponding voltage to each power rail with a current limit of 1 mA, there should not be significant current draw.
    * Check that all nets that should receive the appropriate voltage does so accordingly

## Basic Connection - Bare Board
    * Check all GPIO pads from the nRF are connected to the appropriate pads of the peripherals
    * Check that all GND pins of all components are grounded (!!! NOTE: PIN 11 of the MPU 9250 is not grounded)

## Basic Power - Populated Board
* Check for shorts between power rails and ground via test points
    * VDD and GND (TP3 & TP4)
    * VBOOST_5V and GND (TP1 & TP4)
    * VBAT and GND (TP 2 & TP4)

* Using the desktop power supplies, apply the corresponding voltage to each power rail with a current limit of 10 mA, there should not be significant current draw.
    * Check that all nets that should receive the appropriate voltage does so accordingly

## Basic Power - Populated Board

## Basic Programming
* Via the J-link programming header, flash in the S132 Softdevice
