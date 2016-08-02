#ifndef PCA20027_H
#define PCA20027_H

/*I2C Pins (ext. PU 4.7KOhm)*/
#define TWI_SCL_PIN_NUMBER         (7)
#define TWI_SDA_PIN_NUMBER         (8)

/*Servo PWM Pins*/
#define SERVO_1         (20)
#define SERVO_2         (19)
#define SERVO_3         (18)
#define SERVO_4         (17)
#define SERVO_5         (16)

/*Level Shifter Enable*/
#define SERVO_ENABLE    (15)

/*RGB LED*/
#define LED_R           (28)
#define LED_G           (30)
#define LED_B           (29)

/*5V Boost Regulator*/
#define BOOST_5V_ENABLE (11)
#define POWER_GOOD      (4) //Inverted logic

/*Ignition Supercap*/
#define IGNITION_CH1    (13) //Turn on power mosfets
#define IGNITION_CH2    (14)
#define VSC_SENSE       (5) //Measure supercap voltage
#define SC_DUMP         (12)//Discharge the supercapacitor

/*Sensor Interrupts*/
#define MPU_INT         (6)
#define MPL_INT1        (9)
#define MPL_INT2        (10)

/*RF PA/LNA*/
#define CTX             (26)
#define CRX             (24)
#define CSD             (23)
#define ANT_SEL         (22)
#define CPS             (27)
#define CHL             (25)

/*LF XTAL*/
// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}


#endif /*PCA20027_H*/
