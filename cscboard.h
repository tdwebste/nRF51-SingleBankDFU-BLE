// will be deleted, leaving so you can fix your code
#ifndef NRF6100_10001_H__
#define NRF6100_10001_H__

#include "nrf_gpio.h"

//11 is led, but appears to draw too much power
//03 is TP5 will be used to mimic the internal LED
#define LED_0                                           11
#define LED_1                                           03

// debug jig self check indication
#define UNUSED0                                         00

//testing
#define RF_TEST_IN_PIN_30                               30
#define RF_TEST_IN_PIN_00                               00
#define RF_TEST_IN_PIN_01                               01
#define RF_TEST_IN_PIN_02                               02

#define RF_TEST_IN_PIN                                  RF_TEST_IN_PIN_30
#define TEST_HMC_PIN                                    RF_TEST_IN_PIN_01
#define TEST_MPU_PIN                                    RF_TEST_IN_PIN_02

//test points
#define MPU_INT                                         8
#define M_DRDY									        04

//keep simple uart happy
//debuging by uart is possible
#define RX_PIN_NUMBER  	01
#define TX_PIN_NUMBER  	02
#define CTS_PIN_NUMBER 	03
#define RTS_PIN_NUMBER 	00
#define HWFC		false


#endif  // NRF6310_10001_H__
