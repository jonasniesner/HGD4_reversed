#ifndef _VARIANT_GENERIC_
#define _VARIANT_GENERIC_

#include "nrf.h"

#include "nrf_peripherals.h"

#if defined(NRF52_SERIES)
#define VARIANT_MCK       (64000000ul)
#else
#define VARIANT_MCK       (16000000ul)
#endif

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Number of pins defined in PinDescription array
#if GPIO_COUNT == 1
#define PINS_COUNT           (32u)
#define NUM_DIGITAL_PINS     (32u)
#elif GPIO_COUNT == 2
#define PINS_COUNT           (64u)
#define NUM_DIGITAL_PINS     (64u)
#else
#error "Unsupported GPIO_COUNT"
#endif
#define NUM_ANALOG_INPUTS    (6u)
#define NUM_ANALOG_OUTPUTS   (0u)

// PIN setup
#define GREEN_LED              (0)
#define RED_LED                (1)
#define LED_BUILTIN          GREEN_LED
#define PWR_LATCH              (2)
#define WAKE                   (9)
#define DONE                   (10)
#define I2C_SCL                (13)
#define I2C_SDA                (15)

/*
 * Analog pins
 */
#define PIN_A2               (3) // P0.03
#define PIN_A3               (4) // P0.04
#define PIN_A4               (5) // P0.05
#define PIN_A5               (6) // P0.06

static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;

#define ADC_RESOLUTION    14

/*
 * Serial interfaces
 */
// Serial
#define PIN_SERIAL_RX       (32)
#define PIN_SERIAL_TX       (33)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (12)
#define PIN_SPI_MOSI         (41)
#define PIN_SPI_SCK          (24)

static const uint8_t SS   = 25 ;
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (I2C_SDA)
#define PIN_WIRE_SCL         (I2C_SCL)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#ifdef __cplusplus
}
#endif

#endif
