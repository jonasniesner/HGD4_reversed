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

// LEDs
#define PIN_LED              (13) // P0.13
#define LED_BUILTIN          PIN_LED

/*
 * Analog pins
 */
#define PIN_A0               (1) // P0.01
#define PIN_A1               (2) // P0.02
#define PIN_A2               (3) // P0.03
#define PIN_A3               (4) // P0.04
#define PIN_A4               (5) // P0.05
#define PIN_A5               (6) // P0.06

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;
#if defined(NRF52_SERIES)
#define ADC_RESOLUTION    14
#else
#define ADC_RESOLUTION    10
#endif

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

#define PIN_WIRE_SDA         (15)
#define PIN_WIRE_SCL         (13)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#ifdef __cplusplus
}
#endif

#endif
