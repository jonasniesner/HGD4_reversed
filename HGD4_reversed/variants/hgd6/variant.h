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
#define PWR_LATCH              (2)
#define ACCL1_CS               (4)
#define ACCL2_CS               (6)
#define SPI_MISO               (8)
#define WAKE                   (9)
#define DONE                   (10)
#define SPI_CLK                (12)
#define I2C_SCL                (13)
#define I2C_SDA                (15)
#define MODEM_RXD              (17)
#define MODEM_TXD              (22)
#define MODEM_ESP_PWR          (29)
#define DEBUG_RXD              (32)
#define DEBUG_TXD              (33)
#define MODEM_PWRKEY           (34)
#define SPI_MOSI               (41)
#define ESP_RXD                (42)
#define ESP_TXD                (43)
#define ESP_PWR                (45)

#define PIN_A2               (3)
#define PIN_A3               (4)
#define PIN_A4               (5)
#define PIN_A5               (6)

static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
static const uint8_t A4  = PIN_A4 ;
static const uint8_t A5  = PIN_A5 ;

#define ADC_RESOLUTION    14

#define LED_BUILTIN          GREEN_LED

#define PIN_SERIAL_RX       DEBUG_RXD
#define PIN_SERIAL_TX       DEBUG_TXD

#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         SPI_MISO
#define PIN_SPI_MOSI         SPI_MOSI
#define PIN_SPI_SCK          SPI_CLK

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

#define AL_ADDR 0x29

#ifdef __cplusplus
}
#endif

#endif
