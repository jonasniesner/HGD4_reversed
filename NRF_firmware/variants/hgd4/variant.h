#ifndef _VARIANT_GENERIC_
#define _VARIANT_GENERIC_

#include "nrf.h"

#include "nrf_peripherals.h"

#define VARIANT_MCK       (64000000ul)

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#define PINS_COUNT           (64u)
#define NUM_DIGITAL_PINS     (64u)

#define GREEN_LED              (0)
#define RED_LED                (1)
#define VBAT_DIV               (2)
#define ACCL1_CS               (4)
#define ACCL2_CS               (6)
#define SPI_MISO               (8)
#define WAKE                   (9)
#define DONE                   (10)
#define SENSOR_PWR             (11)
#define SPI_CLK                (12)
#define I2C_SCL                (13)
#define I2C_SDA                (15)
#define MODEM_RXD              (17)
#define MODEM_CON_A            (20)
#define MODEM_TXD              (22)
#define MODEM_CON_B            (24)
#define ACC_INT                (26)
#define MODEM_ESP_PWR          (29)
#define PWR_LATCH              (31)
#define DEBUG_RXD              (32)
#define DEBUG_TXD              (33)
#define MODEM_PWRKEY           (34)
#define PWR_SW_IN              (38)
#define SPI_MOSI               (41)
#define ESP_RXD                (42)
#define ESP_TXD                (43)
#define ESP_PWR                (45)
#define ESP_GPIO0              (47)

#define ADC_RESOLUTION    14

#define LED_BUILTIN          GREEN_LED

#define PIN_SERIAL1_RX MODEM_RXD
#define PIN_SERIAL1_TX MODEM_TXD

#define LED_STATE_ON -1
#define LED_BLUE -1

#define HAVE_HWSERIAL2

#define PIN_SERIAL2_RX ESP_RXD
#define PIN_SERIAL2_TX ESP_TXD

#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         SPI_MISO
#define PIN_SPI_MOSI         SPI_MOSI
#define PIN_SPI_SCK          SPI_CLK

static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (I2C_SDA)
#define PIN_WIRE_SCL         (I2C_SCL)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define AL_ADDR 0x29
#define TINY_GSM_MODEM_BG95
#define SerialAT Serial1

#ifdef __cplusplus
}
#endif

#endif
