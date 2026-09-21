/* SX1276/77/78/79 driver for STM32 (C99, HAL).
 * Port of sandeepmistry/arduino-LoRa. MIT license.
 *
 * Wiring: SPI master, MODE0, MSB first, NSS driven by software (this driver).
 *
 *   lora_hw_t hw = { &hspi1, NSS_GPIO_Port, NSS_Pin, RST_GPIO_Port, RST_Pin };
 *   if (!lora_begin(&hw, 868000000)) Error_Handler();
 *   lora_send((uint8_t *)"hola", 4);
 *
 * Interrupt mode: configure DIO0 as EXTI rising, then in your
 * HAL_GPIO_EXTI_Callback() call lora_handle_dio0() for that pin.
 * Callbacks run in ISR context: no HAL_Delay(), no printf.
 */
#ifndef LORA_H
#define LORA_H

#include <stdint.h>

#ifdef LORA_HOST_TEST
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
#else
#include "main.h"   /* CubeMX header: pulls in the HAL for your family */
#endif

/* Knobs. Override with -D if your module differs. */
#ifndef LORA_XTAL_HZ
#define LORA_XTAL_HZ       32000000UL  /* trim if your crystal reads off frequency */
#endif
#ifndef LORA_SPI_TIMEOUT
#define LORA_SPI_TIMEOUT   100         /* ms, per transaction */
#endif
#ifndef LORA_TX_TIMEOUT_MS
#define LORA_TX_TIMEOUT_MS 10000       /* 255 B @ SF12/BW125 is ~9 s */
#endif

#define LORA_MAX_PKT_LENGTH        255
#define LORA_PA_OUTPUT_RFO_PIN     0
#define LORA_PA_OUTPUT_PA_BOOST    1

typedef struct {
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef      *nss_port;
  uint16_t           nss_pin;
  GPIO_TypeDef      *rst_port;   /* NULL if reset is not wired */
  uint16_t           rst_pin;
} lora_hw_t;

/* lifecycle */
int  lora_begin(const lora_hw_t *hw, uint32_t frequency_hz);  /* 1 = ok, 0 = no chip */
void lora_end(void);
void lora_idle(void);
void lora_sleep(void);

/* transmit */
int  lora_begin_packet(int implicit_header);                  /* 0 if still transmitting */
int  lora_write(const uint8_t *buf, uint16_t len);            /* bytes actually queued */
int  lora_end_packet(int async);                              /* 1 = ok, 0 = TX timeout */
int  lora_send(const uint8_t *buf, uint16_t len);             /* begin + write + end */

/* receive, polled */
int  lora_parse_packet(int size);                             /* payload length, 0 = none */
int  lora_available(void);
int  lora_read(void);                                         /* -1 when empty */
int  lora_peek(void);
int  lora_read_bytes(uint8_t *buf, uint16_t len);

/* receive, interrupt driven */
void lora_on_receive(void (*cb)(int packet_len));
void lora_on_tx_done(void (*cb)(void));
void lora_on_cad_done(void (*cb)(int detected));
void lora_receive(int size);                                  /* size 0 = explicit header */
void lora_cad(void);
void lora_handle_dio0(void);                                  /* call from your EXTI callback */

/* link quality */
int   lora_packet_rssi(void);
float lora_packet_snr(void);
long  lora_packet_frequency_error(void);
int   lora_rssi(void);

/* radio parameters (both ends must match) */
void lora_set_frequency(uint32_t hz);
void lora_set_tx_power(int level, int output_pin);            /* RFO 0..14, PA_BOOST 2..20 */
void lora_set_spreading_factor(int sf);                       /* 6..12 */
void lora_set_signal_bandwidth(uint32_t hz);                  /* 7800..500000 */
void lora_set_coding_rate4(int denominator);                  /* 5..8 */
void lora_set_preamble_length(uint16_t length);
void lora_set_sync_word(uint8_t sw);
void lora_set_crc(int on);
void lora_set_invert_iq(int on);
void lora_set_ldo(int on);                                    /* forced low data rate optimize */
void lora_set_ocp(uint8_t mA);
void lora_set_gain(uint8_t gain);                             /* 0 = AGC, 1..6 = manual LNA */
uint8_t lora_random(void);                                    /* wideband RSSI noise */

void lora_dump_registers(void (*out)(uint8_t addr, uint8_t value));

#endif /* LORA_H */
