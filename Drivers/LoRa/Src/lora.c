/* SX1276/77/78/79 driver for STM32. Port of sandeepmistry/arduino-LoRa, MIT. */

#include "lora.h"
#include <stddef.h>

/* registers */
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

/* modes */
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06
#define MODE_CAD                 0x07

#define PA_BOOST                 0x80

#define IRQ_CAD_DETECTED_MASK      0x01
#define IRQ_CAD_DONE_MASK          0x04
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define RF_MID_BAND_THRESHOLD    525000000UL
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

static struct {
  lora_hw_t hw;
  uint32_t  frequency;
  int       packet_index;
  int       packet_len;
  int       implicit_header;
  void (*on_receive)(int);
  void (*on_tx_done)(void);
  void (*on_cad_done)(int);
} lora;

/* ---- hardware port: the only MCU-specific code. Swap these for LL/bare metal. ---- */
#ifdef LORA_HOST_TEST
void     lora_port_xfer(const uint8_t *tx, uint8_t *rx, uint16_t n);
uint32_t lora_port_tick(void);
void     lora_port_delay(uint32_t ms);
void     lora_port_nss(int high);
void     lora_port_reset(int high);
#else
static void lora_port_xfer(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
  HAL_SPI_TransmitReceive(lora.hw.spi, (uint8_t *)tx, rx, n, LORA_SPI_TIMEOUT);
}
static uint32_t lora_port_tick(void)      { return HAL_GetTick(); }
static void lora_port_delay(uint32_t ms)  { HAL_Delay(ms); }
static void lora_port_nss(int high)
{
  HAL_GPIO_WritePin(lora.hw.nss_port, lora.hw.nss_pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void lora_port_reset(int high)
{
  if (lora.hw.rst_port)
    HAL_GPIO_WritePin(lora.hw.rst_port, lora.hw.rst_pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
#endif

/* ---- register access ---- */

static uint8_t read_reg(uint8_t address)
{
  uint8_t tx[2] = { (uint8_t)(address & 0x7f), 0x00 }, rx[2] = { 0, 0 };
  lora_port_nss(0);
  lora_port_xfer(tx, rx, 2);
  lora_port_nss(1);
  return rx[1];
}

static void write_reg(uint8_t address, uint8_t value)
{
  uint8_t tx[2] = { (uint8_t)(address | 0x80), value }, rx[2];
  lora_port_nss(0);
  lora_port_xfer(tx, rx, 2);
  lora_port_nss(1);
}

/* FIFO is auto-incrementing, so a whole payload moves in one transaction. */
static void fifo_burst(uint8_t address, const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)
{
  uint8_t addr = address;
  uint8_t scratch;
  lora_port_nss(0);
  lora_port_xfer(&addr, &scratch, 1);
  for (uint16_t i = 0; i < len; i++) {
    uint8_t b = tx_buf ? tx_buf[i] : 0x00;
    uint8_t r = 0;
    lora_port_xfer(&b, &r, 1);
    if (rx_buf) rx_buf[i] = r;
  }
  lora_port_nss(1);
}

/* ---- header mode ---- */

static void explicit_header_mode(void)
{
  lora.implicit_header = 0;
  write_reg(REG_MODEM_CONFIG_1, read_reg(REG_MODEM_CONFIG_1) & 0xfe);
}

static void implicit_header_mode(void)
{
  lora.implicit_header = 1;
  write_reg(REG_MODEM_CONFIG_1, read_reg(REG_MODEM_CONFIG_1) | 0x01);
}

/* ---- bandwidth table, index == REG_MODEM_CONFIG_1[7:4] ---- */

static const uint32_t bw_table[10] = {
  7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000
};

static uint32_t get_signal_bandwidth(void)
{
  uint8_t bw = read_reg(REG_MODEM_CONFIG_1) >> 4;
  return (bw < 10) ? bw_table[bw] : 0;
}

static int get_spreading_factor(void)
{
  return read_reg(REG_MODEM_CONFIG_2) >> 4;
}

static void set_ldo_bit(int on)
{
  uint8_t cfg3 = read_reg(REG_MODEM_CONFIG_3);
  cfg3 = on ? (cfg3 | 0x08) : (cfg3 & ~0x08);
  write_reg(REG_MODEM_CONFIG_3, cfg3);
}

/* Datasheet 4.1.1.5/4.1.1.6: LDO on when a symbol lasts more than 16 ms. */
static void update_ldo(void)
{
  uint32_t bw = get_signal_bandwidth();
  if (!bw) return;
  uint32_t symbol_ms = (1000UL << get_spreading_factor()) / bw;
  set_ldo_bit(symbol_ms > 16);
}

/* ---- lifecycle ---- */

int lora_begin(const lora_hw_t *hw, uint32_t frequency_hz)
{
  lora.hw = *hw;
  lora.packet_index = 0;
  lora.packet_len = 0;
  lora.implicit_header = 0;
  lora.on_receive = NULL;
  lora.on_tx_done = NULL;
  lora.on_cad_done = NULL;

  lora_port_nss(1);

  if (lora.hw.rst_port) {
    lora_port_reset(0);
    lora_port_delay(10);
    lora_port_reset(1);
    lora_port_delay(10);
  }

  if (read_reg(REG_VERSION) != 0x12)
    return 0;                       /* wrong wiring, wrong chip, or SPI not running */

  lora_sleep();
  lora_set_frequency(frequency_hz);

  write_reg(REG_FIFO_TX_BASE_ADDR, 0);
  write_reg(REG_FIFO_RX_BASE_ADDR, 0);
  write_reg(REG_LNA, read_reg(REG_LNA) | 0x03);   /* LNA boost */
  write_reg(REG_MODEM_CONFIG_3, 0x04);            /* auto AGC */
  lora_set_tx_power(17, LORA_PA_OUTPUT_PA_BOOST);
  lora_idle();

  return 1;
}

void lora_end(void)
{
  lora_sleep();
}

void lora_idle(void)
{
  write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void lora_sleep(void)
{
  write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/* ---- transmit ---- */

static int is_transmitting(void)
{
  if ((read_reg(REG_OP_MODE) & MODE_TX) == MODE_TX)
    return 1;
  if (read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)
    write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  return 0;
}

int lora_begin_packet(int implicit_header)
{
  if (is_transmitting())
    return 0;

  lora_idle();

  if (implicit_header) implicit_header_mode();
  else                 explicit_header_mode();

  write_reg(REG_FIFO_ADDR_PTR, 0);
  write_reg(REG_PAYLOAD_LENGTH, 0);
  return 1;
}

int lora_write(const uint8_t *buf, uint16_t len)
{
  int current = read_reg(REG_PAYLOAD_LENGTH);

  if (current + len > LORA_MAX_PKT_LENGTH)
    len = (uint16_t)(LORA_MAX_PKT_LENGTH - current);

  fifo_burst(REG_FIFO | 0x80, buf, NULL, len);
  write_reg(REG_PAYLOAD_LENGTH, (uint8_t)(current + len));
  return len;
}

int lora_end_packet(int async)
{
  if (async && lora.on_tx_done)
    write_reg(REG_DIO_MAPPING_1, 0x40);           /* DIO0 => TxDone */

  write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  if (async)
    return 1;

  uint32_t start = lora_port_tick();
  while ((read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
    if (lora_port_tick() - start > LORA_TX_TIMEOUT_MS) {
      lora_idle();
      return 0;                                   /* radio wedged; caller decides */
    }
  }
  write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  return 1;
}

int lora_send(const uint8_t *buf, uint16_t len)
{
  if (!lora_begin_packet(0))
    return 0;
  lora_write(buf, len);
  return lora_end_packet(0);
}

/* ---- receive ---- */

int lora_parse_packet(int size)
{
  int packet_length = 0;
  uint8_t irq_flags = read_reg(REG_IRQ_FLAGS);

  if (size > 0) {
    implicit_header_mode();
    write_reg(REG_PAYLOAD_LENGTH, (uint8_t)size);
  } else {
    explicit_header_mode();
  }

  write_reg(REG_IRQ_FLAGS, irq_flags);

  if ((irq_flags & IRQ_RX_DONE_MASK) && !(irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK)) {
    lora.packet_index = 0;
    packet_length = lora.implicit_header ? read_reg(REG_PAYLOAD_LENGTH)
                                         : read_reg(REG_RX_NB_BYTES);
    lora.packet_len = packet_length;
    write_reg(REG_FIFO_ADDR_PTR, read_reg(REG_FIFO_RX_CURRENT_ADDR));
    lora_idle();
  } else if (read_reg(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    write_reg(REG_FIFO_ADDR_PTR, 0);
    write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packet_length;
}

int lora_available(void)
{
  return lora.packet_len - lora.packet_index;
}

int lora_read(void)
{
  if (!lora_available())
    return -1;
  lora.packet_index++;
  return read_reg(REG_FIFO);
}

int lora_read_bytes(uint8_t *buf, uint16_t len)
{
  int avail = lora_available();
  if (len > (uint16_t)avail)
    len = (uint16_t)avail;
  fifo_burst(REG_FIFO, NULL, buf, len);
  lora.packet_index += len;
  return len;
}

int lora_peek(void)
{
  if (!lora_available())
    return -1;
  uint8_t addr = read_reg(REG_FIFO_ADDR_PTR);
  uint8_t b = read_reg(REG_FIFO);
  write_reg(REG_FIFO_ADDR_PTR, addr);
  return b;
}

void lora_receive(int size)
{
  write_reg(REG_DIO_MAPPING_1, 0x00);             /* DIO0 => RxDone */

  if (size > 0) {
    implicit_header_mode();
    write_reg(REG_PAYLOAD_LENGTH, (uint8_t)size);
  } else {
    explicit_header_mode();
  }

  write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void lora_cad(void)
{
  write_reg(REG_DIO_MAPPING_1, 0x80);             /* DIO0 => CadDone */
  write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
}

void lora_on_receive(void (*cb)(int))  { lora.on_receive = cb; }
void lora_on_tx_done(void (*cb)(void)) { lora.on_tx_done = cb; }
void lora_on_cad_done(void (*cb)(int)) { lora.on_cad_done = cb; }

void lora_handle_dio0(void)
{
  uint8_t irq_flags = read_reg(REG_IRQ_FLAGS);
  write_reg(REG_IRQ_FLAGS, irq_flags);

  if (irq_flags & IRQ_CAD_DONE_MASK) {
    if (lora.on_cad_done)
      lora.on_cad_done((irq_flags & IRQ_CAD_DETECTED_MASK) != 0);
  } else if (!(irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK)) {
    if (irq_flags & IRQ_RX_DONE_MASK) {
      lora.packet_index = 0;
      lora.packet_len = lora.implicit_header ? read_reg(REG_PAYLOAD_LENGTH)
                                             : read_reg(REG_RX_NB_BYTES);
      write_reg(REG_FIFO_ADDR_PTR, read_reg(REG_FIFO_RX_CURRENT_ADDR));
      if (lora.on_receive)
        lora.on_receive(lora.packet_len);
    } else if (irq_flags & IRQ_TX_DONE_MASK) {
      if (lora.on_tx_done)
        lora.on_tx_done();
    }
  }
}

/* ---- link quality ---- */

static int rssi_offset(void)
{
  return lora.frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT;
}

int lora_packet_rssi(void) { return read_reg(REG_PKT_RSSI_VALUE) - rssi_offset(); }
int lora_rssi(void)        { return read_reg(REG_RSSI_VALUE) - rssi_offset(); }

float lora_packet_snr(void)
{
  return (float)(int8_t)read_reg(REG_PKT_SNR_VALUE) * 0.25f;
}

long lora_packet_frequency_error(void)
{
  int32_t err = (int32_t)(read_reg(REG_FREQ_ERROR_MSB) & 0x07);
  err = (err << 8) + read_reg(REG_FREQ_ERROR_MID);
  err = (err << 8) + read_reg(REG_FREQ_ERROR_LSB);

  if (read_reg(REG_FREQ_ERROR_MSB) & 0x08)        /* sign bit */
    err -= 524288;

  /* datasheet p.37, integer math: err * 2^24 / Fxtal * bw / 500000 */
  int64_t hz = ((int64_t)err << 24) / (int64_t)LORA_XTAL_HZ;
  return (long)(hz * (int64_t)get_signal_bandwidth() / 500000);
}

/* ---- radio parameters ---- */

void lora_set_frequency(uint32_t hz)
{
  lora.frequency = hz;
  uint32_t frf = (uint32_t)(((uint64_t)hz << 19) / LORA_XTAL_HZ);

  write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
  write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
  write_reg(REG_FRF_LSB, (uint8_t)frf);
}

void lora_set_tx_power(int level, int output_pin)
{
  if (output_pin == LORA_PA_OUTPUT_RFO_PIN) {
    if (level < 0)       level = 0;
    else if (level > 14) level = 14;
    write_reg(REG_PA_CONFIG, (uint8_t)(0x70 | level));
    return;
  }

  if (level > 17) {
    if (level > 20) level = 20;
    level -= 3;                                   /* 18..20 dBm maps to 15..17 */
    write_reg(REG_PA_DAC, 0x87);                  /* +20 dBm mode, datasheet 5.4.3 */
    lora_set_ocp(140);
  } else {
    if (level < 2) level = 2;
    write_reg(REG_PA_DAC, 0x84);
    lora_set_ocp(100);
  }
  write_reg(REG_PA_CONFIG, (uint8_t)(PA_BOOST | (level - 2)));
}

void lora_set_spreading_factor(int sf)
{
  if (sf < 6)       sf = 6;
  else if (sf > 12) sf = 12;

  if (sf == 6) {
    write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
    write_reg(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
    write_reg(REG_DETECTION_THRESHOLD, 0x0a);
  }

  write_reg(REG_MODEM_CONFIG_2, (read_reg(REG_MODEM_CONFIG_2) & 0x0f) | (uint8_t)((sf << 4) & 0xf0));
  update_ldo();
}

void lora_set_signal_bandwidth(uint32_t hz)
{
  int bw = 9;
  for (int i = 0; i < 10; i++) {
    if (hz <= bw_table[i]) { bw = i; break; }
  }
  write_reg(REG_MODEM_CONFIG_1, (read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (uint8_t)(bw << 4));
  update_ldo();
}

void lora_set_coding_rate4(int denominator)
{
  if (denominator < 5)      denominator = 5;
  else if (denominator > 8) denominator = 8;

  write_reg(REG_MODEM_CONFIG_1,
            (read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (uint8_t)((denominator - 4) << 1));
}

void lora_set_preamble_length(uint16_t length)
{
  write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  write_reg(REG_PREAMBLE_LSB, (uint8_t)length);
}

void lora_set_sync_word(uint8_t sw)
{
  write_reg(REG_SYNC_WORD, sw);
}

void lora_set_crc(int on)
{
  uint8_t cfg2 = read_reg(REG_MODEM_CONFIG_2);
  write_reg(REG_MODEM_CONFIG_2, on ? (cfg2 | 0x04) : (cfg2 & 0xfb));
}

void lora_set_invert_iq(int on)
{
  write_reg(REG_INVERTIQ,  on ? 0x66 : 0x27);
  write_reg(REG_INVERTIQ2, on ? 0x19 : 0x1d);
}

void lora_set_ldo(int on)
{
  set_ldo_bit(on);
}

void lora_set_ocp(uint8_t mA)
{
  uint8_t trim = 27;

  if (mA <= 120)      trim = (uint8_t)((mA - 45) / 5);
  else if (mA <= 240) trim = (uint8_t)((mA + 30) / 10);

  write_reg(REG_OCP, (uint8_t)(0x20 | (trim & 0x1f)));
}

void lora_set_gain(uint8_t gain)
{
  if (gain > 6) gain = 6;

  lora_idle();

  if (gain == 0) {
    write_reg(REG_MODEM_CONFIG_3, 0x04);          /* AGC on */
  } else {
    write_reg(REG_MODEM_CONFIG_3, 0x00);          /* AGC off */
    write_reg(REG_LNA, 0x03);                     /* clear gain, keep LNA boost */
    write_reg(REG_LNA, read_reg(REG_LNA) | (uint8_t)(gain << 5));
  }
}

uint8_t lora_random(void)
{
  return read_reg(REG_RSSI_WIDEBAND);
}

void lora_dump_registers(void (*out)(uint8_t addr, uint8_t value))
{
  for (uint8_t i = 0; i < 128; i++)
    out(i, read_reg(i));
}
