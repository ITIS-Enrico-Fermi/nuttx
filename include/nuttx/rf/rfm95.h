/****************************************************************************
 * include/nuttx/rf/rfm95.h
 * Character driver for the SPI RFM95 Radio module
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_RF_RFM95_H
#define __INCLUDE_NUTTX_RF_RFM95_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/rf/ioctl.h>
#include <nuttx/spi/spi.h>

/*
 * Register definitions
 */
#define REG_FIFO                       (0x00)
#define REG_OP_MODE                    (0x01)
#define REG_FRF_MSB                    (0x06)
#define REG_FRF_MID                    (0x07)
#define REG_FRF_LSB                    (0x08)
#define REG_PA_CONFIG                  (0x09)
#define REG_LNA                        (0x0c)
#define REG_FIFO_ADDR_PTR              (0x0d)
#define REG_FIFO_TX_BASE_ADDR          (0x0e)
#define REG_FIFO_RX_BASE_ADDR          (0x0f)
#define REG_FIFO_RX_CURRENT_ADDR       (0x10)
#define REG_IRQ_FLAGS                  (0x12)
#define REG_RX_NB_BYTES                (0x13)
#define REG_PKT_SNR_VALUE              (0x19)
#define REG_PKT_RSSI_VALUE             (0x1a)
#define REG_MODEM_CONFIG_1             (0x1d)
#define REG_MODEM_CONFIG_2             (0x1e)
#define REG_PREAMBLE_MSB               (0x20)
#define REG_PREAMBLE_LSB               (0x21)
#define REG_PAYLOAD_LENGTH             (0x22)
#define REG_MODEM_CONFIG_3             (0x26)
#define REG_RSSI_WIDEBAND              (0x2c)
#define REG_DETECTION_OPTIMIZE         (0x31)
#define REG_DETECTION_THRESHOLD        (0x37)
#define REG_SYNC_WORD                  (0x39)
#define REG_DIO_MAPPING_1              (0x40)
#define REG_VERSION                    (0x42)

/*
 * Transceiver modes
 */
#define MODE_LONG_RANGE_MODE           (0x80)
#define MODE_SLEEP                     (0x00)
#define MODE_STDBY                     (0x01)
#define MODE_TX                        (0x03)
#define MODE_RX_CONTINUOUS             (0x05)
#define MODE_RX_SINGLE                 (0x06)

/*
 * PA configuration
 */
#define PA_BOOST                       (0x80)

/*
 * IRQ masks
 */
#define IRQ_TX_DONE_MASK               (0x08)
#define IRQ_PAYLOAD_CRC_ERROR_MASK     (0x20)
#define IRQ_RX_DONE_MASK               (0x40)

#define PA_OUTPUT_RFO_PIN              (0)
#define PA_OUTPUT_PA_BOOST_PIN         (1)

#define TIMEOUT_RESET                  (100)

/* IOCTL DEFINITION */
#define RFM95_IOCTL_RESET 0

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void rfm95_reset(void);
void rfm95_explicit_header_mode(void);
void rfm95_implicit_header_mode(int size);
void rfm95_idle(void);
void rfm95_sleep(void); 
void rfm95_receive(void);
void rfm95_set_tx_power(int level);
void rfm95_set_frequency(long frequency);
void rfm95_set_spreading_factor(int sf);
void rfm95_set_bandwidth(long sbw);
void rfm95_set_coding_rate(int denominator);
void rfm95_set_preamble_length(long length);
void rfm95_set_sync_word(int sw);
void rfm95_enable_crc(void);
void rfm95_disable_crc(void);
int rfm95_init(const char *dev_path);
void rfm95_send_packet(const uint8_t *buf, int size);
int rfm95_receive_packet(uint8_t *buf, int size);
int rfm95_received(void);
int rfm95_packet_rssi(void);
float rfm95_packet_snr(void);
void rfm95_close(void);
int rfm95_initialized(void);
void rfm95_dump_registers(void);

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: rfm95_register
 *
 * Description:
 *   Register the rfm95 character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/radio0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *   spidev  - Number of the spi device (used to drive the Latch Enable pin).
 *
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rfm95_register(FAR const char *devpath,
                       FAR struct spi_dev_s *spi,
                       int spidev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_RF_RFM95_H */
