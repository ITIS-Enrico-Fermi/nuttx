/****************************************************************************
 * drivers/rf/rfm95.c
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

/* Custom SPI RFM95 Driver */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/rf/ioctl.h>
#include <nuttx/rf/attenuator.h>
#include <nuttx/rf/rfm95.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SPI) && defined(CONFIG_RF_RFM95)

/* We set SPI Frequency to 1 MHz */

#ifndef CONFIG_RF_RFM95_SPI_FREQUENCY
// We can push it to 9MHz, and maybe faster
#  define CONFIG_RF_RFM95_SPI_FREQUENCY 9000000
#endif /* CONFIG_RFM95_SPI_FREQUENCY */

#ifndef CONFIG_RF_RFM95_RESET_PIN
#define CONFIG_RF_RFM95_RESET_PIN 10
//TODO: check pin correctness
#endif

#  define RFM95_SPI_MODE (SPIDEV_MODE0) /* SPI Mode 0: CPOL=0,CPHA=0 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rfm95_dev_s
{
  FAR struct spi_dev_s *spi;    /* Saved SPI driver instance */
  int spidev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int rfm95_open(FAR struct file *filep);
static int rfm95_close(FAR struct file *filep);
static ssize_t rfm95_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t rfm95_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int rfm95_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_rfm95_fops =
{
  rfm95_open,
  rfm95_close,
  rfm95_read,
  rfm95_write,
  NULL,  /* Seek not implemented */
  rfm95_ioctl,
  NULL   /* Poll not implemented */
};

static char recv_buffer[256];  /* Buffer for SPI response */

static int recv_buffer_len = 0;  /* Length of SPI response */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Low level functions to expose the underlying SPI bus to rfm95_logic.c */

/**
 * Sends a reset signal down the RST GPIO pin.
*/
static void rfm95_reset() {
  board_gpio_write(CONFIG_RF_RFM95_RESET_PIN, 0);
  up_mdelay(1);
  board_gpio_write(CONFIG_RF_RFM95_RESET_PIN, 1);
  up_mdelay(10);
}

/****************************************************************************
 * Name: rfm95_configspi
 *
 * Description:
 *   Configure the SPI instance
 *
 ****************************************************************************/

static inline void rfm95_configspi(FAR struct spi_dev_s *spi)
{
  _info("\n");
  DEBUGASSERT(spi != NULL);

  /* Set SPI Mode (Polarity and Phase) and Transfer Size (8 bits) */

  SPI_SETMODE(spi, RFM95_SPI_MODE);
  SPI_SETBITS(spi, 8);

  /* Set SPI Hardware Features and Frequency */

  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_RF_RFM95_SPI_FREQUENCY);

  // Configure RESET pin
  board_gpio_config(CONFIG_RF_RFM95_RESET_PIN, 0, false, false, PIN_FLOAT);
}

/****************************************************************************
 * Name: rfm95_open
 *
 * Description:
 *   This function is called whenever the device is opened.
 *
 ****************************************************************************/

static int rfm95_open(FAR struct file *filep)
{
  _info("\n");
  DEBUGASSERT(filep != NULL);

  /* Check if kernel configs work! */
  sninfo("SPI freq: %d\n", CONFIG_RF_RFM95_SPI_FREQUENCY);
  sninfo("SPI freq: %d\n", CONFIG_RF_RFM95_RESET_PIN);

  /* Get the SPI interface */

  FAR struct inode *inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  FAR struct rfm95_dev_s *priv = inode->i_private;
  DEBUGASSERT(priv != NULL);

  SPI_LOCK(priv->spi, true);
  rfm95_configspi(priv->spi);
  SPI_LOCK(priv->spi, false); // closing lock is important :)

  rfm95_reset();

  return OK;
}

/****************************************************************************
 * Name: rfm95_close
 *
 * Description:
 *   This function is called whenever the device is closed.
 *
 ****************************************************************************/

static int rfm95_close(FAR struct file *filep)
{
  _info("\n");
  DEBUGASSERT(filep != NULL);
  return OK;
}

/****************************************************************************
 * Name: rfm95_write
 *
 * Description:
 *   Write the buffer to the device.
 ****************************************************************************/

static ssize_t rfm95_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t buflen)
{
  _info("buflen=%u\n", buflen);
  DEBUGASSERT(buflen <= sizeof(recv_buffer));  /* TODO: Range eheck */
  DEBUGASSERT(buffer != NULL);
  DEBUGASSERT(filep  != NULL);

  /* Get the SPI interface */

  FAR struct inode *inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  FAR struct rfm95_dev_s *priv = inode->i_private;
  DEBUGASSERT(priv != NULL);

  /* Lock the SPI bus */
  
  DEBUGASSERT(priv->spi != NULL);
  SPI_LOCK(priv->spi, true);
  //CHECK: moved SPI initialization to rfm95_open

  /* Assert CS pin of the module */

  SPI_SELECT(priv->spi, priv->spidev, true);

  /* Transmit buffer to SPI device and receive the response */

  SPI_EXCHANGE(priv->spi, buffer, recv_buffer, buflen);
  recv_buffer_len = buflen;

  /* Deassert CS pin of the module */

  SPI_SELECT(priv->spi, priv->spidev, false);

  /* Unlock the SPI bus */

  SPI_LOCK(priv->spi, false);

  return buflen;
}

/****************************************************************************
 * Name: rfm95_read
 *
 * Description:
 *   Return the data received from the device.
 ****************************************************************************/

static ssize_t rfm95_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  _info("buflen=%u\n", buflen);
  DEBUGASSERT(filep  != NULL);
  DEBUGASSERT(buffer != NULL);

  /* Copy the SPI response to the buffer */

  DEBUGASSERT(recv_buffer_len >= 0);
  DEBUGASSERT(recv_buffer_len <= buflen);  /* TODO: Range check */
  memcpy(buffer, recv_buffer, recv_buffer_len);

  /* Return the number of bytes read */

  return recv_buffer_len;
}

/****************************************************************************
 * Name: rfm95_ioctl
 *
 * Description:
 *   Execute ioctl commands for the device.
 ****************************************************************************/

static int rfm95_ioctl(FAR struct file *filep,
                           int cmd,
                           unsigned long arg)
{
  _info("cmd=0x%x, arg=0x%lx\n", cmd, arg);
  DEBUGASSERT(filep != NULL);

  int ret = OK;

  switch (cmd)
    {
      case RFM95_IOCTL_RESET:
        rfm95_reset();
        break;

      default:
        sninfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rfm95_register
 *
 * Description:
 *   Register the rfm95 character device as 'devpath' during NuttX startup.
 *
 ****************************************************************************/

int rfm95_register(FAR const char *devpath,
                       FAR struct spi_dev_s *spi,
                       int spidev)
{
  _info("devpath=%s, spidev=%d\n", devpath, spidev);
  FAR struct rfm95_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(spi != NULL);

  /* Initialize the device structure */

  priv = (FAR struct rfm95_dev_s *)
      kmm_malloc(sizeof(struct rfm95_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi    = spi;
  priv->spidev = spidev;

  /* Clear the LE pin */

  SPI_SELECT(priv->spi, priv->spidev, false);

  /* Register the character driver */

  ret = register_driver(devpath, &g_rfm95_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif