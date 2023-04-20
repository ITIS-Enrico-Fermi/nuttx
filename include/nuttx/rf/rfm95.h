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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
 *   devpath - The full path to the driver to register. E.g., "/dev/att0"
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
