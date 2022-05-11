/*
 * aiq_PMSA003I.h
 *
 *  Created on: May 11, 2022
 *      Author: silvia
 */

#ifndef SRC_SENSORS_AIQ_PMSA003I_I2C_H_
#define SRC_SENSORS_AIQ_PMSA003I_I2C_H_

#include <stdint.h>

#define PMSA003I_I2CADDR 0x24  // vendor address 0x12 left shifted by 1 bit

typedef struct PMSAQIdata {
  uint16_t framelen;       ///< How long this data chunk is
  uint16_t pm10_standard,  ///< Standard PM1.0
      pm25_standard,       ///< Standard PM2.5
      pm100_standard;      ///< Standard PM10.0
  uint16_t pm10_env,       ///< Environmental PM1.0
      pm25_env,            ///< Environmental PM2.5
      pm100_env;           ///< Environmental PM10.0
  uint16_t particles_03um, ///< 0.3um Particle Count
      particles_05um,      ///< 0.5um Particle Count
      particles_10um,      ///< 1.0um Particle Count
      particles_25um,      ///< 2.5um Particle Count
      particles_50um,      ///< 5.0um Particle Count
      particles_100um;     ///< 10.0um Particle Count
  uint16_t unused;         ///< Unused
  uint16_t checksum;       ///< Packet checksum
} PM25_AQI_Data;

uint8_t aiq_PMSA003I_i2c_read(PM25_AQI_Data *data);

#endif /* SRC_SENSORS_AIQ_PMSA003I_I2C_H_ */
