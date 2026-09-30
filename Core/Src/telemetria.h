#ifndef TELEMETRIA_H
#define TELEMETRIA_H

#include <stdint.h>

#define LORA_FREQ_HZ  915000000UL   

typedef struct __attribute__((packed)) {
  uint32_t t_ms;
  float    Altitud;
  float    AccX;
  float    AccY;
  float    AccZ;
  float    GyrX;
  float    GyrY;
  float    GyrZ;
  float    MagX;
  float    MagY;
  float    MagZ;
} telemetria_t;

#endif