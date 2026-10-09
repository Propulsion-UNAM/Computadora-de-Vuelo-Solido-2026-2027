#ifndef TELEMETRIA_H
#define TELEMETRIA_H

#include <stdint.h>


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
  float    VelX;
  float    Presion;
  int 	   confirmacion_despegue;
} telemetria_t;

#endif
