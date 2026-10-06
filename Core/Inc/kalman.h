/*
 * kalman.h
 *
 *  Created on: 30 sept 2026
 *      Author: Usuario
 */

#ifndef COMPUTADORA_DE_VUELO_SOLIDO_2026_2027_CORE_INC_KALMAN_H_
#define COMPUTADORA_DE_VUELO_SOLIDO_2026_2027_CORE_INC_KALMAN_H_

typedef struct
{
    float altura;  // [m]
    float velocidad; // [m/s]
    float bias_acc; // [m/s^2]
    float P[3][3];

    float sigma_acc;     // ruido acelerometro
    float sigma_bias;    // variacion del bias
    float R_baro;        // ruido del BMP280

} KalmanVertical_t;

// Inicializar filtro
void Kalman_Init(KalmanVertical_t *kf);

// Prediccion usando aceleracion vertical
void Kalman_Predict(
        KalmanVertical_t *kf,
        float acc_vertical,
        float dt);

// Correccion usando altura del BMP280
void Kalman_UpdateBaro(
        KalmanVertical_t *kf,
        float altura_bmp);


#endif

