

#include "kalman.h"


void Kalman_Init(KalmanVertical_t *kf)
{
    kf->altura    = 0.0f;   // [m]
    kf->velocidad = 0.0f;   // [m/s]
    kf->bias_acc  = 0.0f;   // [m/s^2]


    // Matriz de covarianza inicial P
    //        altura   velocidad   bias

    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[0][2] = 0.0f;

    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
    kf->P[1][2] = 0.0f;

    kf->P[2][0] = 0.0f;
    kf->P[2][1] = 0.0f;
    kf->P[2][2] = 0.1f;


    //parametros de ruido
    kf->sigma_acc  = 0.5f;   // [m/s^2]
    kf->sigma_bias = 0.01f;  // variacion del bias
    kf->R_baro     = 1.0f;   // varianza de altura [m^2]
}



//---------Prediccion se ejecuta cada lectura del MPU ----------------
void Kalman_Predict(KalmanVertical_t *kf,
                    float acc_vertical,
                    float dt)
{
    float dt2 = dt * dt;

    // Aceleracion corregida por el bias estimado
    float acc = acc_vertical - kf->bias_acc;


    //=====================================================================
    // 1. Prediccion del estado
    //
    // h(k+1) = h + v*dt + 1/2*a*dt^2
    //
    // v(k+1) = v + a*dt
    //
    // bias(k+1) = bias
    //=====================================================================

    kf->altura +=kf->velocidad * dt + 0.5f * acc * dt2;
    kf->velocidad += acc * dt;


    //=====================================================================
    // 2. Matriz de transicion F
    //
    // Estado:
    //
    // x = [ altura
    //       velocidad
    //       bias ]
    //
    //=====================================================================

    float F[3][3] =
    {
        {1.0f, dt,   -0.5f * dt2},
        {0.0f, 1.0f, -dt},
        {0.0f, 0.0f,  1.0f}
    };


    //=====================================================================
    // 3. Matriz de ruido Q
    //=====================================================================

    float sigma_acc2 =
            kf->sigma_acc * kf->sigma_acc;

    float sigma_bias2 =
            kf->sigma_bias * kf->sigma_bias;


    float G0 = 0.5f * dt2;
    float G1 = dt;


    float Q[3][3] =
    {
        {
            G0 * G0 * sigma_acc2,
            G0 * G1 * sigma_acc2,
            0.0f
        },

        {
            G1 * G0 * sigma_acc2,
            G1 * G1 * sigma_acc2,
            0.0f
        },

        {
            0.0f,
            0.0f,
            sigma_bias2 * dt
        }
    };


    //=====================================================================
    // 4. Actualizacion de covarianza
    //
    // P = F * P * F^T + Q
    //=====================================================================

    float FP[3][3] = {0};
    float P_new[3][3] = {0};


    // FP = F * P
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            for(int k = 0; k < 3; k++)
            {
                FP[i][j] +=
                        F[i][k] * kf->P[k][j];
            }
        }
    }


    // P_new = FP * F^T + Q
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            for(int k = 0; k < 3; k++)
            {
                // F^T[k][j] = F[j][k]
                P_new[i][j] +=
                        FP[i][k] * F[j][k];
            }

            P_new[i][j] += Q[i][j];
        }
    }


    // Guardar nueva matriz P
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            kf->P[i][j] = P_new[i][j];
        }
    }
}


//=========================================================================
// CORRECCION CON BMP280
//
// El BMP280 solamente mide altura.
//
// Medicion:
//
// z = altura_bmp
//
// H = [1  0  0]
//
//=========================================================================

void Kalman_UpdateBaro(KalmanVertical_t *kf,
                       float altura_bmp)
{
    //=====================================================================
    // 1. Innovacion
    //
    // diferencia entre lo que mide BMP280
    // y lo que predijo el Kalman
    //=====================================================================

    float innovacion =
            altura_bmp - kf->altura;


    //=====================================================================
    // 2. Covarianza de la innovacion
    //
    // S = HPH^T + R
    //
    // Como H = [1 0 0]:
    //
    // S = P00 + R
    //=====================================================================

    float S =
            kf->P[0][0] + kf->R_baro;


    // Evitar division entre cero
    if(S <= 0.0f)
    {
        return;
    }


    //=====================================================================
    // 3. Ganancia de Kalman
    //
    // K = P H^T / S
    //=====================================================================

    float K[3];

    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    K[2] = kf->P[2][0] / S;


    //=====================================================================
    // 4. Correccion de los estados
    //=====================================================================

    kf->altura +=
            K[0] * innovacion;

    kf->velocidad +=
            K[1] * innovacion;

    kf->bias_acc +=
            K[2] * innovacion;


    //=====================================================================
    // 5. Actualizacion de la covarianza
    //
    // Forma de Joseph:
    //
    // P = (I-KH) P (I-KH)^T + K R K^T
    //
    // Es un poco mas costosa, pero numericamente mas estable.
    //=====================================================================

    float A[3][3] =
    {
        {1.0f - K[0], 0.0f, 0.0f},
        {-K[1],       1.0f, 0.0f},
        {-K[2],       0.0f, 1.0f}
    };


    float AP[3][3] = {0};
    float P_new[3][3] = {0};


    // AP = A * P
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            for(int k = 0; k < 3; k++)
            {
                AP[i][j] +=
                        A[i][k] * kf->P[k][j];
            }
        }
    }


    // P_new = AP * A^T
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            for(int k = 0; k < 3; k++)
            {
                P_new[i][j] +=
                        AP[i][k] * A[j][k];
            }

            // + K R K^T
            P_new[i][j] +=
                    K[i] *
                    kf->R_baro *
                    K[j];
        }
    }


    // Guardar nueva matriz P
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            kf->P[i][j] = P_new[i][j];
        }
    }
}
