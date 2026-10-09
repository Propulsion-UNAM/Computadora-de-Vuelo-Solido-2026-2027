

#include "kalman.h"


void Kalman_Init(KalmanVertical_t *kf)
{
    kf->altura    = 0.0f;   // [m]
    kf->velocidad = 0.0f;   // [m/s]
    kf->bias_acc  = 0.0f;   // [m/s^2]

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
    kf->sigma_acc  = 0.3f;   // [m/s^2]
    kf->sigma_bias = 0.01f;  // variacion del bias
    kf->R_baro     = 2.0f;   // varianza de altura [m^2]
}

//---------Prediccion se ejecuta cada lectura del MPU ----------------
void Kalman_Predict(KalmanVertical_t *kf,float acc_vertical,float dt)
{
    float dt2 = dt * dt;

    // Aceleracion corregida por el bias estimado
    float acc = acc_vertical - kf->bias_acc;

    kf->altura +=kf->velocidad * dt + 0.5f * acc * dt2;
    kf->velocidad += acc * dt;

    float F[3][3] =
    {
        {1.0f, dt,   -0.5f * dt2},
        {0.0f, 1.0f, -dt},
        {0.0f, 0.0f,  1.0f}
    };

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

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            kf->P[i][j] = P_new[i][j];
        }
    }
}

void Kalman_UpdateBaro(KalmanVertical_t *kf,float altura_bmp){

    float innovacion =altura_bmp - kf->altura;

    float S =kf->P[0][0] + kf->R_baro;

    if(S <= 0.0f){
        return;
    }

    float K[3];

    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    K[2] = kf->P[2][0] / S;

    kf->altura +=
            K[0] * innovacion;

    kf->velocidad +=
            K[1] * innovacion;

    kf->bias_acc +=
            K[2] * innovacion;


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

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            kf->P[i][j] = P_new[i][j];
        }
    }
}
