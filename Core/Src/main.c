/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora.h"
#include "telemetria.h"
#include "bmp280.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sd_functions.h"
#include "HMC5883L.h"
#include "telemetria.h"
#include "BMI270.h"
#include "filter.h"
#include "Madgwick_filter.h"
#include "lora.h"
#include "telemetria.h"
#include "kalman.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	ESTADO_LAUNCHPAD = 0,
	ESTADO_ASCENSO = 1,
	ESTADO_APOGEO = 2, //activación de primera etapa de recuperación
	ESTADO_REEFING_LINE = 3, //activación de segunda etapa de recuperación
	ESTADO_ATERRIZAJE = 4
} EstadoVuelo;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//---------Valores de altura predeterminadas para seguridad
#define ALTURA_ERROR_BMP 10
#define ALTURA_DESPEGUE 10
#define ALTURA_REEFING_LINE 500

//----------------------Constantes --------------------
#define presion_nivel_mar (101325.0f) //Este valor esta en pascales

//---------definir los pines I2C para recuperación --------------
#define I2C_RECOV_SCL_PORT  GPIOB
#define I2C_RECOV_SCL_PIN   GPIO_PIN_8

#define I2C_RECOV_SDA_PORT  GPIOB
#define I2C_RECOV_SDA_PIN   GPIO_PIN_9


//-------------definición para calibración de magnetometro------------
#define OFFSET_X 0
#define OFFSET_Y 0
#define OFFSET_Z 0

//------------Prueba aspiradora ---------------------
#define aspiradora 1

//------------Calibración de sensores --------------

//Placa 2
//----------Calibración ------------------------
#define BIAS_ACC_X_1   (0.0f)
#define BIAS_ACC_Y_1   (0.0f)
#define BIAS_ACC_Z_1   ( 0.0f)

#define BIAS_ACC_X_2   (-0.3216f)
#define BIAS_ACC_Y_2   (-1.0885f)
#define BIAS_ACC_Z_2   ( 0.6007f)


#define FRECUENCIA_MPU (0.003333333f)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim12;

/* USER CODE BEGIN PV */

//---lecturas bmp280---------
float temperatura = 0.0f;
float presion = 0.0f;
float altura = 0.0f;

//------------------Calibración inicial en el main  --------------------
float presion_acumulacion = 0.0f;
float presion_calibrada_suelo = 0.0f;
int cont_calibracion_bmp;

//----Variables para calcular altitud --------

float altura_nivel_mar = 0.0f; //Es la altura en el suelo
float altura_maxima = 0.0f;
float altura_anterior=0.0f;

//------Variables para estados -----------------
uint32_t tiempo_canal_etapa_1 = 0;
uint32_t tiempo_canal_etapa_2 = 0;
uint32_t tiempo_buzzer_aterrizaje=0;
int pwm_activo =0; //para ver si todavia se esta mandando la señal pwm
int cont_aterrizaje=0;//Contar hasta que aterriza
int bandera=1;

int aterrizaje_condiciones=0;

//-----------Variables de aceleración-----------------
//metros por segundo
float acc_x = 0.0f;
float acc_y = 0.0f;
float acc_z = 0.0f;

//grados por segundo
float gyro_x = 0.0f;
float gyro_y = 0.0f;
float gyro_z = 0.0f;

//radianes por segundo
float gx_rad = 0.0f;
float gy_rad = 0.0f;
float gz_rad = 0.0f;

//angulos de rotación
float ang_x = 0.0f;
float ang_y = 0.0f;
float ang_z = 0.0f;

//---------------Variables magnetometro----------------
float mag_x = 0.0f;
float mag_y = 0.0f;
float mag_z = 0.0f;

//------------Gravedad vectorial para sacar aceleracion sin gravedad------
float g_x = 0.0f;
float g_y = 0.0f;
float g_z = 0.0f;

//--------------aceleracion sin gravedad --------------------
float acc_x_sg = 0.0f;
float acc_y_sg = 0.0f;
float acc_z_sg = 0.0f;

//-----bandera para leer cada 500 HZ el imu ------------------
volatile uint8_t imu_300hz = 0;
volatile uint8_t sd_300hz = 0;


//---------------Variables de SD---------------
FATFS SD;
FIL archivo_vuelo;
FRESULT archivo_estatus;
FILINFO fno;

uint8_t bufr[80]; //informacion leida
UINT br; //Para bytes leidos
char header_archivo[] =
		"t_ms,presion,temperatura,altura,altura_max,ax,ay,az,asgx, asgy, asgz, gx, gy,gz, angx, angy, angz,mx,my,mz,v,act1, act2, estado\r\n";
char lecturas_archivo[200];
char nombre_archivo[20];
int contador_lecturas_guardadas;
int numero_archivo;
char buffer_sd[4096];
uint32_t buffer_sd_len = 0;


//variables a quitar solo estan para analizar si hay algun error en el guardado de datos
volatile uint32_t sd_muestras = 0;
volatile uint32_t sd_escrituras = 0;
int ultimo_len = 0;
FRESULT ultimo_error_sd = FR_OK;


//-------------Variables filtro Kalman --------------------
uint8_t bandera_kalman = 0;
uint16_t contador_estable = 0;//necesario porque la aceleración sin gravedad tarda en converger
float error_acc_sg = 0.0f;
float acc_vertical = 0.0f;
float velocidad_vertical = 0.0f;
float altura_kalman = 0.0f;
float acc_vertical_medida = 0.0f;

float modulo_acc = 0.0f;

uint32_t tiempo_bmp = 0;
uint32_t tiempo_mag = 0;
uint32_t tiempo_aterrizaje = 0;


//vARIABLES AUXILIARES PARA VER DE CUANTO ES NUESTRA FRECUENCIA PARA EL FILTRO - despues se quitarám
volatile uint32_t tim6_total = 0;
uint32_t contador_madgwick = 0;

uint32_t tim6_hz = 0;
uint32_t madgwick_hz = 0;

uint32_t tiempo_hz = 0;

uint32_t tim6_anterior = 0;
uint32_t madgwick_anterior = 0;




uint32_t tiempo_sync = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */
#define LORA_FREQ_HZ   915000000UL
#define LORA_SF        10          /* 6..12 */
#define LORA_BW_HZ     250000UL    /* 7800..500000 */
#define LORA_CR4       5           /* 4/5..4/8 */
#define LORA_SYNC_WORD 0x12
#define LORA_TX_DBM    17 		/*2..20 dBm (lora_begin pone 17)*/

//-----------------declaración de función recuperación I2C---------------
void i2c_recover_bus(I2C_HandleTypeDef *hi2c);

//------------------incializar SD -----------------------
void inicializarSD();

//---------------inicilizar BMP --------------------
void inicializarBMP();

//----------------Inicializar MPU ------------------
void inicializarMPU();


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//-------------Interrupción para la lectura de MPU-------------
    if (htim->Instance == TIM6){
        imu_300hz = 1;
        tim6_total++;
    }

	//--------------Buzzer-------------
	if (htim->Instance == TIM7){
	     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	 }

	if (htim->Instance == TIM10){
		 sd_300hz=1;
	}
}

//------------Esta función es para diagnosticar errores en la sd (Prueba)  --------------
void error_led(uint8_t codigo){
    while (1){
        for (uint8_t i = 0; i < codigo; i++){
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
            HAL_Delay(200);

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
            HAL_Delay(200);
        }

        HAL_Delay(1500);
    }
}

//-------------Activación pwm con timer -------------------
void timer_pwm(){

	HAL_TIM_Base_Start_IT(&htim7);
	HAL_Delay(3000);
	HAL_TIM_Base_Stop_IT(&htim7);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_Delay(1000);


	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_Delay(8000);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
	HAL_Delay(3000);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TX_GAP_MS 150              /* pausa entre fin de un paquete y el siguiente */

static int      tx_busy;
static uint32_t t_last;            /* inicio del TX en curso, o fin del último */
static volatile uint8_t dio0_irq;  /* lo pone la EXTI de DIO0 */

static void on_tx_done(void)
{
  tx_busy = 0;
  t_last = HAL_GetTick();
  HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);

}


//-------------------------Creación de objetos ----------------------------

//---------------------Estado de vuelo inicial--------------
EstadoVuelo estado = ESTADO_LAUNCHPAD;

//------------Creación de objetos para BMP280--------------------
BMP280_HandleTypedef bmp280;
bmp280_params_t parametro_bmp280;

//----------------Crear objeto MPU6050 -------------------------------
MPU6050_t mpu6050;

//----------------------Se crea el vector de variables magnetometro --------------------
Vector magnetometro;

//--------------objeto kalman----------------
KalmanVertical_t kalman;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_TIM12_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  //---------------Temporizadores auxiliares para delimitar la frecuencia de sensores en interrupciones ------------
  HAL_TIM_Base_Start_IT(&htim6);//MPU6050 a 300 Hz
  HAL_TIM_Base_Start_IT(&htim10);//SD a 300 Hz



  //----------------Inicialización del LORA -----------------------------
  lora_hw_t hw = { &hspi1, LORA_NSS_GPIO_Port, LORA_NSS_Pin, LORA_RST_GPIO_Port, LORA_RST_Pin };
  if (!lora_begin(&hw, LORA_FREQ_HZ)) Error_Handler();

  lora_set_spreading_factor(LORA_SF);
  lora_set_signal_bandwidth(LORA_BW_HZ);
  lora_set_coding_rate4(LORA_CR4);
  lora_set_sync_word(LORA_SYNC_WORD);
  lora_set_crc(1);

  lora_set_tx_power(LORA_TX_DBM, LORA_PA_OUTPUT_PA_BOOST);

  lora_on_tx_done(on_tx_done);     /* callback a DIO0  */

  telemetria_t tel;

  //--------------------inicializar SD ------------------
  inicializarSD();


//-------------------inicializar BMP ----------------------
  inicializarBMP();

  //------------inicializar MPU ----------------------

  inicializarMPU();

 	//-------------Configuración magnetometro-----------------------------
 	HMC5883L_setOffset(OFFSET_X, OFFSET_Y, OFFSET_Z);
 	HMC5883L_setRange(HMC5883L_RANGE_1_3GA);
 	HMC5883L_setMeasurementMode(HMC5883L_CONTINOUS);
 	HMC5883L_setDataRate(HMC5883L_DATARATE_75HZ);
 	HMC5883L_setSamples(HMC5883L_SAMPLES_8);

 	//-------------Señal de estado de que se inicializaron todos los sensores y demás dpositivos -----------------

 	//-----------Buzzer ------------------
 	HAL_TIM_Base_Start_IT(&htim7);
 	HAL_Delay(3000);
 	HAL_TIM_Base_Stop_IT(&htim7);
 	HAL_Delay(1000);

 	tiempo_sync = HAL_GetTick();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (imu_300hz) {
	  	imu_300hz = 0;
	  	MPU6050_ReadAccel(&hi2c1, &mpu6050);
	  	MPU6050_ReadGyro(&hi2c1, &mpu6050);

	  	acc_x = (float) mpu6050.acc_raw[1] / 2048.0f * 9.80665f;
	  	acc_y = (float) mpu6050.acc_raw[2] / 2048.0f * 9.80665f;
	  	acc_z = (float) mpu6050.acc_raw[0] / 2048.0f * 9.80665f;

	  	//-------------Correcciones que se obtuvieron por datos ----------------
	  	acc_x = acc_x - BIAS_ACC_X_1;
	  	acc_y = acc_y - BIAS_ACC_Y_1;
	  	acc_z = acc_z - BIAS_ACC_Z_1;

	  	gyro_x = (float) mpu6050.gyro_raw[1] / 16.4f;
	  	gyro_y = (float) mpu6050.gyro_raw[2] / 16.4f;
	  	gyro_z = (float) mpu6050.gyro_raw[0] / 16.4f;

	  	//------------Se obtiene en radianes por segundo para el filtro----------
	  	gx_rad = gyro_x * 0.01745329252f;
	  	gy_rad = gyro_y * 0.01745329252f;
	  	gz_rad = gyro_z * 0.01745329252f;

	  	//------------Filtro Madwick para cuaterniones y angulos -----------
	  	MadgwickAHRSupdateIMU(gx_rad, gy_rad, gz_rad, acc_x, acc_y, acc_z);
	  	contador_madgwick++; //--------------contador para ver si son los 300Hz------------

	  	//--------Obtención de angulos de rotación--------------------
	  	computeAngles(); //Variables extern roll, pitch, yaw
	  	ang_x = roll;
	  	ang_y = pitch;
	  	ang_z = yaw;

	  	//---------Gravedad vectorial para sacar aceleración sin gravedad con madwick----------
	  	g_x = 2.0f*(q1*q3-q0*q2)*9.80665f;
	  	g_y = 2.0f*(q0*q1+q2*q3)*9.80665f;
	  	g_z = (q0*q0-q1* q1-q2 * q2+q3 *q3) * 9.80665f;

	  	//---------aceleracion sin gravedad -----------------
	  	acc_x_sg = acc_x - g_x;
	  	acc_y_sg = acc_y - g_y;
	  	acc_z_sg = acc_z - g_z;

	  	acc_vertical_medida =(acc_x*g_x + acc_y*g_y + acc_z*g_z)/ 9.80665f;
	  	acc_vertical =acc_vertical_medida-9.80665f;
	  	error_acc_sg = sqrtf(acc_x_sg*acc_x_sg+acc_y_sg*acc_y_sg+acc_z_sg*acc_z_sg);

	  	//------------bandera para que a partir de la convergencia de la aceleración se active kalman ----------
	  	if (!bandera_kalman){
	  		if (error_acc_sg < 0.5f){
	  			contador_estable++;
	  		if (contador_estable >= 500){
	  			Kalman_Init(&kalman);
	  			kalman.altura = altura;
	  			kalman.velocidad = 0.0f;

	  			bandera_kalman= 1;
	  			}
	  		}
	  		else
	  			contador_estable = 0;
	  	}

	  	//-----------Predicción kalman --------------
	  	if (bandera_kalman){
	  		Kalman_Predict(&kalman,acc_vertical,FRECUENCIA_MPU);
	  		velocidad_vertical = kalman.velocidad;
	  		altura_kalman = kalman.altura;
	  	 }
	  	}

	  //------------------Almacenamiento de datos
	    if (sd_300hz){
	  	sd_300hz = 0;
	  	int len = snprintf(lecturas_archivo,sizeof(lecturas_archivo),"%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
	  		        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d\r\n",

	  		       HAL_GetTick(),presion,temperatura,altura,altura_maxima,acc_x,acc_y,acc_z,acc_x_sg,acc_y_sg,
	  		       acc_z_sg,gyro_x,gyro_y,gyro_z,ang_x,ang_y,ang_z,mag_x,mag_y,mag_z,velocidad_vertical, 0,0,estado);

	  	//----------------Alacenamiento en buffer para no escribir en la SD muchas veces ----------------
	  	if (len > 0 && len < sizeof(lecturas_archivo)){
	  		if (buffer_sd_len + len < sizeof(buffer_sd)){
	  			memcpy(&buffer_sd[buffer_sd_len],lecturas_archivo,len);
	  			buffer_sd_len += len;
	  		}
	  	}

	  	//----------------Se escribe cada que supera ese almacenamiento---------------
	    if (buffer_sd_len > 512){

	    	ultimo_error_sd =sd_write_buffer(buffer_sd, buffer_sd_len);
	  		if (ultimo_error_sd == FR_OK){
	  		            sd_escrituras++;
	  		            buffer_sd_len = 0;
	  		 }

	  	}
	}

	 //----------------Lee el bmp a 25 Hz --------------
	 if(HAL_GetTick()-tiempo_bmp >= 40){

	  	tiempo_bmp=HAL_GetTick();

	  	if(!bmp280_read_float(&bmp280, &temperatura, &presion, NULL)) {
	  		printf("No se pudieron asignar valores .... BMP280\n");
	  	}

	  	altura=CalcularAltura(presion, presion_nivel_mar) - altura_nivel_mar;

	  	if (isnan(altura)){
	  		printf("No se pudo calcular la medida de altura\n");
	  	}

	  	if (bandera_kalman){
	  		Kalman_UpdateBaro(&kalman, altura);
	  	}
	 }

	 //---------------Se lee magnetometro cada 20 Hz -------------------
	 if(HAL_GetTick()-tiempo_mag>=20){

	  	tiempo_mag=HAL_GetTick();

	  	//-------------Obtención de datos del magnetometro-----------
	  	magnetometro = HMC5883L_readNormalize();
	  	mag_x = magnetometro.XAxis;
	  	mag_y = magnetometro.YAxis;
	  	mag_z = magnetometro.ZAxis;

	  }

	 //-----------Guardado de datos cada segundo para no bajar la frecuencia de filtrado ---------------
	 if ((HAL_GetTick()-tiempo_sync) >= 1000){
		 tiempo_sync = HAL_GetTick();

	  	 FRESULT res_sync = sd_sync();
	  	 if (res_sync != FR_OK){
	  		ultimo_error_sd = res_sync;
	  	}
	  }


	 //-----------------Función auxiliar para debuggear el filtrado y ver a cuanta frecuencia llega ---------------
/*	 if ((HAL_GetTick() - tiempo_hz) >= 1000){
		 int32_t ahora = HAL_GetTick();
		 uint32_t dt = ahora - tiempo_hz;

	  	 uint32_t tim6_actual = tim6_total;
	  	 uint32_t madgwick_actual = contador_madgwick;

	  	 tim6_hz =((tim6_actual - tim6_anterior) * 1000UL) / dt;

	  	 madgwick_hz =((madgwick_actual - madgwick_anterior) * 1000UL) / dt;

	     tim6_anterior = tim6_actual;
	     madgwick_anterior = madgwick_actual;

	  	 tiempo_hz = ahora;
	 }
*/


	//-------------Obtención de altura máxima ------------------
	if (altura > altura_maxima)
	   altura_maxima = altura;

	//--------------------Actualización de datos por telemetria -----------------
	tel.t_ms = HAL_GetTick();
	tel.Altitud=altura;
	tel.AccX=acc_x_sg;
	tel.AccY=acc_y_sg;
	tel.AccZ=acc_z_sg;
	tel.GyrX=gyro_x;
	tel.GyrY=gyro_y;
	tel.GyrZ=gyro_z;
	tel.MagX=mag_x;
	tel.MagY=mag_y;
	tel.MagZ=mag_z;
	tel.VelX=velocidad_vertical;
	tel.Presion=presion;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   if (dio0_irq) {			/* -> on_tx_done() */
      dio0_irq = 0;
      lora_handle_dio0();
    }
    if (tx_busy) {
    	// si el tx sigue on y ademas ha pasado un timeout, automaticamente resetea el LoRa (solo por si acaso)
      if (HAL_GetTick() - t_last > LORA_TX_TIMEOUT_MS) {
        lora_idle();
        tx_busy = 0;
      }

    } else if (HAL_GetTick() - t_last >= TX_GAP_MS) { // TX_GAP_MS, indica la frecuencia de envio
      /* paquete en ceros */

     if (lora_begin_packet(0)) {
        lora_write((uint8_t *)&tel, sizeof tel);
        lora_end_packet(1);                         /* vuelve al instante */
      tx_busy = 1;
      t_last = HAL_GetTick();
     }
    }


    /*Insertar logica de vuelo aqui */
    if(aspiradora){
    //----------------------Lógica de vuelo -----------------------------
    	switch(estado){
    		case ESTADO_LAUNCHPAD:
    			if(altura > ALTURA_DESPEGUE){
    /*			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
    			 HAL_Delay(1000);
    			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
    			 HAL_Delay(1000);
    */			 estado=ESTADO_ASCENSO;
    		 }
    		 break;
    		case ESTADO_ASCENSO:
    		 //Esto solo es de prueba para anlizarlo posteriormente se debe de quitar para no parar el código
    /*		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
    		 HAL_Delay(1000);
    		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
    		 HAL_Delay(1000);

    */		 	if(altura <(altura_maxima - ALTURA_ERROR_BMP)){
    				estado=ESTADO_APOGEO;
    				HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    				pwm_activo = 1;
    				//----------se empieza a contar el tiempo para dejar encendido ....etapa 1 ----------
    				tiempo_canal_etapa_1=HAL_GetTick();

    		 	 }
    		 break;


    		case ESTADO_APOGEO:

    			 //-------------tiempo de encendido canal 1-----------------
    			 if(HAL_GetTick()-tiempo_canal_etapa_1>=4000 && pwm_activo==1){
    				 HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
    				 pwm_activo = 0;
    			 }


    		 //condición para la reefing line
    		 if(altura<=ALTURA_REEFING_LINE){
    			 estado=ESTADO_REEFING_LINE;
    			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
    			 //----------se empieza a contar el tiempo para dejar encendido ....etapa 1 ----------
    			 tiempo_canal_etapa_2=HAL_GetTick();

    		 }
    		 break;


    		 case ESTADO_REEFING_LINE:

    		 if(pwm_activo==1 && HAL_GetTick()-tiempo_canal_etapa_1>=4000){
    				 HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
    				 pwm_activo=0;
    				 aterrizaje_condiciones=1;
    				 tiempo_aterrizaje=HAL_GetTick();
    	      }

    		 if(HAL_GetTick()-tiempo_canal_etapa_2>=1000){
    		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
    		 	 if(aterrizaje_condiciones==1){
    		 		aterrizaje_condiciones=2;
    		 	 }
    		 }

    		 if(abs(altura-altura_anterior)<1){
    			 cont_aterrizaje++;
    		 }else{
    			 cont_aterrizaje--;
    		 }

    		 if(cont_aterrizaje>=10000 && aterrizaje_condiciones==2 && (HAL_GetTick()-tiempo_aterrizaje)>=180000){
    			 estado=ESTADO_ATERRIZAJE;
    		 }
    		 break;

    		 case ESTADO_ATERRIZAJE:
    			 	if(bandera){
    				HAL_TIM_Base_Start_IT(&htim7);
    				bandera=0;
    				tiempo_buzzer_aterrizaje=HAL_GetTick();
    			 	}

    			 	if(HAL_GetTick()-tiempo_buzzer_aterrizaje>=100000){
    				HAL_TIM_Base_Stop_IT(&htim7);
    			 	}
    		 break;


    		 }
    		 }
    		 altura_anterior=altura;

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */
  hsd.Init.ClockDiv = 20;
  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 279;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 20;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 559;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 83;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|LORA_NSS_Pin|LORA_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 LED_1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 LORA_NSS_Pin LORA_RST_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_15|LORA_NSS_Pin|LORA_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_DIO0_Pin */
  GPIO_InitStruct.Pin = LORA_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LORA_DIO0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Solo marca: el SPI se hace en el loop para no chocar con otras transacciones. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == LORA_DIO0_Pin)
    dio0_irq = 1;
}


//_-------------Función para inicializar SD ----------------------
void inicializarSD(){

	  //----------------Se inicializa SD------------------
	  if (BSP_SD_Init() != MSD_OK){
	 	while (1){
	 		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	 		HAL_Delay(200);
	 	 	 }
	 	 }

	 	 //-----------Creación e inicialización de archivo SD------------
	 	 if(sd_mount()!=FR_OK){
	 		 while(1){
	 			 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	 			 HAL_Delay(200);
	 		 }
	 	 }

	 	 //----------Nombre del archivo ----------------------
	 	 while(1){
	 		 snprintf(nombre_archivo,sizeof(nombre_archivo),"Vuelo%03d.CSV",numero_archivo);
	 		 archivo_estatus=f_stat(nombre_archivo, &fno);
	 		 if(archivo_estatus == FR_NO_FILE)
	 			 break;

	 		 if(archivo_estatus!=FR_OK){
	 			 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	 			 HAL_Delay(200);
	 		 }

	 	 numero_archivo++;
	 	 }

	 	 //--------------Abrir sesión de datos -----------
	 	 if(sd_open_log(nombre_archivo) != FR_OK)
	 	 {
	 		 while(1){
	 			 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	 			 HAL_Delay(200);
	 		 }
	 	 }

	 	 //---------------------Se escribe el encabezado ---------------------
	 	 if(sd_write_log(header_archivo,&contador_lecturas_guardadas)!=FR_OK)
	 	 {
	 		 while(1){
	 			 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	 			 HAL_Delay(200);
	 		 }
	 	 }

	 	 contador_lecturas_guardadas=0;
}

//---------------------funcion para inicializar BMP --------------------
void inicializarBMP(){
	 //--------Inicialización del BMP280-------------
	 	/*Si queremos utilizar I2C*/
	 	bmp280.comm_mode = BMP280_MODE_I2C;
	 	bmp280.i2c = &hi2c1;
	 	bmp280.addr = BMP280_I2C_ADDRESS_0;

	 	//Si queremos utilizar SPI
	 /*	bmp280.comm_mode=BMP280_MODE_SPI;
	 	bmp280.spi=&hspi1;
	 	bmp280.cs_port=GPIOB;
	 	bmp280.cs_pin=GPIO_PIN_12;
	 	bmp280_init_default_params(&parametro_bmp280);
	*/
	 	//----- Aqui va el cambio de parametros ----
	 	parametro_bmp280.mode = BMP280_MODE_NORMAL;
	 	parametro_bmp280.filter = BMP280_FILTER_4;
	 	parametro_bmp280.oversampling_pressure = BMP280_HIGH_RES;
	 	parametro_bmp280.oversampling_temperature = BMP280_HIGH_RES;
	 	parametro_bmp280.standby = BMP280_STANDBY_05;

	 	//----------------Inicialización de  BMP280---------------
	 	while (!bmp280_init(&bmp280, &parametro_bmp280)) {
	 			//-------recuperar I2C--------------------
	 			i2c_recover_bus(&hi2c1);
	 		}

	 		HAL_Delay(100);

	 	//----------Se obtiene por primera vez los datos ----------
	 	while (!bmp280_read_float(&bmp280, &temperatura, &presion, NULL)) {
	 		printf("No se pudieron asignar valores .... BMP280\n");
	 	}

	 	//----------Se calibra la altura inicial para una mejor lectura ----------
	 	while (cont_calibracion_bmp < 100) {
	 		bmp280_read_float(&bmp280, &temperatura, &presion, NULL);
	 		presion_acumulacion = presion_acumulacion + presion;
	 		cont_calibracion_bmp++;
	 		}

	 	presion_calibrada_suelo = presion_acumulacion / (float) cont_calibracion_bmp;

	 	//----- se calcula la altura con respecto a nivel del mar calibrada --------
	 	altura_nivel_mar = CalcularAltura(presion_calibrada_suelo,presion_nivel_mar);

	 	//-------Condición de seguridad ----------
	 	if (isnan(altura_nivel_mar)) {
	 		printf("No se pudo calcular la medida de altura inicial (nivel del mar)\n");
	 		Error_Handler();
	 	}
}

//---------------Inicializar los registros de MPU6050 y calibración de giroscopio -------------------------
void inicializarMPU(){
		while ((MPU6050_Init(&hi2c1) != HAL_OK)) {
			printf("No se dectectó el MPU6050");
			i2c_recover_bus(&hi2c1);
			//Error_Handler();
		}

		MPU6050_CalibrateGyro(&hi2c1, &mpu6050);
}

void i2c_recover_bus(I2C_HandleTypeDef *hi2c) {
	GPIO_InitTypeDef gpio = { 0 };

	HAL_I2C_DeInit(hi2c);

	__HAL_RCC_GPIOB_CLK_ENABLE();

	gpio.Mode = GPIO_MODE_OUTPUT_OD;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;

	gpio.Pin = I2C_RECOV_SCL_PIN;
	HAL_GPIO_Init(I2C_RECOV_SCL_PORT, &gpio);

	gpio.Pin = I2C_RECOV_SDA_PIN;
	HAL_GPIO_Init(I2C_RECOV_SDA_PORT, &gpio);

	HAL_GPIO_WritePin(I2C_RECOV_SCL_PORT,
	I2C_RECOV_SCL_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(I2C_RECOV_SDA_PORT,
	I2C_RECOV_SDA_PIN, GPIO_PIN_SET);

	HAL_Delay(1);

	for (int i = 0; i < 9; i++) {
		if (HAL_GPIO_ReadPin(I2C_RECOV_SDA_PORT,
		I2C_RECOV_SDA_PIN) == GPIO_PIN_SET) {
		break;
		}

		HAL_GPIO_WritePin(I2C_RECOV_SCL_PORT,
		I2C_RECOV_SCL_PIN, GPIO_PIN_RESET);

		HAL_Delay(1);

		HAL_GPIO_WritePin(I2C_RECOV_SCL_PORT,
		I2C_RECOV_SCL_PIN, GPIO_PIN_SET);

		HAL_Delay(1);
	}


	HAL_GPIO_WritePin(I2C_RECOV_SDA_PORT,
	I2C_RECOV_SDA_PIN, GPIO_PIN_RESET);

	HAL_Delay(1);

	HAL_GPIO_WritePin(I2C_RECOV_SCL_PORT,
	I2C_RECOV_SCL_PIN, GPIO_PIN_SET);

	HAL_Delay(1);

	HAL_GPIO_WritePin(I2C_RECOV_SDA_PORT,
	I2C_RECOV_SDA_PIN, GPIO_PIN_SET);

	HAL_Delay(1);

	gpio.Mode = GPIO_MODE_AF_OD;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = GPIO_AF4_I2C1;

	gpio.Pin = I2C_RECOV_SCL_PIN;
	HAL_GPIO_Init(I2C_RECOV_SCL_PORT, &gpio);

	gpio.Pin = I2C_RECOV_SDA_PIN;
	HAL_GPIO_Init(I2C_RECOV_SDA_PORT, &gpio);

	HAL_I2C_Init(hi2c);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
