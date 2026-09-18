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
#include "bmp280.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sd_functions.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	ESTADO_LAUNCHPAD=0,
	ESTADO_ASCENSO=1,
	ESTADO_APOGEO=2,//activación de primera etapa de recuperación
	ESTADO_REEFING_LINE=3, //activación de segunda etapa de recuperación
	ESTADO_ATERRIZAJE=4
}EstadoVuelo;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


//---------Valores de altura predeterminadas
#define ALTURA_ERROR_BMP 3
#define ALTURA_DESPEGUE 10
#define ALTURA_REEFING_LINE 500


#define I2C_RECOV_SCL_PORT  GPIOB
#define I2C_RECOV_SCL_PIN   GPIO_PIN_8

#define I2C_RECOV_SDA_PORT  GPIOB
#define I2C_RECOV_SDA_PIN   GPIO_PIN_9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

//---lecturas bmp280---------
float temperatura=0.0f;
float presion=0.0f; 
float altura=0.0f;

//------------------Calibración --------------------
float presion_acumulacion=0.0f;
float presion_calibrada_suelo=0.0f;
int cont_calibracion_bmp;

//----Variables para calcular altitud --------


float presion_nivel_mar=101325.0f;//Este valor esta en pascales
float altura_nivel_mar=0.0f; //Es la altura en el suelo
float altura_maxima=0.0f;

//------Variables para estados -----------------
uint32_t tiempo_canal_etapa_1 = 0;
uint32_t tiempo_canal_etapa_2 = 0;


EstadoVuelo estado=ESTADO_LAUNCHPAD;




//---------------Variables de SD---------------
FATFS SD;
FIL archivo_vuelo;
FRESULT archivo_estatus;
FILINFO fno;

uint8_t bufr[80];//informacion leida
UINT br; //Para bytes leidos
char header_archivo[]="t_ms,presion,temperatura,altura,altura_max,ax,ay,az,gx,gy,gz,mx,my,mz,v,estado\r\n";
char lecturas_archivo[200];
char nombre_archivo[20];
int contador_lecturas_guardadas;
int numero_archivo;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */
//-----------------declaración de función recuperación I2C---------------
void i2c_recover_bus(I2C_HandleTypeDef *hi2c);


//-------------Aqui se pondran las funciones creadas--------------
int __io_putchar(int ch){

  //----UART para debugger -----
  HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);

  return ch;

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Creación de objetos para BMP280
BMP280_HandleTypedef bmp280;
bmp280_params_t parametro_bmp280;







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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  hsd.Init.ClockDiv = 10;

/*
  //----------------Se inicializa SD------------------
  if (BSP_SD_Init() != MSD_OK)
  {
      printf("Error inicializando SD\r\n");

      while (1)
      {
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
          HAL_Delay(200);
      }
  }

  //-----------Creación e inicialización de archivo SD------------
  if(sd_mount()!=FR_OK){
	  while(1)
	  	  {
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
		  HAL_Delay(200);
	  	  }
  }

  //----------Nombre del archivo ----------------------
	while(1)
	{
	    snprintf(nombre_archivo,
	    		sizeof(nombre_archivo),
	            "Vuelo%03d.CSV",
	            numero_archivo);
	    archivo_estatus=f_stat(nombre_archivo, &fno);
	    if(archivo_estatus == FR_NO_FILE)
	    {
	        break;
	    }

	    if(archivo_estatus!=FR_OK)
	    {
	    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	    	HAL_Delay(200);
	    }

	    numero_archivo++;
	}

	//--------------Abrir sesión de datos -----------
	if(sd_open_log(nombre_archivo) != FR_OK)
	  	{
	  	    while(1)
	  	    {
	  	        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	  	        HAL_Delay(200);
	  	    }
	  	}

	//---------------------Se escribe el encabezado ---------------------
	if(sd_write_log(header_archivo,&contador_lecturas_guardadas)!=FR_OK)
	{
		while(1)
		  	    {
		  	        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
		  	        HAL_Delay(200);
		  	    }
	}
	contador_lecturas_guardadas=0;


*/
  //--------Inicialización del BMP280-------------

  /*Si queremos utilizar I2C*/
 // bmp280.comm_mode=BMP280_MODE_I2C;
 // bmp280.i2c=&hi2c1;
 // bmp280.addr=BMP280_I2C_ADDRESS_0;


 //Si queremos utilizar SPI
  bmp280.comm_mode=BMP280_MODE_SPI;
  bmp280.spi=&hspi1;
  bmp280.cs_port=GPIOB;
  bmp280.cs_pin=GPIO_PIN_12;

  bmp280_init_default_params(&parametro_bmp280);

  //----- Aqui va el cambio de parametros ----
  //parametro_bmp280.filter=BMP280_FILTER_4;
  //parametro_bmp280.oversampling_pressure=BMP280_HIGH_RES;
  //parametro_bmp280.oversampling_temperature=BMP280_HIGH_RES;
  //parametro_bmp280.standby=BMP280_STANDBY_05;

  parametro_bmp280.mode = BMP280_MODE_NORMAL;
  parametro_bmp280.filter = BMP280_FILTER_4;
  parametro_bmp280.oversampling_pressure = BMP280_HIGH_RES;
  parametro_bmp280.oversampling_temperature = BMP280_HIGH_RES;
  parametro_bmp280.standby = BMP280_STANDBY_05;

  //Se inicializa el BMP280
  while(!bmp280_init(&bmp280, &parametro_bmp280)){
    printf("No se dectecto y/o inicializo el BMP280\n");

    //-------recuperar I2C--------------------
  }

  HAL_Delay(100);


  //----------Se obtiene por primera vez los datos ----------
  while(!bmp280_read_float(&bmp280,&temperatura,&presion,NULL)){
    printf("No se pudieron asignar valores .... BMP280\n");
    }


  //----------Se calibra la altura inicial para una mejor lectura ----------
  /*while(cont_calibracion_bmp<100){
	  bmp280_read_float(&bmp280,&temperatura,&presion,NULL);
	  presion_acumulacion=presion_acumulacion+presion;
	  cont_calibracion_bmp++;
  }*/

  presion_calibrada_suelo=presion;//presion_acumulacion/(float)cont_calibracion_bmp;
  //----- se calcula la altura con respecto a nivel del mar calibrada --------
  altura_nivel_mar=CalcularAltura(presion_calibrada_suelo,presion_nivel_mar);

  //-------Condición de seguridad ----------
  if(isnan(altura_nivel_mar)){
    printf("No se pudo calcular la medida de altura inicial (nivel del mar)\n");
    Error_Handler();
  }

  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
  			  HAL_Delay(1000);
  			  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
  			  HAL_Delay(1000);
  			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
  			  			  HAL_Delay(1000);
  			  			  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
  			  			  HAL_Delay(1000);

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    while(!bmp280_read_float(&bmp280,&temperatura,&presion,NULL)){
      printf("No se pudieron asignar valores .... BMP280\n");
    }

    altura=CalcularAltura(presion,presion_nivel_mar)-altura_nivel_mar;

    while(isnan(altura))
    	printf("No se pudo calcular la medida de altura\n");


    if(altura > altura_maxima)
    	altura_maxima=altura;

    printf("Altura: %.2f\n",altura);
    printf("Altura maxima: %.2f\n",altura_maxima);
    printf("estado %d\n",estado);
    printf("presion %.2f\n",presion);
    printf("altura mar: %.2f\n", altura_nivel_mar);

    //--------------Guarda solo la presion y altura ---------------
    snprintf(lecturas_archivo,
             sizeof(lecturas_archivo),
             "%lu,%.2f,%.2f\r\n",
             HAL_GetTick(),
             presion,
             altura);

    if(sd_write_log(lecturas_archivo,
                    &contador_lecturas_guardadas) != FR_OK)
    {
        printf("Error escribiendo en SD\n");
    }

 //----------------------Lógica de vuelo -----------------------------
    switch(estado)
    {
    	case ESTADO_LAUNCHPAD:
    		if(altura > ALTURA_DESPEGUE){

    			estado=ESTADO_ASCENSO;
    		}
    			break;

    	case ESTADO_ASCENSO:
			//Esto solo es de prueba para anlizarlo posteriormente se debe de quitar para no parar el código
			  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
			  HAL_Delay(1000);

			  if(altura <(altura_maxima - ALTURA_ERROR_BMP)){
				  estado=ESTADO_APOGEO;
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);

		 //----------se empieza a contar el tiempo para dejar encendido ....etapa 1 ----------
				  tiempo_canal_etapa_1=HAL_GetTick();

			  }
    			break;
    	case ESTADO_APOGEO:
    		//-------------tiempo de encendido canal 1-----------------
    		if(HAL_GetTick()-tiempo_canal_etapa_1>=1000){
    			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
    		}

    		//condición para la reefing line
    		if(altura<=500){
    			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);//Por si en dado caso no llega a 2km
    			estado=ESTADO_REEFING_LINE;
    			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
     //----------se empieza a contar el tiempo para dejar encendido ....etapa 1 ----------
    			tiempo_canal_etapa_2=HAL_GetTick();

    		}
    		break;
    	case ESTADO_REEFING_LINE:
    		if(HAL_GetTick()-tiempo_canal_etapa_2>=1000){
    			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
    		}
    		break;
    	case ESTADO_ATERRIZAJE:
    		break;


    		}
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void i2c_recover_bus(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio = {0};

    /* 1. Desactivar I2C */
    HAL_I2C_DeInit(hi2c);

    /* 2. Habilitar reloj del GPIO */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 3. SCL y SDA como GPIO Open Drain */
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = I2C_RECOV_SCL_PIN;
    HAL_GPIO_Init(I2C_RECOV_SCL_PORT, &gpio);

    gpio.Pin = I2C_RECOV_SDA_PIN;
    HAL_GPIO_Init(I2C_RECOV_SDA_PORT, &gpio);

    /* Liberar bus */
    HAL_GPIO_WritePin(I2C_RECOV_SCL_PORT,
                      I2C_RECOV_SCL_PIN,
                      GPIO_PIN_SET);

    HAL_GPIO_WritePin(I2C_RECOV_SDA_PORT,
                      I2C_RECOV_SDA_PIN,
                      GPIO_PIN_SET);

    HAL_Delay(1);

    /* 4. Hasta 9 pulsos de SCL */
    for (int i = 0; i < 9; i++)
    {
        /* Si SDA ya está libre, terminar */
        if (HAL_GPIO_ReadPin(I2C_RECOV_SDA_PORT,
                             I2C_RECOV_SDA_PIN) == GPIO_PIN_SET)
        {
            break;
        }

        /* SCL LOW */
        HAL_GPIO_WritePin(I2C_RECOV_SCL_PORT,
                          I2C_RECOV_SCL_PIN,
                          GPIO_PIN_RESET);

        HAL_Delay(1);

        /* SCL HIGH */
        HAL_GPIO_WritePin(I2C_RECOV_SCL_PORT,
                          I2C_RECOV_SCL_PIN,
                          GPIO_PIN_SET);

        HAL_Delay(1);
    }

    /* 5. Generar STOP:
       SDA LOW -> SCL HIGH -> SDA HIGH
    */

    HAL_GPIO_WritePin(I2C_RECOV_SDA_PORT,
                      I2C_RECOV_SDA_PIN,
                      GPIO_PIN_RESET);

    HAL_Delay(1);

    HAL_GPIO_WritePin(I2C_RECOV_SCL_PORT,
                      I2C_RECOV_SCL_PIN,
                      GPIO_PIN_SET);

    HAL_Delay(1);

    HAL_GPIO_WritePin(I2C_RECOV_SDA_PORT,
                      I2C_RECOV_SDA_PIN,
                      GPIO_PIN_SET);

    HAL_Delay(1);

    /* 6. Regresar pines a Alternate Function I2C */
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;

    gpio.Pin = I2C_RECOV_SCL_PIN;
    HAL_GPIO_Init(I2C_RECOV_SCL_PORT, &gpio);

    gpio.Pin = I2C_RECOV_SDA_PIN;
    HAL_GPIO_Init(I2C_RECOV_SDA_PORT, &gpio);

    /* 7. Reiniciar periférico I2C */
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
