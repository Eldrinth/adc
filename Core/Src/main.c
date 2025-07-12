/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<oled.h>
#include<font.h>
#include "arm_math.h"
#include "arm_const_structs.h"
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Pi 3.1415926
#define SAMPLE 277
#define FFT_LENGTH 1024

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t DualSine12bit[SAMPLE]__attribute__((section(".out")));
uint16_t high[SAMPLE]__attribute__((section(".adc")));
uint16_t adc_data[FFT_LENGTH*2]__attribute__((section(".adc")));
uint16_t out_data[FFT_LENGTH*2]__attribute__((section(".out")));
uint16_t fft_data1[FFT_LENGTH]__attribute__((section(".out")));
uint16_t fft_data2[FFT_LENGTH]__attribute__((section(".out")));

uint8_t fft_complete_flag=0;
char* str1;
int type=0;
float value=0;
float Q=0;
float hanning_window[FFT_LENGTH];

float FREQUENCY[FFT_LENGTH];//FFT变换之后对应的真实频率，我还没写呢
float dif_phase;
float fft_input1[FFT_LENGTH*2];
float fft_output1[FFT_LENGTH];
float fft_input2[FFT_LENGTH*2];
float fft_output2[FFT_LENGTH];

float MAX1;      //FFT处理后的最大值
float MAX2;
uint16_t index_MAX1; //最大值的下标
uint16_t index_MAX2;
bool half_cplt = true;
float vol_data[FFT_LENGTH*2];//这个是转换之后的电压值
// 初始化汉宁窗
void init_window() {
    for(int i=0; i<FFT_LENGTH; i++) {
        hanning_window[i] = 0.5f * (1 - cosf(2 * PI * i / (FFT_LENGTH - 1)));
    }
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
float phase_diff(float phase1, float phase2);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart4, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}//串口重定向
char *Float1String(float value){
    char* str = str1;
    int Head = abs((int)value);

    float p = fabs(value - (int)value);
    char zero[5] = "";
    for(int i = 0; i < 1; i++)
    {
        p *= 10;
        if(p<1)
            sprintf(zero, "%s0", zero);
        else
            break;
    }
    int Point = abs((int)((fabs(value) - Head)*100));
    if(value < 0) sprintf(str, "-%d.%s%d", Head, zero, Point);
    else sprintf(str, "%d.%s%d", Head, zero, Point);

    return str;
}
void SineWave_Data( int num,uint16_t *D,float U) {
    int i;
    for (i = 0; i < num; i++) {
        D[i] = (uint16_t ) ((U * sin((1.0 * i / (num - 1)) * 2 * PI) + U + 1.1) * 4095 / 3.3);
    }
}
void High_Data( int num,uint16_t *D) {
    int i;
    for (i = 0; i < num; i++) {
        D[i] = (uint16_t ) (4095*3/3.3);
//        D[i] = (uint16_t ) ((U * sin((1.0 * i / (num - 1)) * 2 * PI) + U + 0.5) * 4095 / 3.3);
    }
}

bool data_ready = false;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(fft_complete_flag) {
        HAL_ADC_Stop_DMA(&hadc1);
        memcpy(out_data , adc_data , sizeof(uint16_t) * FFT_LENGTH * 2);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_data, FFT_LENGTH * 2);
        data_ready = true; // 标记数据准备就绪
    }

}
void data_separation(){
    for (int i = 0; i < FFT_LENGTH; i++)
    {
        fft_data1[i]=out_data[2*i];
    }
    for (int i = 0; i < FFT_LENGTH; i++)
    {
        fft_data2[i]=out_data[2*i+1];
    }//分两个通道的数据
}
float phase_diff(float phase1,float phase2){
    float phase_diff_t;
    phase_diff_t=phase1-phase2;
    if(phase_diff_t > 180) phase_diff_t -= 360;
    if(phase_diff_t < -180) phase_diff_t += 360;
    return phase_diff_t;
}
typedef struct {
    float Q;         // 过程噪声的方??
    float R;         // 测量噪声的方??
    float x_last;    // 上一时刻的状态估?????????
    float p_last;    // 上一时刻的估计误差协方差
} KalmanFilter;
void kalman_filter_init(KalmanFilter *filter, float Q, float R) {
    filter->Q = Q;
    filter->R = R;
    filter->x_last = 0.0f;
    filter->p_last = 1.0f;
}
float kalman_filter_update(KalmanFilter *filter, float measurement) {
    float x_pred = filter->x_last;      // 当前时刻的状态预?????????
    float p_pred = filter->p_last + filter->Q;  // 当前时刻的预测误差协方差

    float kg = p_pred / (p_pred + filter->R);  // 卡尔曼增??

    float x_now = x_pred + kg * (measurement - x_pred);  // 当前时刻的状态更?????????
    float p_now = (1 - kg) * p_pred;                      // 当前时刻的更新误差协方差

    filter->x_last = x_now;  // 更新?????????估计???
    filter->p_last = p_now;  // 更新估计误差协方??

    return x_now;
}
KalmanFilter freq_filter, vpp_filter, dphase_filter;
void Switch(int k){
    switch(k){
        case 0:
            HAL_GPIO_WritePin(switch1_GPIO_Port,switch1_Pin,RESET);
            HAL_GPIO_WritePin(switch2_GPIO_Port,switch2_Pin,RESET);
            break;

        case 1:
            HAL_GPIO_WritePin(switch1_GPIO_Port,switch1_Pin,SET);
            HAL_GPIO_WritePin(switch2_GPIO_Port,switch2_Pin,RESET);
            break;

        case 2:
            HAL_GPIO_WritePin(switch1_GPIO_Port,switch1_Pin,SET);
            HAL_GPIO_WritePin(switch2_GPIO_Port,switch2_Pin,SET);
            break;
    }
}
void gpio_init(){
    HAL_GPIO_WritePin(switch1_GPIO_Port,switch1_Pin,RESET);
    HAL_GPIO_WritePin(switch2_GPIO_Port,switch2_Pin,RESET);
}
int Judge(){
    float avg=0;
    int k=0;
    int s1=0,s2=0,s3=0;

    Switch(0);
    HAL_Delay(30);
    for(int i=0;i<20;i++){
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2,20);
        avg+=HAL_ADC_GetValue(&hadc2)/65535.*3.3;


    }
    avg/=20.;

    if(avg<0.3){
        s1++;
    }
    else if(avg>2.5){
        s2++;
    }
    else{
        s3++;
    }

    Switch(1);
    HAL_Delay(100);
    avg=0;
    for(int i=0;i<20;i++){
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2,20);
        avg+=HAL_ADC_GetValue(&hadc2)/65535.*3.3;

    }
    avg/=20.;
    if(avg<0.3){
        s1++;
    }
    else if(avg>2.4){
        s2++;
    }
    else{
        s3++;
    }

    Switch(2);
    HAL_Delay(60);
    avg=0;
    for(int i=0;i<20;i++){
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2,20);
        avg+=HAL_ADC_GetValue(&hadc2)/65535.*3.3;

    }
    avg/=20.;
    if(avg<0.3){
        s1++;
    }
    else if(avg>2.4){
        s2++;
    }
    else{
        s3++;
    }

    if(s1>s2 && s1>s3){
        return 0;//电容
    }
    else if(s2>s3 && s2>s1){
        return 1;//电感
    }
    else{
        return 2;//电阻
    }
}
float calculation(int Z_type, float calc_dif_phase){
    float avg1=0,avg2=0;
    float Kc=4.0;
    float dif_phase_radian = calc_dif_phase / 180.0 * Pi;
    float conductor_factor = 27.476;
    float origin_value;float value_t;
    switch(Z_type){
        case 0:
            return (float)(Kc/tan(dif_phase_radian));

        case 1:
            origin_value = (float)( conductor_factor *tan(dif_phase_radian));
            origin_value += origin_value * origin_value * 0.000605 + origin_value * 0.0507;
            return origin_value;

        case 2:
                Switch(0);
                HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
                HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *) high, SAMPLE, DAC_ALIGN_12B_R);

                for (int i = 0; i < 20; i++) {
                    HAL_ADC_Start(&hadc1);
                    HAL_ADC_PollForConversion(&hadc1,20);
                    avg1+=HAL_ADC_GetValue(&hadc1)/65535.*3.3;
                    HAL_ADC_Stop(&hadc1);
                    HAL_ADC_Start(&hadc2);
                    HAL_ADC_PollForConversion(&hadc2,20);
                    avg2+=HAL_ADC_GetValue(&hadc2)/65535.*3.3;
                    HAL_ADC_Stop(&hadc2);
                }
                avg1 /= 20.;
                avg2 /= 20.;
                value_t = avg1 * 8.9 / avg2;
                if (value_t < 100) {
                    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
                    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *) DualSine12bit, SAMPLE, DAC_ALIGN_12B_R);
                    return value_t;
                }//挡位1

                Switch(1);
                for (int i = 0; i < 20; i++) {
                    HAL_ADC_Start(&hadc1);
                    HAL_ADC_PollForConversion(&hadc1,20);
                    avg1+=HAL_ADC_GetValue(&hadc1)/65535.*3.3;
                    HAL_ADC_Stop(&hadc1);
                    HAL_ADC_Start(&hadc2);
                    HAL_ADC_PollForConversion(&hadc2,20);
                    avg2+=HAL_ADC_GetValue(&hadc2)/65535.*3.3;
                    HAL_ADC_Stop(&hadc2);
                }
                avg1 /= 20.;
                avg2 /= 20.;
                value_t = avg1 * 997 / avg2;
                if (value_t < 10000) {
                    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
                    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *) DualSine12bit, SAMPLE, DAC_ALIGN_12B_R);
                    return value_t;
                }//挡位2

                Switch(1);
                for (int i = 0; i < 20; i++) {
                    HAL_ADC_Start(&hadc1);
                    HAL_ADC_PollForConversion(&hadc1,20);
                    avg1+=HAL_ADC_GetValue(&hadc1)/65535.*3.3;
                    HAL_ADC_Stop(&hadc1);
                    HAL_ADC_Start(&hadc2);
                    HAL_ADC_PollForConversion(&hadc2,20);
                    avg2+=HAL_ADC_GetValue(&hadc2)/65535.*3.3;
                    HAL_ADC_Stop(&hadc2);
                }
                avg1 /= 20.;
                avg2 /= 20.;
                value_t = avg1 * 1997 / avg2;
                HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_data, FFT_LENGTH*2);
                HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
                HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *) DualSine12bit, SAMPLE, DAC_ALIGN_12B_R);
                return value_t;
                //挡位3
            }





}
void display(float value){
    OLED_NewFrame();
    OLED_PrintString(0,0,"type:", &font16x16, OLED_COLOR_NORMAL);
    switch(type){
        case 0:
            OLED_PrintString(48,0,"capacity", &font16x16, OLED_COLOR_NORMAL);
            OLED_PrintString(0,16,"value:", &font16x16, OLED_COLOR_NORMAL);
            OLED_PrintString(48,16, Float1String(value), &font16x16, OLED_COLOR_NORMAL);
            break;
        case 1:
            OLED_PrintString(48,0,"inductance", &font16x16, OLED_COLOR_NORMAL);
            OLED_PrintString(0,16,"value:", &font16x16, OLED_COLOR_NORMAL);
            OLED_PrintString(48,16,Float1String(value), &font16x16, OLED_COLOR_NORMAL);
            OLED_PrintString(0,32,"Q:", &font16x16, OLED_COLOR_NORMAL);
            OLED_PrintString(32,32," ", &font16x16, OLED_COLOR_NORMAL);
            break;
        case 2:
            OLED_PrintString(48,0,"resistance", &font16x16, OLED_COLOR_NORMAL);
            OLED_PrintString(0,16,"value:", &font16x16, OLED_COLOR_NORMAL);
            OLED_PrintString(48,16,Float1String(value), &font16x16, OLED_COLOR_NORMAL);
            break;
    }
//    OLED_PrintString(48,0, Float1String(kalman_filter_update(&dphase_filter,value)), &font16x16, OLED_COLOR_NORMAL);

    OLED_ShowFrame();

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
   str1 = (char*)malloc(10);

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM17_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

    kalman_filter_init(&freq_filter, 0.005, 0.4);
    kalman_filter_init(&vpp_filter, 0.01, 0.4);
    kalman_filter_init(&dphase_filter, 0.02, 2.0);

    OLED_Init();

    gpio_init();
    arm_cfft_radix4_instance_f32 scfft;
    arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);
    HAL_TIM_Base_Start(&htim1);
    SineWave_Data(SAMPLE,DualSine12bit,0.7);
    High_Data(SAMPLE, high);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,(uint32_t *)high,SAMPLE,DAC_ALIGN_12B_R);
    type=Judge();
    HAL_DAC_Stop_DMA(&hdac1,DAC_CHANNEL_1);
    HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,(uint32_t *)DualSine12bit,SAMPLE,DAC_ALIGN_12B_R);
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_data, FFT_LENGTH*2);


  /* USER CODE END 2 */
    switch(type){
        case 0:
            Switch(1);
        case 1:
            Switch(0);
        case 2:
            Switch(0);
    }
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        fft_complete_flag=1;

        if(data_ready) {
            data_separation();
            data_ready = false;

            fft_complete_flag = 0;

            char msg[400] = {0};

            float avg_1 = 0, avg_2 = 0;
            for (int i = 0; i < FFT_LENGTH; i++) {
                avg_1 += fft_data1[i];
                avg_2 += fft_data2[i];
            }//????????????A1,A0??
            avg_1 /= 1023.0;
            avg_2 /= 1023.0;
//
//
//            for (int i = 0; i < FFT_LENGTH; i++) {
//                sprintf(msg, "%s%s", msg, Float1String(fft_data1[i] - avg_1));
//                sprintf(msg, "%s,%s\r\n ", msg, Float1String(fft_data2[i] - avg_2));
//            }//打印数据（串口A1,A0）
//
//            printf("%s", msg);
//
//      memset(msg, 0, sizeof (msg));
//
//      for (int i = FFT_LENGTH/2; i < FFT_LENGTH; i++)
//      {
//          sprintf(msg, "%s%s,%s \r\n",msg, Float1String(fft_data1[i]), Float1String((fft_data2[i])));
//      }//打印数据（串口A1,A0）
//

//      float vpp=calculate_rms(fft_input1,FFT_LENGTH)* sqrtf(2.0f);   // 正弦波

            for (int i = 0; i < FFT_LENGTH; i++) {
                fft_input1[i * 2] = ((float) fft_data1[i] - avg_1) / 65535.0 * 3.3;  // 实部
                fft_input1[i * 2 + 1] = 0;              // 虚部

            }
            // 对通道1执行FFT
            arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_input1, 0, 1);  // 复数FFT
            arm_cmplx_mag_f32(fft_input1, fft_output1, FFT_LENGTH);    // 计算幅值谱
            fft_output1[0] = fft_output1[0] / FFT_LENGTH;// 提取直流量 第一个数据除以采样点数为直流分量

            for (int i = 1; i < FFT_LENGTH; i++) {
                fft_output1[i] /= (FFT_LENGTH / 2);
            }//归一化处理
            for (int i = 1; i < FFT_LENGTH / 2; i++) {
                if (fft_output1[i] > MAX1) {
                    MAX1 = fft_output1[i];
                    index_MAX1 = i;
                }
            } // 找到基频位置（幅值最大的点）
            // 通道1相位计算
            float phase1 = atan2f(fft_input1[2 * index_MAX1 + 1], fft_input1[2 * index_MAX1]) * 180.0f / PI;


            for (int i = 0; i < FFT_LENGTH; i++) {
                fft_input2[i * 2] = ((float) fft_data2[i] - avg_2) / 65535. * 3.3;  // 实部
                fft_input2[i * 2 + 1] = 0;              // 虚部

            }
            // 对通道2执行FFT

            arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_input2, 0, 1);  // 复数FFT
            arm_cmplx_mag_f32(fft_input2, fft_output2, FFT_LENGTH);    // 计算幅值谱
            fft_output2[0] = fft_output2[0] / FFT_LENGTH;// 提取直流量 第一个数据除以采样点数为直流分量

            for (int i = 1; i < FFT_LENGTH; i++) {
                fft_output2[i] /= (FFT_LENGTH / 2);
            }//归一化处理
            for (int i = 1; i < FFT_LENGTH / 2; i++) {


                if (fft_output2[i] > MAX2) {
                    MAX2 = fft_output2[i];
                    index_MAX2 = i;
                }
            }// 找到基频位置（幅值最大的点）

            float phase2 = atan2f(fft_input2[2 * index_MAX2 + 1], fft_input2[2 * index_MAX2]) * 180.0f / PI;
            dif_phase = phase_diff(phase1, phase2);

            if (index_MAX2 > 20) {
                dif_phase += 7.0;
            } else {
                dif_phase += -1.8;
            }

//      for(int i=0;i<FFT_LENGTH;i++){
//          printf("%s", Float1String(fft_output1[i]));
//          printf(",%s\r\n", Float1String(fft_output2[i]));
//      }

            float ka_dif_phase = kalman_filter_update(&dphase_filter,dif_phase);
            value = calculation(type, ka_dif_phase);
            display(value);

            printf("%s\r\n", Float1String(ka_dif_phase));

        }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL3.PLL3M = 32;
  PeriphClkInitStruct.PLL3.PLL3N = 129;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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

#ifdef  USE_FULL_ASSERT
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
