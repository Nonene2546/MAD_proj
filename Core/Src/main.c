/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "lptim.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

#include "ILI9341_GFX.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_Touchscreen.h"

#include "OV7670.h"

#include "color_conversion.h"

#include "network.h"
#include "network_data.h"

//#include "ai_datatypes_defines.h"
//#include "ai_platform.h"
//#include "network.h"
//#include "network_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;

extern I2C_HandleTypeDef hi2c2;

extern LPTIM_HandleTypeDef hlptim1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define IMAGE_SIZE 128
#define MAX_OUT_CHANNEL 84
#define PRED_BOUND 64

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))


struct YoloOut {
	uint16_t x;
	uint16_t y;
	uint16_t width;
	uint16_t height;
	float class_prob;
	uint8_t class_pred;
};

bool keep_structs[MAX_OUT_CHANNEL] = { false };
float scales = 0.05172783136367798;
int8_t zero_points = -124;
float pred_threshold = 0.45;

struct YoloOut *postprocess(struct YoloOut *yolo_outs);

void unpack_output_to_struct(struct YoloOut *yolo_outs);

void ai_data_process();

ai_i8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
ai_i8 out_data[AI_NETWORK_OUT_1_SIZE_BYTES];

ai_buffer *ai_input;
ai_buffer *ai_output;

Camera_settings OV7670_settings={
		QCIF, 		//Resolution
		RGB565, 	//Format
		NORMAL, 	//Effect
		ON,			//AEC
		ON, 		//AGC
		ON, 		//AWB
		OFF,		//Color bar
		OFF,		//vertical flip
		OFF,			//Horizontal flip
		OFF,		//Night mode
		OFF,		//ASC
		ON,			//De-noise
		ON,			//Banding filter
		HISTOGRAM,	//AEC algorithm
		NORMAL_FPS, //Min. fps in night mode
		F_AUTO,		//Auto detect banding freq.
		256, 		//Exposure - 2 bytes
		0, 			//Gain	[0-7]=[1-128]
		50,		//Brightness - byte
		64, 		//Contrast - byte
		80, 		//Saturation - byte
		20,			//Sharpness	- [0-31]
		0,			//De-noise strength - byte
		x16, 		//Gain ceiling
		25, 		//R channel gain - byte
		25, 		//G channel gain - byte
		0			//B channel gain - byte
};

uint16_t width, height;
uint8_t format;

_Bool frame_transfer_cplt_flag = 0;
uint32_t buffer_image[176 * 72] = {0}; //max resolution is CIF
uint8_t screen_image[320*240*2] = {0};
int8_t image_data[128 * 128 * 3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart3, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
}

int yolo_out_comparer(const void *a, const void *b) {
    const struct YoloOut *box_1 = (struct YoloOut *)a;
    const struct YoloOut *box_2 = (struct YoloOut *)b;

    if (box_1->class_prob > box_2->class_prob) {
        return -1;
    }
    else if (box_1->class_prob < box_2->class_prob) {
        return 1;
    }
    else {
        return 0;
    }
}


float cal_IOU(const struct YoloOut *ref, const struct YoloOut *candi) {
    //หาซ้ายบน
	uint8_t x1_inter = MAX(ref->x - ref->width / 2, candi->x - candi->width / 2);
	uint8_t y1_inter = MAX(ref->y - ref->height / 2, candi->y - candi->height / 2);

    //หาขวาล่าง
	uint8_t x2_inter = MIN(ref->x + ref->width / 2, candi->x + candi->width / 2);
	uint8_t y2_inter = MIN(ref->y + ref->height / 2, candi->y + candi->height / 2);

	uint8_t inter_width = MAX(0, x2_inter - x1_inter);
	uint8_t inter_height = MAX(0, y2_inter - y1_inter);

    float intersection_area = 0;

    if (inter_width && inter_height) {
        intersection_area = inter_width * inter_height;
    }
    else {
        return 0;
    }

    uint16_t ref_area = ref->width * ref->height;
    uint16_t candi_area = candi->width * candi->height;

    float iou = intersection_area / (ref_area + candi_area - intersection_area);
    return iou;
}

void display_yolo_struct(struct YoloOut output) {
    printf("x: %u, y: %u, w: %u, h: %u, class_prob: %f, class_pred: %u\r\n",
            output.x,
            output.y,
            output.width,
            output.height,
            output.class_prob,
            output.class_pred
        );
}

void display_yolo_outs(struct YoloOut *yolo_outs) {
    for (uint8_t i = 0; i < MAX_OUT_CHANNEL; i++) {
        if (keep_structs[i]) {
            display_yolo_struct(yolo_outs[i]);
        }
    }
}

float dequantize(int value) {
	return scales*(value - zero_points);
}

void unpack_output_to_struct(struct YoloOut *yolo_outs) {
    for (int i = 0; i < MAX_OUT_CHANNEL; i++) {
        struct YoloOut box;

        box.x = (uint16_t)(dequantize(out_data[i]) * 128);
        box.y = (uint16_t)(dequantize(out_data[i + MAX_OUT_CHANNEL]) * 128);
        box.width = (uint16_t)(dequantize(out_data[i + MAX_OUT_CHANNEL*2]) * 128);
        box.height = (uint16_t)(dequantize(out_data[i + MAX_OUT_CHANNEL*3]) * 128);

        float class_1 = dequantize(out_data[i + MAX_OUT_CHANNEL*4]);
        float class_2 = dequantize(out_data[i + MAX_OUT_CHANNEL*5]);

        if (class_1 > class_2) {
            box.class_prob = class_1;
            box.class_pred = 0;
        }
        else {
            box.class_prob = class_2;
            box.class_pred = 1;
        }

        yolo_outs[i] = box;
    }
}

struct YoloOut *get_servo_output(struct YoloOut *ref, struct YoloOut *cur) {
    if (
        cur->x >= PRED_BOUND &&
        cur->x > ref->x ||
        (cur->x == ref->x && cur->width > ref->width)
    ) {
        return cur;
    }

    return ref;
}


bool NMS_optimize(
    struct YoloOut *yolo_outs,
    struct YoloOut *current_box,
	uint8_t idx,
    float iou_threshold
) {
    if (current_box->class_prob <= pred_threshold) {
      keep_structs[idx] = false;
      return false;
    }

    float area = current_box->width*current_box->height;
    if(area <= 1200 || area >= 8100){
    	keep_structs[idx] = false;
    	return false;
    }

    bool flag = 1;
    for (uint8_t i = 0; i < MAX_OUT_CHANNEL; i++) {
        if (!keep_structs[i]) {
            continue;
        }

        struct YoloOut *ref_box = &yolo_outs[i];

        if (current_box->class_prob <= pred_threshold ||
            cal_IOU(ref_box, current_box) >= iou_threshold
        ) {
            flag = 0;
            keep_structs[idx] = false;
            break;
        }

    }

    return flag;
}


struct YoloOut *postprocess(struct YoloOut *yolo_outs) {
    qsort(yolo_outs, MAX_OUT_CHANNEL, sizeof(struct YoloOut), yolo_out_comparer);

    struct YoloOut *servo_out = &yolo_outs[0];

    if (servo_out->class_prob <= pred_threshold) {
    	for (uint8_t i = 0; i < MAX_OUT_CHANNEL; i++) {
    		keep_structs[i] = false;
    	}

    	return NULL;
    }

    for (uint8_t i = 1; i < MAX_OUT_CHANNEL; i++) {
        struct YoloOut *current_box = &yolo_outs[i];

        if (NMS_optimize(yolo_outs, current_box, i, 0.3)) {
            keep_structs[i] = true;
            servo_out = get_servo_output(yolo_outs, current_box);
        }
    }

    return servo_out;
}

void draw_bounding_box(struct YoloOut *yolo_outs, float row_scaler, float col_scaler) {

	for (uint8_t i = 0; i < MAX_OUT_CHANNEL; i++) {
		if(keep_structs[i]) {

			uint16_t color = yolo_outs[i].class_pred ? RED : GREEN;

			uint16_t yolo_width = (yolo_outs[i].width*row_scaler)/2;
			uint16_t x_pos = yolo_outs[i].x*row_scaler;

			uint16_t yolo_height = (yolo_outs[i].height*col_scaler)/2;
			uint16_t y_pos = yolo_outs[i].y*col_scaler;

			ILI9341_Draw_Hollow_Rectangle_Coord(
					x_pos - yolo_width,
					y_pos - yolo_height,
					x_pos + yolo_width,
					y_pos + yolo_height,
					color
			);
		}
	}
}

void plt_screen(struct YoloOut *yolo_outs) {

    uint32_t px=0;
	uint16_t half_width = 88;

	float row_scaler = 128.0/240;
	float col_scaler = 64.0/160;

	for (uint32_t row = 0; row < 240; row++) {
		uint8_t row_pos = row_scaler * row;

//	  px = 640*row;

	  for (uint32_t col = 0; col < 160; col++) {
		  uint8_t col_pos = col_scaler * col;

		  uint32_t cur_pixel = buffer_image[row_pos*half_width + col_pos];

		  screen_image[px] = (cur_pixel >> 24) & 0xFF;
		  screen_image[px + 1] = (cur_pixel >> 16) & 0xFF;
		  screen_image[px + 2] = (cur_pixel >> 8) & 0xFF;
		  screen_image[px + 3] = cur_pixel & 0xFF;

		  px += 4;
	  }
	}

	ILI9341_Draw_Image((const char*) screen_image, SCREEN_HORIZONTAL_1);

	draw_bounding_box(yolo_outs, 320.0/128, 240.0/128);

//	ILI9341_Draw_Vertical_Line(80, 0, 240, CYAN);

}

void ai_data_process() {
	  struct YoloOut yolo_outs[MAX_OUT_CHANNEL];
//
	  unpack_output_to_struct(yolo_outs);
//
	  struct YoloOut *servo_output = postprocess(yolo_outs);

	  if (servo_output != NULL) {
		  printf("servo out: ");
		  display_yolo_struct(*servo_output);
		  display_yolo_outs(yolo_outs);
		  printf("\n");

		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, servo_output->class_pred);
	  }

	  plt_screen(yolo_outs);
}

void preprocess() {
	const volatile uint8_t half_width = 88;
	const volatile uint8_t half_image = IMAGE_SIZE/2;

    for (uint16_t offset = 0; offset < IMAGE_SIZE*half_image; offset++) {
    		uint16_t px = offset*6;
    		uint32_t pixel_pointer = (offset/half_image)*half_width + offset%half_image;

            uint16_t upper = (buffer_image[pixel_pointer] >> 16) ;
            uint16_t lower = buffer_image[pixel_pointer] & 0xFFFF;

            int rgb_upper[3] = {
    			( (float) ((upper >> 11) & 0x1F)/ 31) * 255 - 128,
				( (float) ((upper >> 5) & 0x3F)/ 63) * 255 - 128,
				( (float) (upper & 0x1F)/31 * 255 ) - 128
            };

            int rgb_lower[3] = {
				( (float) ((lower >> 11) & 0x1F)/ 31) * 255 - 128,
				( (float) ((lower >> 5) & 0x3F)/ 63) * 255 - 128,
				( (float) (lower & 0x1F)/31 * 255 ) - 128
            };

            image_data[px] = rgb_upper[0];
            image_data[px + 1] = rgb_upper[1];
            image_data[px + 2] = rgb_upper[2];

            image_data[px + 3] = rgb_lower[0];
			image_data[px + 4] = rgb_lower[1];
			image_data[px + 5] = rgb_lower[2];
    }

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_LPTIM1_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */


  OV7670_Power(DISABLE);

  ILI9341_Init();

  //I2C2 is used for SCCB and LPTIM1 for XLCK generation
  OV7670_Init(&hdcmi, &hdma_dcmi, &hi2c2, &hlptim1);

  OV7670_PowerUp();
  OV7670_UpdateSettings(OV7670_settings);
  OV7670_SetFrameRate(XCLK_DIV(1), PLL_x4);
  HAL_Delay(10);

  OV7670_Start(SNAPSHOT, buffer_image);
  OV7670_getImageInfo(&width,&height,&format);

  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);

  HAL_Delay(100);

  ai_handle network = AI_HANDLE_NULL;
  	    ai_error err;
  	    ai_network_report report;

  	    /** @brief Initialize network */
  	    const ai_handle acts[] = { activations };
  	    err = ai_network_create_and_init(&network, acts, NULL);
  	    if (err.type != AI_ERROR_NONE) {
  	        printf("ai init_and_create error\r\n");
  	        return -1;
  	    }

  	    /** @brief {optional} for debug/log purpose */
  	    if (ai_network_get_report(network, &report) != true) {
  	        printf("ai get report error\r\n");
  	        return -1;
  	    }

  	    printf("Model name      : %s\r\n", report.model_name);
  	    printf("Model signature : %s\r\n", report.model_signature);

  	    ai_input = &report.inputs[0];
  	    ai_output = &report.outputs[0];
  	    printf("input[0] : (%lu, %lu, %lu)\r\n", AI_BUFFER_SHAPE_ELEM(ai_input, AI_SHAPE_HEIGHT),
  	                                        AI_BUFFER_SHAPE_ELEM(ai_input, AI_SHAPE_WIDTH),
  	                                        AI_BUFFER_SHAPE_ELEM(ai_input, AI_SHAPE_CHANNEL));
  	    printf("output[0] : (%ld, %ld, %ld)\r\n", AI_BUFFER_SHAPE_ELEM(ai_output, AI_SHAPE_HEIGHT),
  	                                         AI_BUFFER_SHAPE_ELEM(ai_output, AI_SHAPE_WIDTH),
  	                                         AI_BUFFER_SHAPE_ELEM(ai_output, AI_SHAPE_CHANNEL));

  	    /** @brief Perform inference */
  	    ai_i32 n_batch;

  	    /** @brief Create the AI buffer IO handlers
  	     *  @note  ai_inuput/ai_output are already initilaized after the
  	     *         ai_network_get_report() call. This is just here to illustrate
  	     *         the case where get_report() is not called.
  	     */
  	    ai_input = ai_network_inputs_get(network, NULL);
  	    ai_output = ai_network_outputs_get(network, NULL);

  	    /** @brief Set input/output buffer addresses */
  	    ai_input[0].data = AI_HANDLE_PTR(image_data);
  	    ai_output[0].data = AI_HANDLE_PTR(out_data);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (frame_transfer_cplt_flag) {
		  preprocess();


		    /** @brief Perform the inference */
		    n_batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
		    if (n_batch != 1) {
		        err = ai_network_get_error(network);
		        printf("ai run error %d, %d\r\n", err.type, err.code);
		      return -1;
		    }
	//
	//	    /** @brief Post-process the output results/predictions */
		    printf("Inference output..\r\n");
	//
		    ai_data_process();

		    frame_transfer_cplt_flag = 0;

			OV7670_Start(SNAPSHOT, buffer_image);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_4);
}

/* USER CODE BEGIN 4 */

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
