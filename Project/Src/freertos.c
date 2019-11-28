/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "usbd_cdc_if.h"
#include "stm32f4xx_it.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "spi.h"
#include "i2c.h"
#include "camera.h"
#include "structs.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osMessageQId pirQueueHandle;
osMessageQId guiQueueHandle;
osMessageQId resetQueueHandle;
osMutexId myMutex01Handle;
osSemaphoreId cameraSemaphoreHandle;

/* USER CODE BEGIN Variables */
uint8_t RxData[256];
uint32_t data_received;
uint8_t Str4Display[50];

// Task Handlers
osThreadId TaskGUIHandle;
osThreadId TaskCOMMHandle;
osThreadId TaskBTNHandle;
osThreadId TaskIntruderHandle;

osMessageQId CommQueueHandle;
osMutexId dataMutexHandle;

int currentTimeStamp = 0;


typedef struct
{
	char Source[10];
	int Value;
}data;


PIRData PIRDataInit={1};
data DataVCP={"VCP\0",2};

int resetInit = 0;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void StartTaskCOMM(void const * argument);
void StartTaskBTN(void const * argument);
void StartTaskGUI(void const * argument);
void StartTaskIntruder(void const * argument);

/* USER CODE END FunctionPrototypes */

void captureImage();


/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {

	osMutexDef(dataMutex);
	dataMutexHandle = osMutexCreate(osMutex(dataMutex));

	osSemaphoreDef(cameraSemaphore);
	cameraSemaphoreHandle = osSemaphoreCreate(osSemaphore(cameraSemaphore), 1);

	osMessageQDef(guiQueue, 1, &PIRDataInit);
	guiQueueHandle = osMessageCreate(osMessageQ(guiQueue), NULL);

	osMessageQDef(resetQueue, 1, &resetInit);
	resetQueueHandle = osMessageCreate(osMessageQ(resetQueue), NULL);


	osThreadDef(TaskCOMM, StartTaskCOMM, osPriorityHigh, 0, 128);
	TaskCOMMHandle = osThreadCreate(osThread(TaskCOMM), NULL);

	osThreadDef(TaskBTN, StartTaskBTN, osPriorityNormal, 0, 128);
	TaskBTNHandle = osThreadCreate(osThread(TaskBTN), NULL);

	osThreadDef(TaskGUI, StartTaskGUI, osPriorityAboveNormal, 0, 128);
	TaskGUIHandle = osThreadCreate(osThread(TaskGUI), NULL);

	osThreadDef(TaskIntruder, StartTaskIntruder, osPriorityRealtime, 0, 128);
	TaskIntruderHandle = osThreadCreate(osThread(TaskIntruder), NULL);

	osMessageQDef(pirQueue, 1, &PIRDataInit);
	pirQueueHandle = osMessageCreate(osMessageQ(pirQueue), NULL);

	/* Comm QUEUE, don't delete */
	osMessageQDef(CommQueue, 1, &DataVCP);
	CommQueueHandle = osMessageCreate(osMessageQ(CommQueue), NULL);
}


void StartTaskCOMM(void const * argument)
{

  osEvent vcpValue;

  while(1)
  {
	  vcpValue = osMessageGet(CommQueueHandle, osWaitForever);
	  osMutexWait(dataMutexHandle, 0);
	  memcpy(Str4Display,(char *)(((data *)vcpValue.value.p)->Value), data_received+1);
	  osMutexRelease(dataMutexHandle);
	  printf("CommTask received: %s\n\r", Str4Display);

	  // if received 'c', capture a picture from camera
	  if (Str4Display[0] == 'c')
	  {
		  printf("hello world\n\r");
	  }
  }
}

void StartTaskGUI(void const * argument)
{
  uint8_t buf[sizeof(Str4Display)];
  osEvent receivedData;
  osEvent resetMessage;
  PIRData* pirMessage;

  const int intruderMessageLine = 3;
  char intruderMessage[100];

  const int currentTimeLine = 2;
  char currentTimeMessage[200] = {};
  char alertMessage[200] = {};
  int currentTime = 0;
  int intruderTime = -1;

  int intruderDetected = 0;

  BSP_LCD_Clear(LCD_COLOR_ORANGE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
  BSP_LCD_SetFont(&Font12);


  while(1)
  {
	  currentTimeStamp++;
	  currentTime = currentTimeStamp;

	  resetMessage = osMessageGet(resetQueueHandle, 200);
	  receivedData = osMessageGet(guiQueueHandle, 200);



	  if (resetMessage.status == 0x10) {
		  intruderDetected = 0;
		   BSP_LCD_Clear(LCD_COLOR_ORANGE);
		   BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		   BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
		   BSP_LCD_SetFont(&Font12);
		  sprintf(intruderMessage, "Resetting Alert...");
		  intruderTime = -1;
	  }  else if (receivedData.status == 0x10 && !intruderDetected) {
		  intruderDetected = 1;
		  pirMessage = (PIRData *) receivedData.value.p;
		  intruderTime = pirMessage->timeStamp;
	  }

	  itoa(currentTime, currentTimeMessage, 10);

	  BSP_LCD_ClearStringLine(intruderMessageLine);
	  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

	  BSP_LCD_DisplayStringAtLine(1, "Current Time:");
	  BSP_LCD_DisplayStringAtLine(currentTimeLine, &currentTimeMessage);

	  if (!intruderDetected) {
		  BSP_LCD_DisplayStringAtLine(3, "No Intruder...");
	  } else {
		  BSP_LCD_DisplayStringAtLine(3, "Intruder Alert! Timestamp:");
		  itoa(intruderTime, intruderMessage, 10);
		  BSP_LCD_DisplayStringAtLine(4, &intruderMessage);
	  }




	  osDelay(800);
  }
}

void StartTaskBTN(void const * argument)
{
  uint8_t count_press = 0;

  while(1)
  {
	  if(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN) == GPIO_PIN_SET){
		  printf("TaskBTN Button Pressed : %i times \n\r", count_press);
		  count_press++;

		  int resetValue = 1;
		  osMessagePut(resetQueueHandle, &resetValue, osWaitForever);

		  while(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN)==GPIO_PIN_SET);
	  }

	  osDelay(10);
  }
}



void StartTaskIntruder(void const * argument) {
	osEvent receivedData;
	PIRData* pirMessage;
	PIRData messageToSend;

	while(1) {
		receivedData = osMessageGet(pirQueueHandle, osWaitForever);

		if (receivedData.status == 0x10) {
			pirMessage = (PIRData*) receivedData.value.p;
			messageToSend.timeStamp = pirMessage->timeStamp;
			osMessagePut(guiQueueHandle, &messageToSend, 200);

			captureImage();
		}

	}
}


void notifyIntruderTask() {
	PIRData message;
	message.timeStamp = currentTimeStamp;

	osMessagePut(pirQueueHandle, &message, 20);
}


void captureImage(void) {
	osSemaphoreWait(cameraSemaphoreHandle, osWaitForever);
	camera_setup();
	camera_initiate_capture();
	osSemaphoreRelease(cameraSemaphoreHandle);
}


