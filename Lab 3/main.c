
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @Modified by    : Kevin Dong and Graham Thoms for ENGG*4420 Real-Time Systems
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "gpio.h"

/* user Includes */
#include "includes.h"


/* Private variables ---------------------------------------------------------*/
uint8_t RxData[256];
uint32_t data_received = 0;
uint8_t Str4Display[100];


static  OS_TCB   StartupTaskTCB;
static  CPU_STK  StartupTaskStk[APP_CFG_STARTUP_TASK_STK_SIZE];

static  OS_TCB   CommTaskTCB;
static  CPU_STK  CommTaskStk[APP_CFG_COMM_TASK_STK_SIZE];

static  OS_TCB   BtnTaskTCB;
static  CPU_STK  BtnTaskStk[APP_CFG_BTN_TASK_STK_SIZE];

static  OS_TCB   GuiTaskTCB;
static  CPU_STK  GuiTaskStk[APP_CFG_GUI_TASK_STK_SIZE];

static  OS_TCB	 PlantTaskTCB;
static	CPU_STK	 PlantTaskStk[APP_CFG_PLANT_TASK_STK_SIZE];
OS_Q  	PlantInputQ;

static OS_TCB PIDTaskTCB;
static CPU_STK PIDTaskStack[APP_CFG_PID_TASK_STK_SIZE];
OS_Q PIDQ;

// Automode
OS_Q SetPointQ;
OS_Q GUIQ;

OS_Q   CommQ;

OS_Q OperatorInputQ;


typedef enum {
  AUTO,
  MANUAL
} Mode ;


static Mode currentControlMode = AUTO;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* user private function prototypes */
static void AppTaskCreate(void);
static void StartupTask(void  *p_arg);
static void CommTask(void  *p_arg);
static void BtnTask(void  *p_arg);
static void GuiTask(void  *p_arg);
static void PIDTask(void *p_arg);


/* added functions for Lab 3 */
static void PlantTask(void *p_arg);
static float TempToVoltage(float temp);
static float VoltageToTemp(float volt);
static void createAxis(Point frame, int xAxisSize, int yAxisSize);
static void drawLine(Point a, Point b);
static void showText(uint16_t line, char* text);
static void drawReferenceLine(Point p, int distance);



/**
  * @brief  The application entry point.
  * @retval None
  */
int main(void)
{

  OS_ERR  os_err;

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  BSP_ClkInit();                                              /* Initialize the main clock                            */
  BSP_IntInit();                                              /* Initialize the interrupt vector table.               */
  BSP_OS_TickInit();                                          /* Initialize kernel tick timer                         */

  Mem_Init();                                                 /* Initialize Memory Managment Module                   */
  CPU_IntDis();                                               /* Disable all Interrupts                               */
  CPU_Init();                                                 /* Initialize the uC/CPU services                       */
  Math_Init();                                                /* Initialize Mathematical Module                       */

  OSInit(&os_err);                                            /* Initialize uC/OS-III
         */
  if (os_err != OS_ERR_NONE) {
      while (1);
  }

  App_OS_SetAllHooks();                                       /* Set all applications hooks                           */

  OSQCreate(&CommQ,
            "Comm Queue",
             10,
            &os_err);                    					  /* Create COMM Queue         */

  // plant input queue
  OSQCreate(&PlantInputQ,
		  	"Plant Input Queue",
			20,
			&os_err);

  OSQCreate(&PIDQ,
		"PID Queue",
		20,
		&os_err);

  OSQCreate(&SetPointQ,
		"SetPoint Queue",
		20,
		&os_err);

  OSQCreate(&GUIQ,
  		"GUI Queue",
  		20,
  		&os_err);

  OSQCreate(&OperatorInputQ,
  	  "OperatorInputQ",
	  20,
	  &os_err);


  OSTaskCreate(&StartupTaskTCB,                               /* Create the startup task                              */
               "Startup Task",
                StartupTask,
                0u,
                APP_CFG_STARTUP_TASK_PRIO,
               &StartupTaskStk[0u],
                StartupTaskStk[APP_CFG_STARTUP_TASK_STK_SIZE / 10u],
                APP_CFG_STARTUP_TASK_STK_SIZE,
                0u,
                0u,
                0u,
               (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               &os_err);
  if (os_err != OS_ERR_NONE) {
      while (1);
  }

  OSStart(&os_err);                                           /* Start multitasking (i.e. give control to uC/OS-III)  */

  while (DEF_ON) {}                                            /* Should Never Get Here.                               */

}

/**********************************************************************************************************
*                                            STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
* Arguments   : p_arg   is the argument passed to 'StartupTask()' by 'OSTaskCreate()'.
* Returns     : none
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  StartupTask (void *p_arg)
{
  OS_ERR  os_err;
  (void)p_arg;

  OS_TRACE_INIT();                                            /* Initialize the uC/OS-III Trace recorder              */

  BSP_OS_TickEnable();                                        /* Enable the tick timer and interrupt                  */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();

  BSP_LED_Init();

  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();

  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_WHITE);

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&os_err);                            /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset();
#endif

  AppTaskCreate();                                            /* Create Application tasks                             */

  while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
      BSP_LED_Toggle(0);
      OSTimeDlyHMSM(0u, 0u, 1u, 0u,
                    OS_OPT_TIME_HMSM_STRICT,
                    &os_err);

  }
}

/**********************************************************************************************************
*                                          AppTaskCreate()
*
* Description : Create application tasks.
* Argument(s) : none
* Return(s)   : none
* Caller(s)   : AppTaskStart()
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppTaskCreate (void)
{
    OS_ERR  os_err;

    OSTaskCreate(&CommTaskTCB,                               /* Create the comm task                              */
                 "Comm Task",
                  CommTask,
                  0u,
                  APP_CFG_COMM_TASK_PRIO,
                 &CommTaskStk[0u],
                  CommTaskStk[APP_CFG_COMM_TASK_STK_SIZE / 10u],
                  APP_CFG_COMM_TASK_STK_SIZE,
                  2u,										// maximum messages to send to this task
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
    if (os_err != OS_ERR_NONE) {
        while (1);
    }

    OSTaskCreate(&BtnTaskTCB,                               /* Create the comm task                              */
                 "Button Task",
                  BtnTask,
                  0u,
                  APP_CFG_BTN_TASK_PRIO,
                 &BtnTaskStk[0u],
                  BtnTaskStk[APP_CFG_BTN_TASK_STK_SIZE / 10u],
                  APP_CFG_BTN_TASK_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
    if (os_err != OS_ERR_NONE) {
        while (1);
    }

    OSTaskCreate(&GuiTaskTCB,                               /* Create the comm task                              */
                 "GUI Task",
                  GuiTask,
                  0u,
                  APP_CFG_GUI_TASK_PRIO,
                 &GuiTaskStk[0u],
                  GuiTaskStk[APP_CFG_GUI_TASK_STK_SIZE / 10u],
                  APP_CFG_GUI_TASK_STK_SIZE,
                  0u,
                  0u,
                  0u,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);

    if (os_err != OS_ERR_NONE) {
        while (1);
    }

    OSTaskCreate(&PlantTaskTCB,
				"Plant Task",
				 PlantTask,
				 0u,
				 APP_CFG_PLANT_TASK_PRIO,
				&PlantTaskStk[0u],
				 PlantTaskStk[APP_CFG_PLANT_TASK_STK_SIZE / 10u],
				 APP_CFG_PLANT_TASK_STK_SIZE,
				 0u,
				 0u,
				 0u,
				(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				&os_err);

    OSTaskCreate(&PIDTaskTCB,
    		"PID Task",
			PIDTask,
			0u,
			APP_CFG_PID_TASK_PRIO,
			&PIDTaskStack[0u],
			PIDTaskStack[APP_CFG_PID_TASK_STK_SIZE / 10u],
			APP_CFG_PID_TASK_STK_SIZE,
			0u,
			0u,
			0u,
			(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
			&os_err);

    if (os_err != OS_ERR_NONE) {
        while (1);
    }

}


/*
*********************************************************************************************************
*                                            COMM TASK
*
* Description : This is an example of a Comm task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
* Arguments   : p_arg   is the argument passed to 'StartupTask()' by 'OSTaskCreate()'.
* Returns     : none
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  CommTask (void *p_arg)
{
  OS_ERR  os_err;
  void        *p_msg;
  OS_MSG_SIZE  msg_size;
  CPU_TS       ts;
  char buffer[10];
  int index = 0;
  float input = 0.0;

  (void)p_arg;

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&os_err);                            /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset();
#endif

  while (DEF_TRUE) {/* Task body, always written as an infinite loop.       */


	p_msg = OSQPend(&CommQ,
                     0,
                     OS_OPT_PEND_BLOCKING,
                    &msg_size,
                    &ts,
                    &os_err);

	char value = *((char *)p_msg);

    if(value == '\r') {
    	buffer[index] = '\0';
    	index = 0;
    	input = strtof(buffer, NULL);


    	if (currentControlMode == MANUAL){
			OSQPost(&PlantInputQ,
						(float *) &input,
						sizeof((void *)&input),
						OS_OPT_POST_FIFO,
						&os_err);
    	}
    	else{
    		OSQPost(&SetPointQ,
    					(float *) &input,
						sizeof((void *)&input),
						OS_OPT_POST_FIFO,
						&os_err);
    	}

    	OSQPost(&OperatorInputQ,
    			(float *)&input,
				sizeof((void *) &input),
				OS_OPT_POST_FIFO,
				&os_err);

    } else if(value != '\n') {
    	buffer[index] = value;
    	index++;
    }

  }
}

/**********************************************************************************************************
*                                            BUTTON TASK
*
* Description : This is an example of a Button task.
* Arguments   : p_arg   is the argument passed to 'BtnTask()' by 'OSTaskCreate()'.
* Returns     : none
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  BtnTask (void *p_arg)
{
  OS_ERR  os_err;
  unsigned int count_press = 0;

  (void)p_arg;

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&os_err);                            /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset();
#endif

  while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */
	OSTimeDlyHMSM(0u, 0u, 0u, 10u,
					OS_OPT_TIME_HMSM_STRICT,
					&os_err);
    if(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN) == GPIO_PIN_SET){

    	count_press++;
    	if(currentControlMode == AUTO) {
    		currentControlMode = MANUAL;
    	} else {
    		currentControlMode = AUTO;
    	}

    	while(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN)==GPIO_PIN_SET);
    }
  }
}

/**********************************************************************************************************
*                                            GUI TASK
*
* Description : This is the Graphical User Interfae task
* Arguments   : p_arg   is the argument passed to 'GuiTask()' by 'OSTaskCreate()'.
* Returns     : none
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/
void GuiTask(void *p_arg)
{

  OS_ERR  os_err;

  CPU_TS       ts;

  int ledXSize = BSP_LCD_GetXSize();
  int startXPosition = ledXSize * 0.05;

  int ledYSize = BSP_LCD_GetYSize();
  int startYPosition = ledYSize * 0.95;

  Point coordinateFrame = {startXPosition, startYPosition};

  int xAxisMax = ledXSize * 0.9;
  int yAxisMax = ledYSize * 0.6;

  int maxVoltage = 10;

  float yScaleFactor = yAxisMax / maxVoltage;

  Point previousPoint = {startXPosition, startYPosition};
  int xSpacing = 2;
  Point currentPoint = {0, 0};

  void *p_plant_msg;
  void *p_operator_input_msg;
  OS_MSG_SIZE msg_size = 0;

  int plantModeLine = 1;
  char plantMode[30];

  int plantOutputLine = 3;
  float plantOutput = 0.0;
  char plantOutputStr[30];

  int operatorInputLine = 2;
  float operatorInput = 0.0;
  float voltageInput = 0.0;
  float temperatureInput = 20.0;
  char operatorInputStr[50];

  int samplingTimeLine = 4;
  char samplingTimeStr[50];
  BSP_LCD_Clear(LCD_COLOR_ORANGE);

  Point referencePoint = {startXPosition, startYPosition};

  while(1)
  {
	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
	  BSP_LCD_SetFont(&Font12);

	  if(previousPoint.X > xAxisMax + startXPosition) {
	  		  BSP_LCD_Clear(LCD_COLOR_ORANGE);
	  		  previousPoint.X = startXPosition;
	  }

	  BSP_LCD_DrawHLine(0, ledYSize * 0.3, ledXSize);

	  p_plant_msg = OSQPend(&GUIQ,
			  200,
			  OS_OPT_PEND_BLOCKING,
			  &msg_size,
			  &ts,
			  &os_err);

	  plantOutput = *((float*)p_plant_msg);

	  p_operator_input_msg = OSQPend(&OperatorInputQ,
			  0,
			  OS_OPT_PEND_NON_BLOCKING,
			  &msg_size,
			  &ts,
			  &os_err);

	  if(msg_size > 0) {
		  operatorInput = *((float *)p_operator_input_msg);
		  if(currentControlMode == AUTO) {
			  temperatureInput = operatorInput;
		  } else {
			  voltageInput = operatorInput;
		  }
	  }


	  currentPoint.X = previousPoint.X + xSpacing;
	  currentPoint.Y = startYPosition - (plantOutput * yScaleFactor) ;


	  if(currentControlMode == AUTO) {
		  strcpy(plantMode, "Plant Mode: AUTO");
		  sprintf(operatorInputStr, "Desired Temperature: %.3f C", temperatureInput);
		  referencePoint.Y = startYPosition - (TempToVoltage(temperatureInput) * yScaleFactor);
		  drawReferenceLine(referencePoint, xAxisMax);
		  BSP_LCD_DisplayStringAt(referencePoint.X, referencePoint.Y, "Vref", CENTER_MODE);
	  } else {
		  strcpy(plantMode, "Plant Mode: MANUAL");
		  sprintf(operatorInputStr, "Plant Input Voltage: %.3f V", voltageInput);
	  }

	  sprintf(plantOutputStr, "Plant Output: %.3f V", plantOutput);
	  sprintf(samplingTimeStr, "Sampling Time: 200ms");

	  showText(plantModeLine, plantMode);
	  showText(operatorInputLine, operatorInputStr);
	  showText(plantOutputLine, plantOutputStr);
	  showText(samplingTimeLine, samplingTimeStr);


	  createAxis(coordinateFrame, xAxisMax, yAxisMax);

	  drawLine(previousPoint, currentPoint);

	  previousPoint = currentPoint;
	  previousPoint.X += xSpacing;

	  OSTimeDlyHMSM(0u, 0u, 0u, 300u,
						OS_OPT_TIME_HMSM_STRICT,
						&os_err);
  }
}


/**********************************************************************************************************
*                                            Plant TASK
*
* Description : This task is the discretized transfer function of the Hot Air Plant from Lab 1 & 2, producing:
*
* 				 		  0.119217
* 				H(z) = ---------------
* 						z - 0.904837
*
* 				using settings:
*
* 				 - Sample Time = 200ms
*				 - Blower opening = 50
*
* Arguments   : p_arg   is the argument passed to 'PlantTask()' by 'OSTaskCreate()'.
* Returns     : none
* Notes       : 1) 	You must tune your PID gains in LabVIEW with the above settings (in Description),
* 					so that those gains will work when used within your PID Task
*
* 				2)  float plant_intput --> input to descretized transfer function / plant
* 				    float plant_output --> output of descretized transfer function / plant
*********************************************************************************************************
*/


static float prev_plant_output = 0;
static float plant_input = 0;

static void PlantTask(void *p_arg){

	OS_ERR  os_err;
	void        *p_msg;
	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;

	float plant_output = 0;

	while(1){

		// Get voltage from plant input queue
	    p_msg = OSQPend(&PlantInputQ,
	                     0,
	                     OS_OPT_PEND_NON_BLOCKING, // doesn't wait for queue
	                    &msg_size,
	                    &ts,
	                    &os_err);


	    if (msg_size > 0){
		    plant_input = *((float *) p_msg);
	    }

	    // calculate plant output based on discrete transfer function
		plant_output = 0.119217*plant_input + 0.904837*prev_plant_output;

		if(currentControlMode == AUTO) {
			OSQPost(&PIDQ,
					(float *) &plant_output,
					sizeof((void *)&plant_output),
				    OS_OPT_POST_FIFO,
				    &os_err);
		}

		OSQPost(&GUIQ,
						(float *) &plant_output,
						sizeof((void *)&plant_output),
						OS_OPT_POST_FIFO,
						&os_err);


		/*
		 * In order to print out floating point numbers,
		 * go to Project > Properties > C/C++ Build > Settings and
		 * change Runtime Library from 'Newlib-nano' to 'Newlib standard'
		 */

		// convert plant output to temperature before printing to teraterm
		printf("uC/OS - Plant Input : %0.3f Volts ----------- Plant Output : %0.3f Volts \\ %0.3f C\n\r", plant_input, plant_output, VoltageToTemp(plant_output) );

		prev_plant_output = plant_output;


		// 'sample time' (runs every  200 ms)
		OSTimeDlyHMSM(0u, 0u, 0u, 200u,
						OS_OPT_TIME_HMSM_STRICT,
						&os_err);
	}
}


/**********************************************************************************************************
*                                            PID TASK
*
* Description : This task is the PID controller task. It receives a setpoint value from the user and the plant output from the plant.
* The velocity PID algorithm is implemented to ensure a bumpless transfer between manual and auto mode.
*
*	e[n] = r[n] - c[n]
*	m[n] - m[n - 1] = k_p * (e[n] - e[n - 1]) + K_i * (e[n]) + K_d * (e[n - 1] + e[n - 2])
*
*	Where
*	- e[n] is the error
*	- m[n] is the manipulated variable
*	- r[n] is the set point
*
*
* Arguments   : p_arg   is the argument passed to 'PIDTask()' by 'OSTaskCreate()'.
* Returns     : none
* Notes       : 1) 	You must tune your PID gains in LabVIEW with the above settings (in Description),
* 					so that those gains will work when used within your PID Task
*********************************************************************************************************
*/
static void PIDTask(void* p_arg) {
	// PID Coefficients
	float K_p = 1.34;
	float T_i = 0.045;
	float T_d = 0.0;
	float T_s = 0.2/60;

	float pidInput = 0.0;
	float pidOutput = 0.0;

	// Desired value in the plant (Temperature)
	float setPoint = 20.0;

	float currentError= 0.0;
	float errorDelta_1 = 0.0;
	float errorDelta_2 = 0.0;
	float changeInOutput = 0.0;
	float errors[2] = {0.0, 0.0};

	void *p_pid_input_msg;
	void *p_setpoint_msg;

	OS_ERR  os_err;

	OS_MSG_SIZE  msg_size;
	CPU_TS       ts;

	while(DEF_ON) {

		p_setpoint_msg = OSQPend(&SetPointQ,
			                     0,
			                     OS_OPT_PEND_NON_BLOCKING,
			                    &msg_size,
			                    &ts,
			                    &os_err);

		if(msg_size > 0){
			setPoint = *((float *)p_setpoint_msg);
		}


		// Get voltage from plant input queue
		p_pid_input_msg = OSQPend(&PIDQ,
		           0,
		           OS_OPT_PEND_BLOCKING,
				   &msg_size,
			       &ts,
			       &os_err);


		pidInput = *((float *)p_pid_input_msg);

		currentError = TempToVoltage(setPoint) - pidInput;
		errorDelta_1 = currentError - errors[0];
		errorDelta_2 = errors[0] - errors[1];

		changeInOutput = K_p * (errorDelta_1 + (currentError * T_s/T_i) + (errorDelta_2 * T_d/T_s));

		pidOutput += changeInOutput;

		errors[1] = errors[0];
		errors[0] = currentError;

		if(currentControlMode == AUTO) {
			OSQPost(&PlantInputQ,
					(float *) &pidOutput,
					sizeof((void *)&pidOutput),
					OS_OPT_POST_FIFO,
					&os_err);
		}

		OSTimeDlyHMSM(0u, 0u, 0u, 200u,
								OS_OPT_TIME_HMSM_STRICT,
								&os_err);
	}
}




/**********************************************************************************************************
*
*            						HELPER FUNCTIONS
*
*********************************************************************************************************
*/


static void createAxis(Point frame, int xAxisSize, int yAxisSize) {
	BSP_LCD_DrawHLine(frame.X, frame.Y, xAxisSize);
	BSP_LCD_DrawVLine(frame.X, frame.Y - yAxisSize, yAxisSize);
}

static void drawLine(Point a, Point b) {
	BSP_LCD_DrawLine(a.X, a.Y, b.X, b.Y);
}

static void showText(uint16_t line, char* text) {
	BSP_LCD_ClearStringLine(line);
	BSP_LCD_DisplayStringAtLine((uint16_t) line, text);
}

static void drawReferenceLine(Point p, int distance) {
	int currentColor =  BSP_LCD_GetBackColor();

	BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	BSP_LCD_DrawHLine(p.X, p.Y, distance);

	BSP_LCD_SetBackColor(currentColor);
}


/*
 * Lab 1 & 2 temp/voltage relationship
 * using polynomial order 2
 */

static float TempToVoltage(float temp){
	return -0.0015*pow(temp,2) + 0.3319*temp - 6.9173;
}

static float VoltageToTemp(float volt){
	return 0.3053*pow(volt,2) + 2.2602*volt + 25.287;
}




/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

#if 0
  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
#endif
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */
#define USE_VCP 1
#define USE_MYMUTEX 0
#if USE_VCP
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
int __io_putchar(int ch);
int _write(int file,char *ptr, int len)
{
 int DataIdx;
#if USE_MYMUTEX
 static	OS_MUTEX           MyMutex;
 OS_ERR  os_err;
 CPU_TS  ts;
		// use mutex to protect VCP access
		OSMutexPend((OS_MUTEX *)&MyMutex,
				  (OS_TICK   )0,
				  (OS_OPT    )OS_OPT_PEND_BLOCKING,
				  (CPU_TS   *)&ts,
				  (OS_ERR   *)&os_err);
#endif
 for(DataIdx= 0; DataIdx< len; DataIdx++)
 {
 __io_putchar(*ptr++);
 }
#if USE_MYMUTEX
		OSMutexPost((OS_MUTEX *)&MyMutex,
				  (OS_OPT    )OS_OPT_POST_NONE,
				  (OS_ERR   *)&os_err);
#endif
 return len;
}
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	while(CDC_Transmit_HS((uint8_t *)&ch, 1) != USBD_OK);
	return ch;
}

#else
#ifdef __GNUC__
int _write(int32_t file, uint8_t *ptr, int32_t len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	/* return len; */
	int i;
	for(i=0; i<len; i++)
		ITM_SendChar(*ptr++);
	return len;
}
#endif
volatile int32_t ITM_RxBuffer = ITM_RXBUFFER_EMPTY;

int fputc(int ch, FILE *f) {
  return (ITM_SendChar(ch));
}

int fgetc(FILE *f) {                /* blocking */
  while (ITM_CheckChar() != 1);
  return (ITM_ReceiveChar());
}

#endif


/******END OF FILE****/
