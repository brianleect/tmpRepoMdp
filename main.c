/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 *openocd -f openocd.cfg
 *
 *from connection.STM_Connection import*
 *stm = STMConnection()
 *stm.connect_STM()
 *stm.thread_send(b"0")
 *
 *sudo sync && sudo systemctl poweroff
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "math.h"
#include "PID.h"
#include "profile.h"
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* PID defines */
#define CMPERREV 0.01325f
#define CMPERREVTURN 0.0141f
#define PID_KP 1250.0f
#define PID_KI 0.0f
#define PID_KD 0.0f
#define LEFT 110
#define CENTER 147.6
// Before tuning #define RIGHT 215
#define RIGHT 203
#define PWMM 54.0f
#define PWMC 310.0f
#define TURNRATIO 0.8f
#define CIRCUM 305.0f

/* Motion profile defines */
#define AMAX 22.0f // Maximum acceleration
#define VMAX 40.0f // Maximum velocity
#define AMAXT 25.0f
#define VMAXT 50.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void *argument);
void motors(void *argument);

/* USER CODE BEGIN PFP */
void fastestCar();
void PIDmotor(float);
int motorCont(int speedL, int speedR, char dirL, char dirR, double dist);
void degTurn(int);
void forward(int);
void reverse(int);
void spotTurn(int);
void adjustDistance(int targetDist, int detectUpperBound);
void PIDturn(float degree, int turn);
void straightenWhileMoving(int);
void motorSpeed(float,float);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int16_t currentLeft, currentRight;
float fLeft, fRight;
int16_t diffl = 0, diffr = 0, avg = 0;
int32_t tick = 0;
uint8_t display[20];
int32_t totalTurns = 0;

/* Ultrasonic sensor */
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
int Is_First_Captured = 0; // boolean function
uint16_t Distance = 0;
uint16_t uDistCheck1 = 0;
uDistCheck2 = 0;
uDistFinal = 0;
uint8_t ultra[20];

/* UART */
uint8_t aRxBuffer[1];

/* IR */
uint32_t ir1Dist = 0;
uint32_t ir2Dist = 0;
uint32_t irLprev;
uint32_t irRprev;
uint32_t adc1;
uint32_t adc2;

float outLeft = 0;
float outRight = 0;
float errorLeft = 0;
float errorRight = 0;
float currDist;
float velocityLeft = 0;
float velocityRight = 0;
float distanceLeft = 0;
float distanceRight = 0;
float prePWMLeft;
float prePWMRight;
float pwmLeft = 2000;
float pwmRight = 2000;
float acc;
float vel;
float disp;
float constDisp;

/* Timer variables */
float startSec;
float currSec;

void delay(uint16_t time)
{ // provide us delay
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (__HAL_TIM_GET_COUNTER(&htim4) < time)
		;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // For HCSR04_Read();
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel1
	{
		if (Is_First_Captured == 0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;									  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured == 1) // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);							  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2 - IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034 / 2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
		}
	}
}

uint16_t HCSR04_Read(void) // Read Ultrasonic Distance
{
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);	 // pull the TRIG pin HIGH
	delay(10);													 // wait for 10 us
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET); // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
	return Distance;
}

uint16_t ultraDistCheck(void)
{
	uDistCheck1 = HCSR04_Read();
	HAL_Delay(100);
	uDistCheck2 = HCSR04_Read();
	HAL_Delay(100);
	while (abs(uDistCheck1 - uDistCheck2) >= 5)
	{
		uDistCheck1 = HCSR04_Read();
		HAL_Delay(100);
		uDistCheck2 = HCSR04_Read();
		HAL_Delay(100);
	}
	uDistFinal = (uDistCheck1 + uDistCheck2) / 2;
	return uDistFinal;
}

uint32_t irLeft(void)
{ // ADC1 (a bit more wonky)
	adc1 = 0;
	float V = 0;
	HAL_ADC_Start(&hadc1);
	adc1 = HAL_ADC_GetValue(&hadc1);
	V = (float)adc1 / 1000;

	if (V <= 0.5)
		V = 0.5; // cap at 80 cm
	else if (V >= 2.84)
		V = 2.84; // cap at 10 cm

	ir1Dist = 34.96332 * pow(V, -1.19878);
	return ir1Dist;
}

uint32_t irRight(void)
{ // ADC2
	adc2 = 0;
	float V = 0;
	HAL_ADC_Start(&hadc2);
	adc2 = HAL_ADC_GetValue(&hadc2);
	V = (float)adc2 / 1000;

	if (V <= 0.42)
		V = 0.44; // cap at 80 cm
	else if (V >= 2.9)
		V = 2.95; // cap at 10 cm

	ir2Dist = 32.6167 * pow(V, -1.0928);
	return ir2Dist;
}

void waitCmd(void)
{
	while (*aRxBuffer == 'Z')
	{
		HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);
		HAL_Delay(100);
	}
}

//
// void fastestCar(){
//	/* (1) Start straight line loop */
//// maybe can move 40cm first
// PIDmotor(40);
//
// while (ultraDistCheck() > 40)
//{
//	/* Increase RPM until ??? */
//
//	/* Break to new loop for slowing down */
//	//		if(ultraDistCheck()<=40){
//	//			break;
//	//		}
// }
//
///* (2) Slow down car to prepare for turning */
// while (1)
//{
//	// Slow down code
//
//	if (ultraDistCheck() <= 20)
//	{ // turn right
//		//			htim1.Instance->CCR4 = RIGHT;
//		//			HAL_Delay(1000);
//		//			htim1.Instance->CCR4 = LEFT;
//		fastCarTurn(1);
//		break;
//	}
// }
//
///* (3) Move until near end of obstacle's corner */
// int x = 0;
// while (1)
//{
//	/* Constant Speed & calculate dist moved */
//
//	irLeft();
//	if (irLeft() >= 30)
//	{ // turn 180?
//		//			htim1.Instance->CCR4 = LEFT;
//		//			HAL_Delay(1000);
//		//			htim1.Instance->CCR4 = CENTER;
//		//			HAL_Delay(1000);
//		//			htim1.Instance->CCR4 = LEFT;
//		//			HAL_Delay(1000);
//		//			htim1.Instance->CCR4 = RIGHT;
//		fastCarTurn(2);
//		break;
//	}
// }
//// see if nd correct to be straight
//
///* (4) Move 50cm */
// PIDmotor(50);
//
///* (5) Move straight until centre of obstacle */
//// Calculate Dist to move
// int y = 60 - x - 10; // extra to make space for turning
//// Move Dist
// PIDmotor(y);
//
//// Turn Right
// fastCarTurn(1);
//
///* (6) Move back to parking spot */
// while (1)
//{
//	// Move straight
//
//	if (ultraDistCheck() <= 20)
//	{
//		// Stop
//		break;
//	}
// }
// }

void fastCarTurn(int mode)
{
	switch (mode)
	{
	case 1: // 90 degree
		htim1.Instance->CCR4 = LEFT;
		HAL_Delay(500);
		htim1.Instance->CCR4 = RIGHT;
		HAL_Delay(500);
		PIDturn(90, 2);
		PIDmotor(6);
		break;
	case 2: // 180 degree
		htim1.Instance->CCR4 = LEFT;
		HAL_Delay(500);
		htim1.Instance->CCR4 = RIGHT;
		HAL_Delay(500);
		//		PIDturn(180,2);
		break;
	}
}

void corrMotor()
{
	int x = ultraDistCheck();
	int y = abs(22 - x);
	if (x > 22 && x < 40)
	{
		PIDmotor(y);
	}
	else if (x < 22)
	{
		PIDmotor(-y);
	}
}

void adjustDistance(int targetDist, int detectUpperBound)
{
	int x = ultraDistCheck();
	int y = abs(targetDist - x);
	if (x > targetDist && x < detectUpperBound)
	{
		PIDmotor(y);
	}
	else if (x < targetDist)
	{
		PIDmotor(-y);
	}
}

void correctAngle(int mode)
{
	switch (mode)
	{
	case 1: // leftIR detect
		htim1.Instance->CCR4 = RIGHT;
		HAL_Delay(150);
		motorCont(1000, 1000, 'R', 'R', 1);
		htim1.Instance->CCR4 = LEFT;
		HAL_Delay(150);
		motorCont(1000, 1000, 'F', 'F', 1);
		htim1.Instance->CCR4 = CENTER;
		break;
	case 2: // rightIR detect
		htim1.Instance->CCR4 = LEFT;
		HAL_Delay(150);
		motorCont(1000, 1000, 'R', 'R', 1);
		htim1.Instance->CCR4 = RIGHT;
		HAL_Delay(150);
		motorCont(1000, 1000, 'F', 'F', 1);
		htim1.Instance->CCR4 = CENTER;
		break;
	}
	HAL_Delay(50);

	if (ultraDistCheck() < 20)
	{
		PIDmotor(-3);
	}
	if (ultraDistCheck() > 20 && uDistFinal < 40)
	{
		PIDmotor(3);
	}
}

void straightenWhileMoving(int direction) {
	*aRxBuffer = 'Z';
	htim1.Instance->CCR4 = CENTER+3;
	motorSpeed(2000, 2000);
	irLprev = irLeft(); irRprev = irRight(); osDelay(500);
	int targetDist = 15;

	if (direction == 1) // IR left
	{
		while (ir1Dist<50)
		{
//			if (htim1.Instance->CCR4<RIGHT && ir1Dist<targetDist) htim1.Instance->CCR4 += 2;
//			else if (htim1.Instance->CCR4>LEFT &&ir1Dist>targetDist) htim1.Instance->CCR4 -= 2;
			if (htim1.Instance->CCR4>LEFT && irLprev<ir1Dist) htim1.Instance->CCR4 -= 5; // Adjust to left more due to speed
			else if (htim1.Instance->CCR4<RIGHT && irLprev>ir1Dist) htim1.Instance->CCR4 += 5;
//
			irLprev = ir1Dist;
			//HAL_Delay(10);
			irLeft();
		}
	}
	else // IR right
	{
		while (ir2Dist<50)
		{
//			if (htim1.Instance->CCR4>LEFT && ir2Dist<targetDist) htim1.Instance->CCR4 -= 2;
//			else if (htim1.Instance->CCR4<RIGHT && ir2Dist>targetDist) htim1.Instance->CCR4 += 2;
			if (htim1.Instance->CCR4<RIGHT && irRprev<ir2Dist) htim1.Instance->CCR4 += 5; // Adjust to right more due to speed
			else if (htim1.Instance->CCR4>LEFT && irRprev>ir2Dist) htim1.Instance->CCR4 -= 5;

			irRprev = ir2Dist;
			//HAL_Delay(10);
			irRight();
		}
	}
	/* Set UARTBuffer to default value */
	*aRxBuffer = 'Z';

	motorSpeed(0, 0); // Stop motor once out
}

void correction()
{
	*aRxBuffer = 'Z';
	int mode;
	irLeft();
	irRight();
	if (irLeft() < 35 || irRight() < 35 || ultraDistCheck() > 40)
	{

		//--move forward until ir sense || cap 3 movements--
		for (int i = 0; i < 3; i++)
		{
			irLeft();
			irRight();
			if (irLeft() < 35 || irRight() < 35 || ultraDistCheck() < 20)
			{
				break;
			}
			PIDmotor(5);
		}

		//--set mode--
		uint32_t L = irLeft(); // to make below coherent
		uint32_t R = irRight();
		if (L < 35 && R < 35)
		{ // both detected
		  // can it judge base on positioning of obstacles on algo?
		}
		else if (L < 35 && L < R)
		{
			mode = 1;
		}
		else if (R < 35 && R < L)
		{
			mode = 2;
		}

		//--start angle correction until no detect || cap 3 movements--
		int cnt = 5;
		if (mode == 1)
		{
			while (cnt > 0 && irLeft() < 35)
			{
				correctAngle(1);
				cnt--;
				irLeft();
			}
		}
		else if (mode == 2)
		{
			while (cnt > 0 && irRight() < 35)
			{
				correctAngle(2);
				cnt--;
				irRight();
			}
		}
	}
	adjustDistance(10, 40);
}

void motorSpeed(float left, float right)
{
	if (left < 0)
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		left = fabs(left);
	}
	else if (left > 0)
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	}
	if (right < 0)
	{
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		right = fabs(right);
	}
	else if (right > 0)
	{
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, left);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, right);
}

/* ========================= PID Functions ========================= */

/* Read current encoder values */
void readEncoder()
{ // Forward = Positive, Backwards = negative
	currentLeft = __HAL_TIM_GET_COUNTER(&htim2);
	fLeft = currentLeft * -1 * CMPERREV;

	currentRight = __HAL_TIM_GET_COUNTER(&htim3);
	fRight = currentRight * CMPERREV;
}

/* Time value functions, in seconds with decimals */
float getTime()
{
	return __HAL_TIM_GET_COUNTER(&htim5) / 1e6f;
}

void timeStart()
{
	startSec = getTime();
}

float timeNow()
{
	currSec = getTime() - startSec;
	return currSec;
}

float forwardFeed(float vel)
{
	return vel * PWMM + PWMC;
}

/* Master function for motor with PID control */
void PIDmotor(float setDist)
{

	/* Turn servomotor to extreme end before centering for higher accuracy */
	htim1.Instance->CCR4 = LEFT;
	HAL_Delay(300);
	htim1.Instance->CCR4 = CENTER + 3;
	HAL_Delay(300);

	/* Set UARTBuffer to default value */
	*aRxBuffer = 'Z';

	/* Set distance of motors */
	float setDistLeft = setDist;
	float setDistRight = setDist;

	/* Initialize PID Controllers */
	PIDController pidLeft = {PID_KP, PID_KI, PID_KD, 0, -3000, 3000, -150, 150, prePWMLeft, 1.0f};
	PIDController pidRight = {PID_KP, PID_KI, PID_KD, 0, -3000, 3000, -150, 150, prePWMRight, 1.0f};
	Profile profileLeft;
	Profile profileRight;

	/* Initialize and set Encoder to 0 */
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // left encoder(MotorA) start
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // right encoder(MotorB) start
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);

	/* Initialize PID controllers and profiles */
	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	Profile_Init(&profileLeft);
	Profile_Init(&profileRight);
	float time = trapezoidal(&profileLeft, setDistLeft, AMAX, VMAX);
	trapezoidal(&profileRight, setDistRight, AMAX, VMAX);
	constDisp = profileRight.constDisp;

	timeStart();
	while (timeNow() < time)
	{

		/* Take current encoder values */
		readEncoder();
		currProfile(&profileLeft, currSec);
		currProfile(&profileRight, currSec);

		/* Calculate feed forward of motors */
		prePWMLeft = forwardFeed(profileLeft.currVel);
		prePWMRight = forwardFeed(profileRight.currVel);

		/* For debug purposes */
		distanceLeft = profileLeft.disp;
		distanceRight = profileRight.disp;
		acc = profileLeft.currAcc;
		vel = profileRight.currVel;
		disp = profileLeft.currDisp;

		/* Compute new control signal */
		outLeft = PIDController_Update(&pidLeft, profileLeft.currDisp, fLeft, prePWMLeft);
		outRight = PIDController_Update(&pidRight, profileRight.currDisp, fRight, prePWMRight);

		/* Update new values to motors */
		motorSpeed(outLeft, outRight);
		errorLeft = pidLeft.prevError;
		errorRight = pidRight.prevError;

		//(display, "Left:%2.2f\0", errorLeft);
		OLED_ShowString(10, 35, display);
		//sprintf(display, "Right:%2.2f\0", errorRight);
		OLED_ShowString(10, 50, display);
		OLED_Refresh_Gram();
	}

	/* Stop Motor */
	motorSpeed(0, 0);

	/* Reset Encoders value */
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
}

/* Calculate turn ratio */
void motors(void *argument)
{
	/* USER CODE BEGIN motors */
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	*aRxBuffer = 'W';
	for (;;)
	{
		switch (*aRxBuffer)
		{
		// Initialization
		case '\0':
			HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);
			break;
			//========================Forward Movement Cases========================
		case '0':
			PIDmotor(10);
			break;
		case '1':
			PIDmotor(20);
			break;
		case '2':
			PIDmotor(30);
			break;
		case '3':
			PIDmotor(40);
			break;
		case '4':
			PIDmotor(50);
			break;
		case '5':
			PIDmotor(60);
			break;
		case '6':
			PIDmotor(70);
			break;
		case '7':
			PIDmotor(80);
			break;
		case '8':
			PIDmotor(90);
			break;
		case '9':
			PIDmotor(100);
			break;
		case '9' + 1:
			PIDmotor(110);
			break;
		case '9' + 2:
			PIDmotor(120);
			break;
		case '9' + 3:
			PIDmotor(130);
			break;
		case '9' + 4:
			PIDmotor(140);
			break;
		case '9' + 5:
			PIDmotor(150);
			break;
		case '9' + 6:
			PIDmotor(160);
			break;
		case '9' + 7:
			PIDmotor(170);
			break;
		case '9' + 8:
			PIDmotor(180);
			break;
		case '9' + 9:
			PIDmotor(190);
			break;
		case '9' + 10:
			PIDmotor(200);
			break;
		case '9' + 11:
			PIDmotor(5);
			break;
		//========================Reverse Movement Cases========================
		case 'a':
			PIDmotor(-10);
			break;
		case 'b':
			PIDmotor(-20);
			break;
		case 'c':
			PIDmotor(-30);
			break;
		case 'd':
			PIDmotor(-40);
			break;
		case 'e':
			PIDmotor(-50);
			break;
		case 'f':
			PIDmotor(-60);
			break;
		case 'g':
			PIDmotor(-70);
			break;
		case 'h':
			PIDmotor(-80);
			break;
		case 'i':
			PIDmotor(-90);
			break;
		case 'j':
			PIDmotor(-100);
			break;
		case 'j' + 1: // k
			PIDmotor(-110);
			break;
		case 'j' + 2: // l
			PIDmotor(-120);
			break;
		case 'j' + 3: // m
			PIDmotor(-130);
			break;
		case 'j' + 4: // n
			PIDmotor(-140);
			break;
		case 'j' + 5: // o
			PIDmotor(-150);
			break;
		case 'j' + 6: // p
			PIDmotor(-160);
			break;
		case 'j' + 7: // q
			PIDmotor(-170);
			break;
		case 'j' + 8: // r
			PIDmotor(-180);
			break;
		case 'j' + 9: // s
			PIDmotor(-190);
			break;
		case 'j' + 10: // t
			PIDmotor(-200);
			break;
		case 'j' + 11: // u
			PIDmotor(-5);
			break;
			//========================Turn========================
			//			case 'L':
			//				htim1.Instance->CCR4 = CENTER;
			//				HAL_Delay(500);
			//				htim1.Instance->CCR4 = LEFT;
			//				HAL_Delay(500);
			//				PIDturn(30,1);
			//				htim1.Instance->CCR4 = RIGHT;
			//				HAL_Delay(500);
			//				PIDturn(-28,2);
			//				htim1.Instance->CCR4 = LEFT;
			//				HAL_Delay(500);
			//				PIDturn(30,1);
			//				PIDmotor(4.5); //Forward to fit into 10x10 grid
			//				break;
			//
			//			case 'R':
			//				htim1.Instance->CCR4 = CENTER;
			//				HAL_Delay(500);
			//				htim1.Instance->CCR4 = RIGHT;
			//				HAL_Delay(500);
			//				PIDturn(30,2);
			//				htim1.Instance->CCR4 = LEFT;
			//				HAL_Delay(500);
			//				PIDturn(-28,1);
			//				htim1.Instance->CCR4 = RIGHT;
			//				HAL_Delay(500);
			//				PIDturn(30.2,2);
			//				PIDmotor(4.5); //Forward to fit into 10x10 grid
			//				break;

			//========================Left Movements========================
			//		// 90deg Left
			//		case 'l':
			//			htim1.Instance->CCR4 = RIGHT;
			//			HAL_Delay(500);
			//			htim1.Instance->CCR4 = LEFT;
			//			HAL_Delay(500);
			//			PIDturn(90, 1);
			//			PIDmotor(-15);
			//			// PIDmotor(8);
			//			break;
			// Sharper Left (Changing direction from front to back)
		case 'L':
			PIDturn(90, 1);
			break;
			//========================Right Movements========================
			// 90deg Right
			//		case 'r':
			//			htim1.Instance->CCR4 = LEFT;
			//			HAL_Delay(500);
			//			htim1.Instance->CCR4 = RIGHT;
			//			HAL_Delay(500);
			//			PIDturn(90, 2);
			//			PIDmotor(-14);
			//			break;
			//		// Sharper Right
		case 'R':
			PIDturn(90, 2);
			//motorCont(600, 2100, 'F', 'F', turn1);
			break;
		// 	360deg (checklist)
		case 'K':
			htim1.Instance->CCR4 = RIGHT;
			HAL_Delay(500);
			htim1.Instance->CCR4 = LEFT;
			HAL_Delay(500);
			PIDturn(360, 1);
			PIDmotor(8);
			break;
		// 	Bulleyes, reverse left 45deg -> 180deg turn (checklist)
		case 'M':
			adjustDistance(11, 150);
			htim1.Instance->CCR4 = RIGHT;
			HAL_Delay(500);
			htim1.Instance->CCR4 = LEFT;
			// HAL_Delay(500);
			PIDturn(-90, 1);
			// HAL_Delay(500);
			PIDmotor(70);
			// HAL_Delay(500);
			htim1.Instance->CCR4 = RIGHT;
			HAL_Delay(500);
			htim1.Instance->CCR4 = LEFT;
			HAL_Delay(500);
			PIDturn(90, 1);
			PIDturn(90, 1);
			adjustDistance(11, 150);
			break;
			//========================Fastest Car Cases========================
		case 'N':
			adjustDistance(40, 500);
			break;
		case 'O':
			correction();
			break;
		case 'P': // 1: L
			PIDturn(90,1);
			//spotTurn(1);
			PIDmotor(-30); // Reverse to ensure within range
			spotTurn(2);
			//PIDturn(90,2);   // Travel somewhat straight
			adjustDistance(50, 500);
			// Calculate distance needed to travel till
			break;
		case 'Q': // 1: R
			PIDturn(90, 2);
			//spotTurn(2);
			PIDmotor(-30); // Reverse to ensure within range
			spotTurn(1);
			//PIDturn(90,1);
			adjustDistance(50, 500);
			// Travel somewhat straight
			break;
		case 'T': // Based on 1:R, 2:R
			spotTurn(2);
			//PIDturn(90,2);
			PIDturn(90,1);
			PIDturn(90,1);
			PIDmotor(35);
			spotTurn(1);
			//PIDturn(90,1);
			PIDmotor(25);
			PIDturn(90,1);
			spotTurn(2);
			//PIDturn(90,2);
			adjustDistance(10, 500);
			break;
		case 'S': // Based on 1:L, 2:R
			PIDturn(90,2);
			PIDmotor(20);
			PIDturn(90,1);
			PIDmotor(2);
			PIDturn(90,1);
			PIDmotor(40); // Consider adding for turn buffer dist
			PIDturn(90,1);
			PIDmotor(30); // Add space for movement
			PIDturn(90,1);
			PIDturn(90,2);
			adjustDistance(10, 500);
			break;
		case 'U': // Based on 1:R, 2:L
			PIDturn(90,1);
			PIDmotor(20);
			PIDturn(90,2);
			PIDturn(90,2);
			PIDmotor(40);
			PIDturn(90,2);
			PIDmotor(30);
			PIDturn(90,2);
			PIDturn(90,1);
			adjustDistance(10, 500);
			break;
		case 'V': // 1:L , 2:L
			spotTurn(1);
			spotTurn(2);
			//PIDturn(90,1);
			//PIDturn(90,2);
			PIDmotor(25); // Move straight to prevent collision crossing 60x10
			//PIDturn(90,2);
			spotTurn(2);
			straightenWhileMoving(2);
			//PIDmotor(100); // Move across the top of 60x10, original 35, add 70 for buffer
			PIDturn(120, 2);
			//spotTurn(2);
			//PIDturn(90,2);
			//PIDmotor(80); // Move to ensure further up from 60x10
			//PIDturn(90,2);
			//PIDturn(90,1); // Turn to face car park
			adjustDistance(10, 500);
			break;

		/* Test Cases */
		// case 'V':
		//	correction();
		//	break;
		case 'W':
			straightenWhileMoving(2);
			break;
		case 'X':
			ultraDistCheck();
			irLeft();
			irRight();
			break;
		case 'Y':
			HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // left encoder(MotorA) start
			HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // right encoder(MotorB) start
			while (1)
			{
				readEncoder();
			}
			break;
		case 'Z':
			waitCmd();
			break;
		default:
			*aRxBuffer = 'Z';
			break;
		}
		HAL_Delay(100);
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)"K", 1);
	}
	osDelay(1000);
	/* USER CODE END motors */
}

void spotTurn(int mode)
{
	totalTurns++;
	float adjustDist = 0;
	if (totalTurns % 3 == 0)
		adjustDist += 0;

	// For indoor HPL
	float turn1 = 35.7;
	float turn2 = 35;

	// For outdoor outside SWEL2
	float d1 = 15;
	float d2 = 15;

	float d1out = 34.7; // 35.7
	float d2out = 33.3;

	switch (mode)
	{
	case 1:
		// Turn left (indoor) 3,1
		htim1.Instance->CCR4 = LEFT; // Previously -3
		HAL_Delay(500);
		motorCont(600, 4000, 'F', 'F', turn1);
		HAL_Delay(500);
		htim1.Instance->CCR4 = CENTER;

		// Turn left (outdoor)
		//		htim1.Instance->CCR4 = LEFT-8; //-3
		//		HAL_Delay(300);
		//		motorCont(500, 3300, 'F', 'F', d1out);
		//		HAL_Delay(300);
		//		htim1.Instance->CCR4 = CENTER;

		// Turn left (outdoor)
		//		htim1.Instance->CCR4 = LEFT;
		//		HAL_Delay(100);
		//		motorCont(600, 2100, 'F', 'F', d1);
		//		htim1.Instance->CCR4 = 179; // right
		//		HAL_Delay(100);
		//		motorCont(2100, 600, 'R', 'R', d2);
		//		htim1.Instance->CCR4 = LEFT;
		//		HAL_Delay(100);
		//		motorCont(600, 2100, 'F', 'F', d1);
		//		htim1.Instance->CCR4 = 164; // 2nd right
		//		HAL_Delay(100);
		//		motorCont(2100, 600, 'R', 'R', (d2 + 4 + adjustDist));
		//		htim1.Instance->CCR4 = CENTER;
		//		HAL_Delay(100);
		//		motorCont(1200, 1200, 'F', 'F', (14 + (adjustDist/2)));
		break;
	case 2:
		// Turn right (indoor)
		htim1.Instance->CCR4 = RIGHT - 2; // 3,1 (image recognition)
		htim1.Instance->CCR4 = RIGHT;
		HAL_Delay(300);
		motorCont(4200, 600, 'F', 'F', turn2);
		HAL_Delay(300);
		htim1.Instance->CCR4 = CENTER;

		// Turn right (outdoor)
		//		htim1.Instance->CCR4 = RIGHT+5;
		//		HAL_Delay(300);
		//		motorCont(4000, 600, 'F', 'F', d2out);
		//		HAL_Delay(300);
		//		htim1.Instance->CCR4 = CENTER;

		//		// Turn right (outdoor)
		//		htim1.Instance->CCR4 = CENTER;
		//		HAL_Delay(100);
		//		motorCont(1200, 1200, 'F', 'F', 5);
		//		htim1.Instance->CCR4 = RIGHT;
		//		HAL_Delay(100);
		//		motorCont(2100, 600, 'F', 'F', d1);
		//		htim1.Instance->CCR4 = 128; // left
		//		HAL_Delay(100);
		//		motorCont(600, 2100, 'R', 'R', d2);
		//		htim1.Instance->CCR4 = RIGHT;
		//		HAL_Delay(100);
		//		motorCont(2100, 600, 'F', 'F', d1);
		//		htim1.Instance->CCR4 = 138; // 2nd left
		//		HAL_Delay(100);
		//		motorCont(600, 2100, 'R', 'R', (d2 + 8 + adjustDist));
		//		htim1.Instance->CCR4 = CENTER;
		//		HAL_Delay(100);
		//		motorCont(1200, 1200, 'F', 'F', (11 + 3 + (adjustDist/2)));
		break;
	}
}

float calculateTurn(float dist)
{
	// return ((dist / (2 * 3.14)) - 16) / (dist / (2 *3.14));
	return 0.793;
}

/* Read encoder values for turn */
void readEncoderTurn()
{ // Forward = Positive, Backwards = negative
	currentLeft = __HAL_TIM_GET_COUNTER(&htim2);
	fLeft = currentLeft * -1 * CMPERREVTURN;

	currentRight = __HAL_TIM_GET_COUNTER(&htim3);
	fRight = currentRight * CMPERREVTURN;
}

void PIDturn(float degree, int turn)
{

	/* Set UARTBuffer to default value */
	*aRxBuffer = 'Z';

	/* Calibrate motors */
	float setDist = CIRCUM * (degree / 360);
	float turnRatio = calculateTurn(CIRCUM);

	/* Initialize PID Controllers */
	PIDController pidLeft = {PID_KP, PID_KI, PID_KD, 0, -3000, 3000, -150, 150, prePWMLeft, 1.0f};
	PIDController pidRight = {PID_KP, PID_KI, PID_KD, 0, -3000, 3000, -150, 150, prePWMRight, 1.0f};
	Profile profileLeft;
	Profile profileRight;

	/* Initialize and set Encoder to 0 */
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // left encoder(MotorA) start
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // right encoder(MotorB) start
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);

	/* Initialize PID controllers and profiles */
	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	/* if right turn does not achieve 90 degree, set a higher set distance */
	if (turn == 2) {
		setDist = setDist * 1;
		htim1.Instance->CCR4 = LEFT;
		HAL_Delay(300);
		htim1.Instance->CCR4 = RIGHT;
		HAL_Delay(300);
	}
	else {
		setDist = setDist * 1;
		htim1.Instance->CCR4 = RIGHT;
		HAL_Delay(300);
		htim1.Instance->CCR4 = LEFT;
		HAL_Delay(300);
	}

	/* Initialize motion profile and get total time for movement */
	Profile_Init(&profileLeft);
	float time = trapezoidal(&profileLeft, setDist, AMAXT, VMAXT);

	timeStart();
	while (timeNow() < time)
	{

		/* Take current encoder values */
		readEncoderTurn();
		currProfile(&profileLeft, currSec);

		/* Compute new control signal */
		if (turn == 1)
		{ // Left Turn

			/* Constantly calculate the correct feed forward */
			prePWMLeft = forwardFeed(profileLeft.currVel * turnRatio);
			prePWMRight = forwardFeed(profileLeft.currVel);

			/* Reduce inner wheel distance and speed for turning */
			distanceLeft = profileLeft.currDisp * turnRatio;
			distanceRight = profileLeft.currDisp;

			outLeft = PIDController_Update(&pidLeft, distanceLeft, fLeft, prePWMLeft);
			outRight = PIDController_Update(&pidRight, distanceRight, fRight, prePWMRight);

			/* Update new values to motors */
			motorSpeed(outLeft, outRight);
			errorLeft = pidLeft.prevError;
			errorRight = pidRight.prevError;
		}
		else
		{
			prePWMLeft = forwardFeed(profileLeft.currVel);
			prePWMRight = forwardFeed(profileLeft.currVel * turnRatio);
			distanceLeft = profileLeft.currDisp;
			distanceRight = profileLeft.currDisp * turnRatio;

			outLeft = PIDController_Update(&pidLeft, distanceLeft, fLeft, prePWMLeft);
			outRight = PIDController_Update(&pidRight, distanceRight, fRight, prePWMRight);

			/* Update new values to motors */
			motorSpeed(outLeft, outRight);
			errorLeft = pidLeft.prevError;
			errorRight = pidRight.prevError;
		}
	}

	/* Stop Motor */
	motorSpeed(0, 0);

	/* Reset Encoders value */
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
}

// Master function for image recognition motor control
int motorCont(int speedL, int speedR, char dirL, char dirR, double dist)
{
	*aRxBuffer = 'Z';
	// declaration
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // left encoder(MotorA) start
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // right encoder(MotorB) start
	currentLeft = __HAL_TIM_GET_COUNTER(&htim2);
	currentRight = __HAL_TIM_GET_COUNTER(&htim3);
	tick = HAL_GetTick();
	double encDist = dist * 72;

	// Select direction of motor//
	switch (dirL)
	{
	case 'F':
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		break;

	case 'R':
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		break;
	}

	switch (dirR)
	{
	case 'F':
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		break;

	case 'R':
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		break;
	}
	// End of motor direction selection//

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speedL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, speedR);

	while (1)
	{
		currentLeft = __HAL_TIM_GET_COUNTER(&htim2);
		currentRight = __HAL_TIM_GET_COUNTER(&htim3);
		diffl = abs(currentLeft);
		diffr = abs(currentRight);
		avg = abs((diffl + diffr) / 2);
		//		sprintf(display, "Left:%5d\0", diffl / 68);
		//		OLED_ShowString(10, 35, display);
		//		sprintf(display, "Right:%5d\0", diffr / 68);
		//		OLED_ShowString(10, 50, display);
		//		OLED_Refresh_Gram();

		if (avg >= encDist)
		{
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
			HAL_Delay(500);
			break;
		}
	}
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim3, 0);

	speedL = speedR = tick = diffl = diffr = 0;
	OLED_Refresh_Gram();
	*aRxBuffer = 'Z';
}

void degTurn(int mode)
{
	switch (mode)
	{
	case 1: // Turn left
		htim1.Instance->CCR4 = LEFT;
		HAL_Delay(100);
		motorCont(500, 2200, 'F', 'F', 105 * 0.57);
		htim1.Instance->CCR4 = CENTER;
		break;
	case 2:
		htim1.Instance->CCR4 = RIGHT;
		HAL_Delay(100);
		motorCont(2200, 500, 'F', 'F', 100 * 0.57);
		HAL_Delay(50);
		htim1.Instance->CCR4 = CENTER;
		break;
	}
}

void forward(int mode)
{ // Forward for image recognition
	HAL_Delay(100);
	switch (mode)
	{
	case 0:
		motorCont(pwmLeft, pwmRight, 'F', 'F', 100);
		break;
	case 1:
		motorCont(pwmLeft, pwmRight, 'F', 'F', 8);
		break;
	case 2:
		motorCont(pwmLeft, pwmRight, 'F', 'F', 18);
		break;
	case 3:
		motorCont(pwmLeft, pwmRight, 'F', 'F', 28);
		break;
	case 4:
		motorCont(pwmLeft, pwmRight, 'F', 'F', 38);
		break;
	case 5:
		motorCont(pwmLeft, pwmRight, 'F', 'F', 49);
		break;
	case 6:
		motorCont(pwmLeft, pwmRight, 'F', 'F', 60);
		break;
	case 7:
		motorCont(pwmLeft, pwmRight, 'F', 'F', 70);
		break;
	case 8:
		motorCont(pwmLeft, pwmRight, 'F', 'F', 80);
		break;
	case 9:
		motorCont(pwmLeft, pwmRight, 'F', 'F', 90);
		break;
	}
}

void reverse(int mode)
{ // Reverse for image recognition
	HAL_Delay(100);
	switch (mode)
	{
	case 0:
		motorCont(pwmLeft, pwmRight, 'R', 'R', 100);
		break;
	case 1:
		motorCont(pwmLeft, pwmRight, 'R', 'R', 6);
		break;
	case 2:
		motorCont(pwmLeft, pwmRight, 'R', 'R', 18);
		break;
	case 3:
		motorCont(pwmLeft, pwmRight, 'R', 'R', 28);
		break;
	case 4:
		motorCont(pwmLeft, pwmRight, 'R', 'R', 38);
		break;
	case 5:
		motorCont(pwmLeft, pwmRight, 'R', 'R', 49);
		break;
	case 6:
		motorCont(pwmLeft, pwmRight, 'R', 'R', 60);
		break;
	case 7:
		motorCont(pwmLeft, pwmRight, 'R', 'R', 70);
		break;
	case 8:
		motorCont(pwmLeft, pwmRight, 'R', 'R', 80);
		break;
	case 9:
		motorCont(pwmLeft, pwmRight, 'R', 'R', 90);
		break;
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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);
	OLED_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motors, NULL, &MotorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_11;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_12;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable Calibrartion
  */
  if (HAL_RTCEx_SetCalibrationOutPut(&hrtc, RTC_CALIBOUTPUT_1HZ) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/*Prevent unused argument(s) compilation warning*/
	UNUSED(huart);
	HAL_UART_Transmit(&huart3, (uint8_t *)aRxBuffer, 10, 0xFFFF); // might not nd
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t test[20] = "Testing";
	uint8_t ultra[20];
	uint8_t checkPi[1];

	/* Infinite loop */
	for (;;)
	{
		HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);
		OLED_ShowString(5, 5, test);
		//		sprintf(checkPi, "Pi cmd: %s\0", aRxBuffer);
		//		OLED_ShowString(10, 20, checkPi);
		ultraDistCheck();
		// HAL_Delay(200);
		//sprintf(ultra, "uDistF: %u\0", uDistFinal);
		OLED_ShowString(10, 50, ultra);

		irLeft();
		// HAL_Delay(100);
		//sprintf(ultra, "IR left: %u\0", ir1Dist);
		OLED_ShowString(10, 30, ultra);

		irRight();
		// HAL_Delay(100);
		//sprintf(ultra, "IR right: %u\0", ir2Dist);
		OLED_ShowString(10, 40, ultra);

		OLED_Refresh_Gram();
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		// osDelay(100);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motors */
/**
 * @brief Function implementing the MotorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_motors */
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
