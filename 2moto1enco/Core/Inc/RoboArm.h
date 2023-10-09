/*
 * RoboArm.h
 *
 *  Created on: Oct 3, 2023
 *      Author: admin
 */



#ifndef ROBOARM_H_
#define ROBOARM_H_

#include "stm32f1xx_hal.h"
#include <math.h>
#include "AMT22.h"

#define drvMicroSteps 16
#define spoolStep 20
#define motorStep 200
#define beltRatio 2


class RoboArm
{
public:

	uint8_t ResolutionEncoders=12;
	float linearStepsMil=(spoolStep*(motorStep*drvMicroSteps))/(beltRatio*360);
	//Settings for motors
	TIM_HandleTypeDef htim1M1;
	TIM_HandleTypeDef htim2M2;

	//ENCODERS
	SPI_HandleTypeDef *arm_hspi1;
	uint16_t CS_Pin_Enc1;
	GPIO_TypeDef *CS_GPIO_Port_Enc1;
	uint16_t CS_Pin_Enc2;
	GPIO_TypeDef *CS_GPIO_Port_Enc2;
	uint32_t posNowEnc1, posNowEnc2;


	RoboArm();

	// Функция управления зацепом
	int OpenGripper();
	int CloseGripper();

	// Функция управления моторами M1 M2
	int setMoveLinear(uint16_t);
	int setMove(uint16_t, uint16_t, bool);
	int Turn(int angle);
	int MoveMotors();

	// Функции for encoders
	int SetSettEncoders(SPI_HandleTypeDef &arm_hspi1T,
			GPIO_TypeDef *CS_GPIO_Port_Enc1T, uint16_t CS_Pin_Enc1T,
			GPIO_TypeDef *CS_GPIO_Port_Enc2T, uint16_t CS_Pin_Enc2T,
			uint8_t ResolutionEncodersT);
	uint32_t GetPosEncoders(uint8_t); //1 or 2
	int SetZeroEncoders();
	float GetAngleEncoders(uint32_t);

	//Function for Motors
	int MoveLinearMotor();
	// Функция вывода статуса
	int PrintStatus();

	// Функция екстренный стоп
	int EmergencyStop();
	int SetSettMotors(TIM_HandleTypeDef &htim1, TIM_HandleTypeDef &htim2);

	int saveDatatoFlash();
	int readDataonFlash();


private:


	//essence



	// Данные о текущем положении моторов
	uint16_t lastPosLinear;
	float lastPosAngle;
	bool lastPosGripper;
};

#endif /* ROBOARM_H_ */
