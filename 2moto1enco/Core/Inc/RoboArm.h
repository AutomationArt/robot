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

#define drvMicroSteps 8
#define spoolStep 20
#define motorStep 200
#define beltRatio 2

class RoboArm
{
public:

	float linearStepsMil=motorStep*drvMicroSteps/( beltRatio*spoolStep);
	//Settings for moto/rs
	TIM_HandleTypeDef *htim1M1;
	TIM_HandleTypeDef *htim2M2;

	GPIO_TypeDef * Dir1_GPIO_Port_M1;
	uint16_t Dir1_Pin_M1;
	GPIO_TypeDef * Dir2_GPIO_Port_M2;
	uint16_t Dir2_Pin_M2;
	GPIO_TypeDef * EN1_GPIO_Port_M1;
	uint16_t EN1_Pin_M1;
	GPIO_TypeDef * EN2_GPIO_Port_M2;
	uint16_t EN2_Pin_M2;

	float distPsteps=0,anglePsteps=0;


	//ENCODERS
	uint8_t ResolutionEncoders=14;
	SPI_HandleTypeDef *arm_hspi1;
	uint16_t CS_Pin_Enc1;
	GPIO_TypeDef *CS_GPIO_Port_Enc1;
	uint16_t CS_Pin_Enc2;
	GPIO_TypeDef *CS_GPIO_Port_Enc2;

	uint32_t posNowEnc1, posNowEnc2;

	// 124 мм лыныйне перемышення   добавить к линейному перемщению при сбросе

	//Записать последние данные в память флеш

	//entity
	float posNowAngle;
	uint16_t posNowDistance;
	bool stateMoveM1=false, stateMoveM2=false;
	uint16_t defaultAngle, defaultDistanse; //стандартний кут //0 120 240 та дистанція 124 мм


	RoboArm(uint8_t, uint8_t);

	// Функция управления зацепом
	int OpenGripper();
	int CloseGripper();

	// Функция управления моторами M1 M2
	int setMove(uint32_t, uint32_t);
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
	int setPrintState(bool);
	bool getPrintState();

	// Функция екстренный стоп
	int EmergencyStop();
	int SetSettMotors(TIM_HandleTypeDef &htim1, TIM_HandleTypeDef &htim2, GPIO_TypeDef * Dir1_GPIO_Port_M1T, uint16_t Dir1_Pin_M1T, GPIO_TypeDef * Dir2_GPIO_Port_M2T, uint16_t Dir2_Pin_M2T, GPIO_TypeDef * EN1_GPIO_Port_M1T, uint16_t EN1_Pin_M1T, GPIO_TypeDef *  EN2_GPIO_Port_M2T, uint16_t EN2_Pin_M2T);
	int saveDatatoFlash();
	int readDataonFlash();

	int factoryReset();

private:

	bool PrintAllState=false;



	// Данные о текущем положении моторов
	uint16_t lastPosLinear;
	float lastPosAngle;
	bool lastPosGripper;
};

#endif /* ROBOARM_H_ */
