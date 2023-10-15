/*
 * RoboArm.cpp
 *
 *  Created on: Oct 3, 2023
 *      Author: admin
 */

#include "RoboArm.h"

RoboArm::RoboArm(uint8_t defaultAngleT, uint8_t defaultDistanseT) {
	// TODO Auto-generated constructor stub
	defaultAngle = defaultAngleT;
	defaultDistanse = defaultDistanseT;
	startDWT();
}

//RoboArm::~RoboArm() {
//	// TODO Auto-generated destructor stub
//}

int RoboArm::CloseGripper() {
	return 0;
}

int RoboArm::EmergencyStop() {

	HAL_GPIO_WritePin(EN1_GPIO_Port_M1, EN1_Pin_M1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN2_GPIO_Port_M2, EN2_Pin_M2, GPIO_PIN_SET);

	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_3);
	HAL_TIM_Base_Stop_IT(htim1M1);
	HAL_TIM_Base_Stop_IT(htim2M2);

	return 0;
}

int RoboArm::setMove(uint32_t angle, uint32_t distance) {
//m0
	// TIM1 Х  enc1 -  угол 360  -  8 оборотов движка на 1 оборот энкодера
	// TIM2  Y  enc2 - линейный -  6,4516129 оборотов движка (это целое линейное перемещение с запасом) на 1 оборот энкодера
	// 5 мм - 7.1 градусов
	// 10 мм - 13.8
	// 20 мм - 27.71
	// 40 мм - 56.47
	// 80 мм - 111.01
	// 230 мм - 321.68

	if (lastPosAngle < angle) {
		HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_RESET);
	}

	if (lastPosLinear < distance) {
		HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir1_Pin_M2, GPIO_PIN_RESET);
	}

    uint32_t actualPosAngle = abs(lastPosAngle-angle);
    uint32_t actualPosDistance = abs(lastPosLinear-distance);

	anglePsteps = (actualPosAngle * (8 * motorStep * drvMicroSteps)) / 360; //angle to steps
	distPsteps = actualPosDistance * linearStepsMil; //steps to distanse

	uint32_t distPangle = ((distPsteps / (motorStep * drvMicroSteps)) * 360
			/ 6.45) * 100;

//step1 = anglePsteps;
//step2 = distPsteps;

//числа 1, 2, 3, 4, 6, 8, 9, 12, 18, 24, 36 и 72 - Це можлива обрана максимальна швидкість для мотора з більшої кількістю кроків. Це дільник таймера
	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_3);
	HAL_TIM_Base_Stop_IT(htim1M1);
	HAL_TIM_Base_Stop_IT(htim2M2);

//  частота шим = входящая частота / период (arr)
//  125 000 (125 килогерц)  = 16 000 000 / 128
// (1/60)*1000 = частота 16 (герц);
	float periodM1 = 1200; //мікросекунд
	uint32_t psc = 24;

	if (anglePsteps > distPsteps) {

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = periodM1;
		htim1M1->Instance->CCR3 = periodM1 / 2;

		float delimiter = anglePsteps / distPsteps;
		float mnoj = ceil(periodM1 * delimiter);

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = mnoj;
		htim2M2->Instance->CCR3 = ceil(mnoj / 2);

	} else {
//	uint8_t delimiter=distPsteps/anglePsteps;
//	uint16_t impMore = (72000000/psc_max)/1000; 						//імпульсів кроків за секунду для мотора з більшої кількістю кроків  КРОКІВ НА СЕКУНДУ
//	uint16_t allSecMore = (distPsteps/impMore)*1000;		 				//загальний час роботи мотора із більшої кількістю кроків  мілісекунд
//	uint16_t stepSecM1 =  (anglePsteps/allSecMore)*1000; 					//кроків на секунду на двигуна LESS  250
//	uint16_t PSCmLess= 72000000 / (stepSecM1 * 1000); 					//дільник для мотора LESS

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = periodM1;
		htim2M2->Instance->CCR3 = periodM1 / 2;

		float delimiter = distPsteps / anglePsteps;
		float mnoj = ceil(periodM1 * delimiter);

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = mnoj;
		htim1M1->Instance->CCR3 = ceil(mnoj / 2);
	}

//Старт таймера та переривань

	HAL_TIM_Base_Start_IT(htim1M1);
	HAL_TIM_Base_Start_IT(htim2M2);

	HAL_TIM_PWM_Start(htim1M1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim2M2, TIM_CHANNEL_3);

	return 0;
}

int RoboArm::factoryReset() {
	SetZeroEncoders();
	posNowAngle = defaultAngle;
	posNowDistance = 124;
}

int RoboArm::MoveMotors() {
	return 0;
}

int RoboArm::OpenGripper() {
	return 0;
}

int RoboArm::setPrintState(bool state) {
	if (state) {
		PrintAllState = true;
	} else {
		PrintAllState = false;
	}
	return 0;
}

bool RoboArm::getPrintState() {
	if (PrintAllState) {
		return true;
	} else {
		return false;
	}
}

int RoboArm::SetSettEncoders(SPI_HandleTypeDef &arm_hspi1T,
		GPIO_TypeDef *CS_GPIO_Port_Enc1T, uint16_t CS_Pin_Enc1T,
		GPIO_TypeDef *CS_GPIO_Port_Enc2T, uint16_t CS_Pin_Enc2T,
		uint8_t ResolutionEncodersT) {

	arm_hspi1 = &arm_hspi1T;
	CS_GPIO_Port_Enc1 = CS_GPIO_Port_Enc1T;
	CS_Pin_Enc1 = CS_Pin_Enc1T;
	CS_Pin_Enc2 = CS_Pin_Enc2T;
	CS_GPIO_Port_Enc2 = CS_GPIO_Port_Enc2T;
	ResolutionEncoders = ResolutionEncodersT;
	return 0;
}

float RoboArm::GetAngleEncoders(uint32_t encoderValue) {
	//	https://www.cuidevices.com/product/resource/amt22.pdf
	return calculateAngle(encoderValue, ResolutionEncoders);
}

uint32_t RoboArm::GetPosEncoders(uint8_t numEnc) {
	switch (numEnc) {
	case 1:
		posNowEnc1 = getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc1, CS_Pin_Enc1,
				ResolutionEncoders);
		return posNowEnc1;
		break;
	case 2:
		posNowEnc2 = getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc2, CS_Pin_Enc2,
				ResolutionEncoders);
		return posNowEnc2;
		break;
	default:
		return 1;
		break;
	}
	return 0;
}

int RoboArm::SetZeroEncoders() {

	HAL_Delay(300);
	getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc1, CS_Pin_Enc1,
			ResolutionEncoders);
	getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc2, CS_Pin_Enc2,
			ResolutionEncoders);
	//	resetAMT22();
	setZeroSPI(arm_hspi1, CS_GPIO_Port_Enc1, CS_Pin_Enc1); //воно само зчитає поточну позицію и засейвить її в пам'ять
	setZeroSPI(arm_hspi1, CS_GPIO_Port_Enc2, CS_Pin_Enc2); //воно само зчитає поточну позицію и засейвить її в пам'ять
	HAL_Delay(250);
	return 0;
}

int RoboArm::SetSettMotors(TIM_HandleTypeDef &htim1, TIM_HandleTypeDef &htim2,
		GPIO_TypeDef *Dir1_GPIO_Port_M1T, uint16_t Dir1_Pin_M1T,
		GPIO_TypeDef *Dir2_GPIO_Port_M2T, uint16_t Dir2_Pin_M2T,
		GPIO_TypeDef *EN1_GPIO_Port_M1T, uint16_t EN1_Pin_M1T,
		GPIO_TypeDef *EN2_GPIO_Port_M2T, uint16_t EN2_Pin_M2T) {
	htim1M1 = &htim1;
	htim2M2 = &htim2;

	Dir1_GPIO_Port_M1 = Dir1_GPIO_Port_M1T;
	Dir1_Pin_M1 = Dir1_Pin_M1T;
	Dir2_GPIO_Port_M2 = Dir2_GPIO_Port_M2T;
	Dir2_Pin_M2 = Dir2_Pin_M2T;
	EN1_GPIO_Port_M1 = EN1_GPIO_Port_M1T;
	EN1_Pin_M1 = EN1_Pin_M1T;
	EN2_GPIO_Port_M2 = EN2_GPIO_Port_M2T;
	EN2_Pin_M2 = EN2_Pin_M2T;

	return 0;
}

int RoboArm::Turn(int angle) {
	return 0;
}
