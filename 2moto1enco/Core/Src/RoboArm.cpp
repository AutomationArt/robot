/*
 * RoboArm.cpp
 *
 *  Created on: Oct 3, 2023
 *      Author: admin
 */

#include "RoboArm.h"


RoboArm::RoboArm() {
	// TODO Auto-generated constructor stub
}

//RoboArm::~RoboArm() {
//	// TODO Auto-generated destructor stub
//}

int RoboArm::CloseGripper() {
	return 0;
}

int RoboArm::EmergencyStop() {
return 0;
}

int RoboArm::setMoveLinear(uint16_t Distance) {
//m0
	return 0;
}

int RoboArm::setMove(uint16_t angle, uint16_t distance, bool setGripper) {
//m0
//	- у первого 6,4516129 оборотов движка (это целое линейное перемещение с запасом) на 1 оборот энкодера
//	- у второго 8 оборотов движка на 1 оборот энкодера

uint32_t anglePsteps =  12000; // (angle*8*motorStep*drvMicroSteps)/360; //angle to steps
uint32_t distPsteps= 14000; //distance*linearStepsMil; //steps to distanse

//step1 = anglePsteps;
//step2 = distPsteps;

//числа 1, 2, 3, 4, 6, 8, 9, 12, 18, 24, 36 и 72 - Це можлива обрана максимальна швидкість для мотора з більшої кількістю кроків. Це дільник таймера
HAL_TIM_PWM_Stop(&htim1M1, TIM_CHANNEL_3);
HAL_TIM_PWM_Stop(&htim2M2, TIM_CHANNEL_3);
HAL_TIM_Base_Stop_IT(&htim1M1);
HAL_TIM_Base_Stop_IT(&htim2M2);

//  частота шим = входящая частота / период (arr)
//  125 000 (125 килогерц)  = 16 000 000 / 128
// (1/60)*1000 = частота 16 (герц);
float periodM1 = 1200; //мікросекунд
uint32_t psc = 144;

if (anglePsteps > distPsteps) {

	htim1M1.Instance->PSC = psc;
	htim1M1.Instance->ARR = periodM1;
	htim1M1.Instance->CCR3 = periodM1 / 2;

	float delimiter = anglePsteps / distPsteps;
	float mnoj = ceil(periodM1 * delimiter);

	htim2M2.Instance->PSC = psc;
	htim2M2.Instance->ARR = mnoj;
	htim2M2.Instance->CCR3 = mnoj / 2;

} else if (anglePsteps < distPsteps) {
//	uint8_t delimiter=distPsteps/anglePsteps;
//	uint16_t impMore = (72000000/psc_max)/1000; 						//імпульсів кроків за секунду для мотора з більшої кількістю кроків  КРОКІВ НА СЕКУНДУ
//	uint16_t allSecMore = (distPsteps/impMore)*1000;		 				//загальний час роботи мотора із більшої кількістю кроків  мілісекунд
//	uint16_t stepSecM1 =  (anglePsteps/allSecMore)*1000; 					//кроків на секунду на двигуна LESS  250
//	uint16_t PSCmLess= 72000000 / (stepSecM1 * 1000); 					//дільник для мотора LESS

	htim2M2.Instance->PSC = psc;
	htim2M2.Instance->ARR = periodM1;
	htim2M2.Instance->CCR3 = periodM1 / 2;

	float delimiter = distPsteps / anglePsteps;
	float mnoj = ceil(periodM1 * delimiter);

	htim1M1.Instance->PSC = psc;
	htim1M1.Instance->ARR = mnoj;
	htim1M1.Instance->CCR3 = mnoj / 2;
}

//Старт таймера та переривань

HAL_TIM_PWM_Start(&htim1M1, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim2M2, TIM_CHANNEL_3);
HAL_TIM_Base_Start_IT(&htim1M1);
HAL_TIM_Base_Start_IT(&htim2M2);

		return 0;
}

int RoboArm::MoveMotors() {
	return 0;
}

int RoboArm::OpenGripper() {
	return 0;
}

int RoboArm::PrintStatus() {
	return 0;
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

float RoboArm::GetAngleEncoders(uint32_t encoderValue){
	//	https://www.cuidevices.com/product/resource/amt22.pdf

	float angle = 0.0;
	angle = ((float) encoderValue / ENCODER_RESOLUTION_14_BIT) * 360.0;

	return angle;
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
	DWT_Delay_ms(300); //при старте 200 милисек не хотел ебать?
	getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc1, CS_Pin_Enc1, ResolutionEncoders);
	getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc2, CS_Pin_Enc2, ResolutionEncoders);
	//	resetAMT22();
	setZeroSPI(arm_hspi1, CS_GPIO_Port_Enc1, CS_Pin_Enc1); //воно само зчитає поточну позицію и засейвить її в пам'ять
	setZeroSPI(arm_hspi1, CS_GPIO_Port_Enc2, CS_Pin_Enc2); //воно само зчитає поточну позицію и засейвить її в пам'ять
	DWT_Delay_ms(250);
	return 0;
}

int RoboArm::SetSettMotors(TIM_HandleTypeDef &htim1, TIM_HandleTypeDef &htim2) {
	htim1M1=htim1;
	htim2M2=htim2;
	return 0;
}

int RoboArm::Turn(int angle) {
	return 0;
}
