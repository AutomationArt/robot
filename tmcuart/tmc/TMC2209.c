/*
 * TMC2209.cpp
 *
 *  Created on: Aug 26, 2023
 *      Author: admin
 */

#include "TMC2209.h"

GlobalConfig global_config_;
ChopperConfig chopper_config_;
PwmConfig pwm_config_;
CoolConfig cool_config_;
DriverCurrent driver_current_;


bool tmc_isCommunicating() {

	return (tmc_getVersion() == VERSION);

}

uint8_t tmc_getVersion() {

	Input input;
	input.bytes = read(ADDRESS_IOIN);
	return input.version;

}

uint8_t calculateCrc(ReadRequestDatagram datagram, uint8_t datagram_size) {

	uint8_t crc = 0;
	uint8_t byte;
	for (uint8_t i = 0; i < (datagram_size - 1); ++i) {
		byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
		for (uint8_t j = 0; j < BITS_PER_BYTE; ++j) {
			if ((crc >> 7) ^ (byte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = crc << 1;
			}
			byte = byte >> 1;
		}
	}
	return crc;

}

void tmc_sendDatagramBidirectional(ReadRequestDatagram datagram,
		uint8_t datagram_size) {
	uint8_t byte;

	// Wait for the transmission of outgoing serial data to complete
//  serialFlush();

// clear the serial receive buffer if necessary
//  while (serialAvailable() > 0)
//  {
//    byte = serialRead();
//  }

	while (HAL_UART_Receive(huart2, &data, 1, timeout) != HAL_OK) {
		// Do nothing
	}

	for (uint8_t i = 0; i < datagram_size; ++i) {
		byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
		HAL_UART_Transmit(&huart2, byte, sizeof(data), 0xFFFF);
	}

	while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) {
		// ...
	}

	// Wait for the transmission of outgoing serial data to complete
	//serialFlush();

	// wait for bytes sent out on TX line to be echoed on RX line
//  uint32_t echo_delay = 0;
//  while ((serialAvailable() < datagram_size) and
//    (echo_delay < ECHO_DELAY_MAX_MICROSECONDS))
//  {
//    delayMicroseconds(ECHO_DELAY_INC_MICROSECONDS);
//    echo_delay += ECHO_DELAY_INC_MICROSECONDS;
//  }

	uint32_t echo_delay = 0;

	while (HAL_UART_GetRxCount(&huart2) < datagram_size) {
		if (echo_delay < ECHO_DELAY_MAX_MICROSECONDS) {
			HAL_Delay(ECHO_DELAY_INC_MICROSECONDS);
			echo_delay += ECHO_DELAY_INC_MICROSECONDS;
		} else {
			break;
		}
	}

	if (echo_delay >= ECHO_DELAY_MAX_MICROSECONDS) {
		return;
	}

	// clear RX buffer of echo bytes
	for (uint8_t i = 0; i < datagram_size; ++i) {
		HAL_UART_Receive(&huart2, &byte, 1, 0xFFFF);
	}
}

uint32_t tmc_reverseData(uint32_t data) {
	uint32_t reversed_data = 0;
	uint8_t right_shift;
	uint8_t left_shift;
	for (uint8_t i = 0; i < DATA_SIZE; ++i) {
		right_shift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
		left_shift = i * BITS_PER_BYTE;
		reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
	}
	return reversed_data;
}

uint16_t tmc_getMicrostepsPerStep()
{
  uint16_t microsteps_per_step_exponent;
  switch (chopper_config_.mres)
  {
    case MRES_001:
    {
      microsteps_per_step_exponent = 0;
      break;
    }
    case MRES_002:
    {
      microsteps_per_step_exponent = 1;
      break;
    }
    case MRES_004:
    {
      microsteps_per_step_exponent = 2;
      break;
    }
    case MRES_008:
    {
      microsteps_per_step_exponent = 3;
      break;
    }
    case MRES_016:
    {
      microsteps_per_step_exponent = 4;
      break;
    }
    case MRES_032:
    {
      microsteps_per_step_exponent = 5;
      break;
    }
    case MRES_064:
    {
      microsteps_per_step_exponent = 6;
      break;
    }
    case MRES_128:
    {
      microsteps_per_step_exponent = 7;
      break;
    }
    case MRES_256:
    default:
    {
      microsteps_per_step_exponent = 8;
      break;
    }
  }
  return 1 << microsteps_per_step_exponent;
}

uint8_t tmc_holdDelaySettingToPercent(uint8_t hold_delay_setting)
{
//  uint8_t percent = map(hold_delay_setting,
//    HOLD_DELAY_MIN,
//    HOLD_DELAY_MAX,
//    PERCENT_MIN,
//    PERCENT_MAX);

	uint8_t percent = (hold_delay_setting - HOLD_DELAY_MIN) * (PERCENT_MAX- PERCENT_MIN) / (HOLD_DELAY_MAX - HOLD_DELAY_MIN) + PERCENT_MIN;

  return percent;
}


uint8_t tmc_currentSettingToPercent(uint8_t current_setting)
{
//  uint8_t percent = map(current_setting,
//    CURRENT_SETTING_MIN,
//    CURRENT_SETTING_MAX,
//    PERCENT_MIN,
//    PERCENT_MAX);
//long x, long in_min, long in_max, long out_min, long out_max

	uint8_t percent = (current_setting - CURRENT_SETTING_MIN) * (PERCENT_MAX- PERCENT_MIN) / (CURRENT_SETTING_MAX - CURRENT_SETTING_MIN) + PERCENT_MIN;

  return percent;
}

void tmc_readAndStoreRegisters() {

	global_config_.bytes = read(ADDRESS_GCONF);
	chopper_config_.bytes = read(ADDRESS_CHOPCONF);
	pwm_config_.bytes = read(ADDRESS_PWMCONF);

}

uint32_t read(uint8_t register_address) {

	ReadRequestDatagram read_request_datagram;
	read_request_datagram.bytes = 0;
	read_request_datagram.sync = SYNC;
	read_request_datagram.serial_address = serial_address_;
	read_request_datagram.register_address = register_address;
	read_request_datagram.rw = RW_READ;
	read_request_datagram.crc = calculateCrc(read_request_datagram,
			READ_REQUEST_DATAGRAM_SIZE);

	tmc_sendDatagramBidirectional(read_request_datagram,
			READ_REQUEST_DATAGRAM_SIZE);

	uint32_t reply_delay = 0;

	while (HAL_UART_GetRxCount(&huart1) < WRITE_READ_REPLY_DATAGRAM_SIZE) {
		if (reply_delay < REPLY_DELAY_MAX_MICROSECONDS) {
			HAL_Delay(REPLY_DELAY_INC_MICROSECONDS);
			reply_delay += REPLY_DELAY_INC_MICROSECONDS;
		} else {
			return 0;
		}
	}

	uint64_t byte;
	uint8_t byte_count = 0;
	WriteReadReplyDatagram read_reply_datagram;
	read_reply_datagram.bytes = 0;

	for (uint8_t i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; ++i) {
		HAL_UART_Receive(&huart2, &byte, 1, 0xFFFF);
		read_reply_datagram.bytes |= (byte << (byte_count++ * BITS_PER_BYTE));
	}

	return tmc_reverseData(read_reply_datagram.data);

}

SettingsTMC tmc_getSettings() {
	SettingsTMC settings;
	settings.is_communicating = tmc_isCommunicating();
	if (settings.is_communicating) {
		tmc_readAndStoreRegisters();

		settings.is_setup = global_config_.pdn_disable;
		settings.software_enabled = (chopper_config_.toff > TOFF_DISABLE);
		settings.microsteps_per_step = tmc_getMicrostepsPerStep();
		settings.inverse_motor_direction_enabled = global_config_.shaft;
		settings.stealth_chop_enabled = not
		global_config_.enable_spread_cycle;
		settings.standstill_mode = pwm_config_.freewheel;
		settings.irun_percent = tmc_currentSettingToPercent(driver_current_.irun);
		settings.irun_register_value = driver_current_.irun;
		settings.ihold_percent = tmc_currentSettingToPercent(driver_current_.ihold);
		settings.ihold_register_value = driver_current_.ihold;
		settings.iholddelay_percent = tmc_holdDelaySettingToPercent(driver_current_.iholddelay);
		settings.iholddelay_register_value = driver_current_.iholddelay;
		settings.automatic_current_scaling_enabled = pwm_config_.pwm_autoscale;
		settings.automatic_gradient_adaptation_enabled = pwm_config_.pwm_autograd;
		settings.pwm_offset = pwm_config_.pwm_offset;
		settings.pwm_gradient = pwm_config_.pwm_grad;
		settings.cool_step_enabled = cool_step_enabled_;
		settings.analog_current_scaling_enabled = global_config_.i_scale_analog;
		settings.internal_sense_resistors_enabled =	global_config_.internal_rsense;
	} else {
		settings.is_setup = false;
		settings.software_enabled = false;
		settings.microsteps_per_step = 0;
		settings.inverse_motor_direction_enabled = false;
		settings.stealth_chop_enabled = false;
		settings.standstill_mode = pwm_config_.freewheel;
		settings.irun_percent = 0;
		settings.irun_register_value = 0;
		settings.ihold_percent = 0;
		settings.ihold_register_value = 0;
		settings.iholddelay_percent = 0;
		settings.iholddelay_register_value = 0;
		settings.automatic_current_scaling_enabled = false;
		settings.automatic_gradient_adaptation_enabled = false;
		settings.pwm_offset = 0;
		settings.pwm_gradient = 0;
		settings.cool_step_enabled = false;
		settings.analog_current_scaling_enabled = false;
		settings.internal_sense_resistors_enabled = false;
	}

	return settings;
};
