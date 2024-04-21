/*
 * ir_remote.h
 *
 *  Created on: Apr 8, 2024
 *      Author: User
 */

#ifndef INC_IR_REMOTE_H_
#define INC_IR_REMOTE_H_

#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define IR_REMOTE_DATA_OFFSET	(int)10 //10x32bit

typedef struct
{
	uint32_t signalBlock[1024];
	uint32_t signalOffset[5];
	uint32_t signalLength[5];
}IR_REMOTE_HandleTypeDef;

typedef struct
{
	uint8_t currentSignalNum;
	uint8_t mode;
}IR_REMOTE_MGMT_HandleTypeDef;

typedef enum
{
	IR_REMOTE_MODE_IDLE		 = 0x00,
	IR_REMOTE_MODE_LISTENING = 0x01,
	IR_REMOTE_MODE_EMITTING	 = 0x02
}IR_REMOTE_MGMT_Mode;

typedef enum
{
	IR_REMOTE_BAD_STATUS 	 = -1,
	IR_REMOTE_GOOD_STATUS	 = 1
}IR_REMOTE_STATUS;

//IR_REMOTE_HandleTypeDef func's
int  IR_REMOTE_GetSignal(IR_REMOTE_HandleTypeDef* handle, uint32_t* data, uint32_t signalNum);
int	 IR_REMOTE_GetSignalPtr(IR_REMOTE_HandleTypeDef* handle, uint32_t** dataPtr, uint32_t signalNum);
int  IR_REMOTE_PutSignal(IR_REMOTE_HandleTypeDef* handle, uint32_t* data, uint32_t signalNum);
void IR_REMOTE_GetSignalCount(IR_REMOTE_HandleTypeDef* handle, uint32_t* count);
void IR_REMOTE_PackMetadata(IR_REMOTE_HandleTypeDef* handle);
void IR_REMOTE_UnpackMetadata(IR_REMOTE_HandleTypeDef* handle);
void IR_REMOTE_FlushData(IR_REMOTE_HandleTypeDef* handle);
int IR_REMOTE_DeleteSignal(IR_REMOTE_HandleTypeDef* handle, uint32_t signalNum);

//IR_REMOTE_MGMT_HandleTypeDef func's
int IR_REMOTE_MGMT_StartListening_IT(IR_REMOTE_MGMT_HandleTypeDef* handle);
int IR_REMOTE_MGMT_StartSending_IT(IR_REMOTE_MGMT_HandleTypeDef* handle);

#endif /* INC_IR_REMOTE_H_ */
