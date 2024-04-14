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

#define IR_REMOTE_BAD_STATUS 	(int)-1
#define IR_REMOTE_GOOD_STATUS	(int)1
#define IR_REMOTE_DATA_OFFSET	(int)10 //10x32bit

typedef struct
{
	uint32_t signalBlock[1024];
	uint32_t signalOffset[5];
	uint32_t signalLength[5];
}IR_REMOTE_HandleTypeDef;


int  IR_REMOTE_GetSignal(IR_REMOTE_HandleTypeDef* handle, uint32_t* data, uint32_t signalNum);
int	 IR_REMOTE_GetSignalPtr(IR_REMOTE_HandleTypeDef* handle, uint32_t** dataPtr, uint32_t signalNum);
int  IR_REMOTE_PutSignal(IR_REMOTE_HandleTypeDef* handle, uint32_t* data, uint32_t signalNum);
void IR_REMOTE_GetSignalCount(IR_REMOTE_HandleTypeDef* handle, uint32_t* count);
void IR_REMOTE_PackMetadata(IR_REMOTE_HandleTypeDef* handle);
void IR_REMOTE_UnpackMetadata(IR_REMOTE_HandleTypeDef* handle);
void IR_REMOTE_FlushData(IR_REMOTE_HandleTypeDef* handle);

#endif /* INC_IR_REMOTE_H_ */
