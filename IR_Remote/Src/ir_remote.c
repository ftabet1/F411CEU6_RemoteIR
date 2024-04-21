/*
 * ir_remote.c
 *
 *  Created on: Apr 8, 2024
 *      Author: User
 */

#include "ir_remote.h"

void IR_REMOTE_FlushData(IR_REMOTE_HandleTypeDef* handle)
{
	memset(handle->signalBlock, 0, 1024*4);
	memset(handle->signalOffset, 0, 5*4);
	memset(handle->signalLength, 0, 5*4);
}

void IR_REMOTE_PackMetadata(IR_REMOTE_HandleTypeDef* handle)
{
	for(int i = 0; i < 5; i++)
	{
		handle->signalBlock[i] = handle->signalOffset[i];
		handle->signalBlock[i+5] = handle->signalLength[i];
	}
}

void IR_REMOTE_UnpackMetadata(IR_REMOTE_HandleTypeDef* handle)
{
	for(int i = 0; i < 5; i++)
	{
		handle->signalOffset[i]  = handle->signalBlock[i];
		handle->signalLength[i] = handle->signalBlock[i+5];
	}
}

void IR_REMOTE_GetSignalCount(IR_REMOTE_HandleTypeDef* handle, uint32_t* count)
{
	uint32_t cnt = 0;
	for(int i = 0; i < 5; i++)
	{
		if(handle->signalOffset != 0)	cnt++;
	}
	(*count) = cnt;
}

int IR_REMOTE_GetSignal(IR_REMOTE_HandleTypeDef* handle, uint32_t* data, uint32_t signalNum)
{
	if(signalNum>4 || signalNum<0) return IR_REMOTE_BAD_STATUS;
	if(handle->signalOffset[signalNum] == 0) return IR_REMOTE_BAD_STATUS;
	for(int i = 0; i<200; i++)
	{
		data[i] = handle->signalBlock[handle->signalOffset[signalNum] + i];
	}
	return IR_REMOTE_GOOD_STATUS;
}

int IR_REMOTE_GetSignalPtr(IR_REMOTE_HandleTypeDef* handle, uint32_t** dataPtr, uint32_t signalNum)
{
	if(signalNum>4 || signalNum<0) return IR_REMOTE_BAD_STATUS;
	if(handle->signalOffset[signalNum] == 0) return IR_REMOTE_BAD_STATUS;
	(*dataPtr) = (handle->signalBlock)+(handle->signalOffset[signalNum]);
	return IR_REMOTE_GOOD_STATUS;
}

int IR_REMOTE_PutSignal(IR_REMOTE_HandleTypeDef* handle, uint32_t* data, uint32_t signalNum)
{
	if(signalNum>4 || signalNum<0) return IR_REMOTE_BAD_STATUS;
	handle->signalLength[signalNum] = 200;
	handle->signalOffset[signalNum] = signalNum*200+IR_REMOTE_DATA_OFFSET;
	for(int i = 0; i < 200; i++)
	{
	handle->signalBlock[((handle->signalOffset[signalNum])+i)] = data[i];
	}
	return IR_REMOTE_GOOD_STATUS;
}

int IR_REMOTE_DeleteSignal(IR_REMOTE_HandleTypeDef* handle, uint32_t signalNum)
{
	if(signalNum>4 || signalNum<0) return IR_REMOTE_BAD_STATUS;
	//if(handle->signalLength[signalNum] == 0 || handle->signalLength[signalNum] == 0xffffffff || handle->signalOffset[signalNum] == 0 || handle->signalOffset[signalNum] == 0xffffffff) return IR_REMOTE_BAD_STATUS;
	for(int i = handle->signalOffset[signalNum]; i < handle->signalOffset[signalNum] + handle->signalLength[signalNum]; i++)
	{
		handle->signalBlock[i] = 0;
	}
	handle->signalLength[signalNum] = 0;
	handle->signalOffset[signalNum] = 0;
	return IR_REMOTE_GOOD_STATUS;
}

int IR_REMOTE_MGMT_StartListening_IT(IR_REMOTE_MGMT_HandleTypeDef* handle)
{
	if(handle->mode != IR_REMOTE_MODE_IDLE) return IR_REMOTE_BAD_STATUS;
	handle->mode = IR_REMOTE_MODE_LISTENING;
	TIM5->SR &= ~(TIM_SR_UIF);
	EXTI->IMR |= ((1 << 13) | (1 << 14));
	return IR_REMOTE_GOOD_STATUS;
}

int IR_REMOTE_MGMT_StartSending_IT(IR_REMOTE_MGMT_HandleTypeDef* handle)
{
	if(handle->mode != IR_REMOTE_MODE_IDLE) return IR_REMOTE_BAD_STATUS;
	handle->mode = IR_REMOTE_MODE_EMITTING;
	TIM2->CR1 &= ~(TIM_CR1_CEN);
	TIM2->ARR = 1000;
	TIM2->CNT = 0;
	TIM2->CR1 |= (TIM_CR1_CEN);
	return IR_REMOTE_GOOD_STATUS;
}
