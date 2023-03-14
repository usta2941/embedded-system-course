#include "USART.h"


static void closeUSART_ISR_Rx(USART_HandleTypedef_t *USART_Handle)
{
	USART_Handle->RxBufferSize = 0;
	USART_Handle->pRxBuffer = NULL;
	USART_Handle->RxStatus = USART_BUS_FREE;

	USART_Handle->Instance->CR1 &= ~(0x1U << USART_CR1_RxNEIE);
}



static void USART_ReceiveWith_IT(USART_HandleTypedef_t *USART_Handle)
{
	uint16_t *p16BitsBuffer;
	uint8_t *p8BitsBuffer;

	if( (USART_Handle->Init.WordLenght == USART_WORDLENGTH_9Bits) && (USART_Handle->Init.Parity == USART_PARITY_NONE))
	{
		p16BitsBuffer = (uint16_t*)USART_Handle->pRxBuffer;
		p8BitsBuffer = NULL;
	}
	else
	{
		p8BitsBuffer = (uint8_t*)USART_Handle->pRxBuffer;
		p16BitsBuffer = NULL;
	}

	if(p8BitsBuffer == NULL)
	{
		*p16BitsBuffer = (uint16_t)(USART_Handle->Instance->DR & 0x01FFU);
		USART_Handle->pRxBuffer += sizeof(uint16_t);
		USART_Handle->RxBufferSize -= 2;
	}
	else
	{
		if( (USART_Handle->Init.WordLenght == USART_WORDLENGTH_9Bits) && (USART_Handle->Init.Parity != USART_PARITY_NONE) )
		{
			*p8BitsBuffer = (uint8_t)(USART_Handle->Instance->DR & 0x00FFU);
			USART_Handle->pRxBuffer++;
			USART_Handle->RxBufferSize--;
		}
		else if((USART_Handle->Init.WordLenght == USART_WORDLENGTH_8Bits) && (USART_Handle->Init.Parity == USART_PARITY_NONE))
		{
			*p8BitsBuffer = (uint8_t)(USART_Handle->Instance->DR & 0x00FFU);
			USART_Handle->pRxBuffer++;
		    USART_Handle->RxBufferSize--;
		}
		else
		{
			*p8BitsBuffer = (uint8_t)(USART_Handle->Instance->DR & 0x007FU);
			USART_Handle->pRxBuffer++;
			USART_Handle->RxBufferSize--;
		}

		if(USART_Handle->RxBufferSize == 0)
		{
			closeUSART_ISR_Rx(USART_Handle);
		}


	}
}



static void closeUSART_ISR(USART_HandleTypedef_t *USART_Handle)
{
	USART_Handle->TxBufferSize = 0;
	USART_Handle->pTxBuffer = NULL;
	USART_Handle->TxStatus = USART_BUS_FREE;

	USART_Handle->Instance->CR1 &= ~(0x1U << USART_CR1_TxEIE);
}


static void USART_SendWith_IT(USART_HandleTypedef_t *USART_Handle)
{
	if( (USART_Handle->Init.WordLenght == USART_WORDLENGTH_9Bits) && (USART_Handle->Init.Parity == USART_PARITY_NONE))
	{
		uint16_t *p16BitsData = (uint16_t*)(USART_Handle->pTxBuffer);

		USART_Handle->Instance->DR = (uint16_t)(*p16BitsData & (uint16_t)0x01FF);
		USART_Handle->pTxBuffer += sizeof(uint16_t);
		USART_Handle->TxBufferSize -=2;
	}
	else
	{
		USART_Handle->Instance->DR = (uint8_t)(*(USART_Handle->pTxBuffer) & (uint8_t)0x00FF);
		USART_Handle->pTxBuffer++;
		USART_Handle->TxBufferSize--;
	}

	if(USART_Handle->TxBufferSize == 0)
	{
		closeUSART_ISR(USART_Handle);
	}
}
void USART_Init(USART_HandleTypedef_t *USART_Handle)
{
	uint32_t mantissaPart = 0;
	uint32_t fractionPart = 0;
	uint32_t USART_DIV_Value = 0;
	uint32_t tempValue = 0;

	uint32_t periphClock = 0;
	uint32_t tempReg = 0;

	tempReg = USART_Handle->Instance->CR1;

	tempReg |= (USART_Handle->Init.OverSampling) | (USART_Handle->Init.WordLenght) | (USART_Handle->Init.Mode) | \
			   (USART_Handle->Init.Parity);

	 USART_Handle->Instance->CR1 = tempReg;

	 tempReg = USART_Handle->Instance->CR2;

	 tempReg &= ~(0x3U << UART_CR2_STOP);

	 tempReg |= (USART_Handle->Init.StopBits);

	 USART_Handle->Instance->CR2 = tempReg;

	 tempReg = USART_Handle->Instance->CR3;

	 tempReg |= (USART_Handle->Init.HardwareFlowControl);

	 USART_Handle->Instance->CR3 = tempReg;

	 if(USART_Handle->Instance == USART1 || USART_Handle->Instance == USART6)
	 {
		 periphClock = RCC_GetPClock2();
	 }
	 else
	 {
		 periphClock = RCC_GetPClock1();
	 }

	 if(USART_Handle->Init.OverSampling == USART_OVERSAMPLE_8)
	 {
		 USART_DIV_Value = __USART_DIV_VALUE_8(periphClock, USART_Handle->Init.BaudRate);
		 mantissaPart = (USART_DIV_Value / 100U);
		 fractionPart = (USART_DIV_Value) - (mantissaPart * 100U);

		 fractionPart = ( ( (fractionPart * 8U) + 50U) / 100U) & (0x7U);
	 }
	 else
	 {
		 USART_DIV_Value = __USART_DIV_VALUE_16(periphClock, USART_Handle->Init.BaudRate);
		 mantissaPart = (USART_DIV_Value / 100U);
		 fractionPart = (USART_DIV_Value) - (mantissaPart * 100U);

		 fractionPart = ( ( (fractionPart * 16U) + 50U) / 100U) & (0xFU);
	 }

	 tempValue |= (mantissaPart << 4U);
	 tempValue |= (fractionPart << 0U);
	 USART_Handle->Instance->BRR = tempValue;
}


void USART_TransmitData(USART_HandleTypedef_t *USART_Handle, uint8_t *pData, uint16_t dataSize)
{
	uint16_t *data16Bits;

	if( (USART_Handle->Init.WordLenght == USART_WORDLENGTH_9Bits) && (USART_Handle->Init.Parity == USART_PARITY_NONE) )
	{
		data16Bits = (uint16_t*)pData;
	}
	else
	{
		data16Bits = NULL;
	}

	while(dataSize > 0)
	{
		while(!USART_GetFlagStatus(USART_Handle, USART_TxE_FLAG));

		if(data16Bits == NULL)
		{
			USART_Handle->Instance->DR = (uint16_t)(*pData & 0xFFu);
			pData++;
			dataSize--;
		}
		else
		{
			USART_Handle->Instance->DR = (uint16_t)(*data16Bits & (0x1FFU));
			data16Bits++;
		}

		while( !(USART_GetFlagStatus(USART_Handle, USART_TC_FLAG)) );
	}

}


void USART_ReceiveData(USART_HandleTypedef_t *USART_Handle, uint8_t *pBuffer, uint16_t dataSize)
{
	uint16_t *p16BitsBuffer;
	uint8_t *p8BitsBuffer;

	if( (USART_Handle->Init.WordLenght == USART_WORDLENGTH_9Bits) && (USART_Handle->Init.Parity == USART_PARITY_NONE))
	{
		p16BitsBuffer = (uint16_t*)pBuffer;
		p8BitsBuffer = NULL;
	}
	else
	{
		p8BitsBuffer = (uint8_t*)pBuffer;
		p16BitsBuffer = NULL;
	}

	while(dataSize > 0)
	{
		while( !(USART_GetFlagStatus(USART_Handle, USART_RxNE_FLAG)));

	}
	if(p8BitsBuffer == NULL)
	{
		*p16BitsBuffer = (uint16_t)(USART_Handle->Instance->DR & 0x01FFU);
		p16BitsBuffer++;
		dataSize -= 2;
	}
	else
	{
		if( (USART_Handle->Init.WordLenght == USART_WORDLENGTH_9Bits) && (USART_Handle->Init.Parity != USART_PARITY_NONE))
		{
			*p8BitsBuffer = (uint8_t)(USART_Handle->Instance->DR & 0x00FU);
			p8BitsBuffer++;
			dataSize--;
		}
		else if( (USART_Handle->Init.WordLenght == USART_WORDLENGTH_8Bits) && (USART_Handle->Init.Parity == USART_PARITY_NONE))
		{
			*p8BitsBuffer = (uint8_t)(USART_Handle->Instance->DR & 0x00FU);
			p8BitsBuffer++;
			dataSize--;
		}
		else
		{
			*p8BitsBuffer = (uint8_t)(USART_Handle->Instance->DR & 0x007U);
			p8BitsBuffer++;
			dataSize--;
		}
	}

}

void USART_TransmitData_IT(USART_HandleTypedef_t *USART_Handle, uint8_t *pData, uint16_t dataSize)
{
	USART_BusState_t usartBusState = USART_Handle->TxStatus;

	if(usartBusState != USART_BUS_Tx)
	{
		USART_Handle->pTxBuffer = (uint8_t*)pData;
		USART_Handle->TxBufferSize = (uint16_t)dataSize;
		USART_Handle->TxStatus = USART_BUS_Tx;
		USART_Handle->TxISR_Function = USART_SendWith_IT;


		USART_Handle->Instance->CR1 |= (0x1U << USART_CR1_TxEIE);
	}

}


void USART_ReceiveData_IT(USART_HandleTypedef_t *USART_Handle, uint8_t *pBuffer, uint16_t dataSize)
{
	USART_BusState_t usart_BusState = USART_Handle->RxStatus;

	if(usart_BusState != USART_BUS_Rx)
	{
		USART_Handle->pRxBuffer = (uint8_t*)pBuffer;
		USART_Handle->RxBufferSize = (uint16_t)dataSize;
		USART_Handle->RxStatus = USART_BUS_Rx;
		USART_Handle->RxISR_Function = USART_ReceiveWith_IT;

		USART_Handle->Instance->CR1 |= (0x1U << USART_CR1_RxNEIE);
	}
}




void USART_PeriphCmd(USART_HandleTypedef_t *USART_Handle, FunctionalState_t stateOfUSART)
{
	if(stateOfUSART == ENABLE)
	{
		USART_Handle->Instance->CR1 |= (0x1U << USART_CR1_UE);
	}
	else
	{
		USART_Handle->Instance->CR1 &= ~(0x1U << USART_CR1_UE);
	}
}


USART_FlagStatus_t USART_GetFlagStatus(USART_HandleTypedef_t *USART_Handle, uint16_t flagName )
{
	return (USART_Handle->Instance->SR & flagName) ? USART_FLAG_SET : USART_FLAG_RESET;

}

void USART_InterruptHandler(USART_HandleTypedef_t *USART_Handle)
{
	uint8_t flagValue = 0;
	uint8_t interruptValue = 0;

	flagValue = (uint8_t)( (USART_Handle->Instance->SR >> 7U) & 0x1U);
	interruptValue = (uint8_t)( (USART_Handle->Instance->CR1 >> 7U) & 0x1U);

	if(flagValue && interruptValue)
	{
		USART_Handle->TxISR_Function(USART_Handle);
	}


	flagValue = (uint8_t)( (USART_Handle->Instance->SR >> 5U) & 0x1U);
	interruptValue = (uint8_t)( (USART_Handle->Instance->CR1 >> 5U) & 0x1U);

	if(flagValue && interruptValue)
		{
			USART_Handle->RxISR_Function(USART_Handle);
		}


}




