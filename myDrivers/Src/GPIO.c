#include "GPIO.h"


void GPIO_Init(GPIO_TypeDef_t *GPIOx,GPIO_InitTypeDef_t *GPIO_ConfigStruct)
{
	uint32_t position;
	uint32_t fakePosition = 0;
	uint32_t lastPosition = 0;
	for(position = 0; position < 16; ++position)
	{
		fakePosition = (0x1 << position);
		lastPosition = (uint32_t)(GPIO_ConfigStruct->pinNumber) & fakePosition;

		if(fakePosition == lastPosition)
		{
			uint32_t tempValue = GPIOx->MODER;
			tempValue &= ~(0x3U << (position * 2));
			tempValue |= (GPIO_ConfigStruct->Mode << (position * 2) );
			GPIOx->MODER = tempValue;

			if(GPIO_ConfigStruct->Mode == GPIO_MODE_OUTPUT || GPIO_ConfigStruct->Mode == GPIO_MODE_AF)
			{
				tempValue = GPIOx->OTYPER;
				tempValue &= ~(0x1U << position);
				tempValue |=(GPIO_ConfigStruct->Otype << position);
				GPIOx->OTYPER = tempValue;


				tempValue = GPIOx->OSPEEDR;
			    tempValue &= ~(0x3U << (position * 2) );
				tempValue |=(GPIO_ConfigStruct->Speed << (position * 2) );
				GPIOx->OSPEEDR = tempValue;

			}

			tempValue = GPIOx->PUPDR;
			tempValue &= ~(0x3U << (position * 2) );
			tempValue |= (GPIO_ConfigStruct->PuPd << (position * 2) );
			GPIOx->PUPDR = tempValue;


			if(GPIO_ConfigStruct->Mode == GPIO_MODE_AF)
			{
				tempValue = GPIOx->AFR[position >> 3U];
				tempValue &= ~(0xFU << ( (position & 0x7U) * 4) );
				tempValue |= (GPIO_ConfigStruct->Alternate << ( (position & 0x7U) * 4) );
				GPIOx->AFR[position >> 3U] = tempValue;
			}

		}

	}
}

void GPIO_WritePin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber, GPIO_PinState_t pinState)
{
	if(pinState == GPIO_Pin_Set)
	{
		GPIOx->BSRR = pinNumber;
	}
	else
	{
		GPIOx->BSRR = (pinNumber << 16U);
	}
}

GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber)
{
	GPIO_PinState_t bitStatus = GPIO_Pin_Reset;

	if( (GPIOx->IDR & pinNumber) != GPIO_Pin_Reset)
	{
		bitStatus = GPIO_Pin_Set;
	}

	return bitStatus;
}


void GPIO_LockPin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber)
{
	uint32_t tempValue = (0x1U << 16U) | pinNumber;
	GPIOx->LCKR = tempValue;
	GPIOx->LCKR = pinNumber;
	GPIOx->LCKR = tempValue;
	tempValue = GPIOx->LCKR;
}


void GPIO_TogglePin(GPIO_TypeDef_t *GPIOx, uint16_t pinNumber)
{
	uint32_t tempValue = GPIOx->ODR;
	GPIOx->BSRR = ( (tempValue & pinNumber) << 16U )  | (~tempValue & pinNumber );
}

