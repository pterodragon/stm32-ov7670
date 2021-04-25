/*
 *	==========================================================================
 *   OV7670_control.c
 *   (c) 2014, Petr Machala
 *
 *   Updated by Erik Andre, 2015
 *
 *   Description:
 *   OV7670 camera configuration and control file.
 *   Optimized for 32F429IDISCOVERY board.
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *	==========================================================================
 */

#include "OV7670_control.h"
#include <misc.h>

extern void Serial_log(char *s);
extern void Serial_logi(int val);
extern void Serial_logih(int val);

// Image buffer
volatile uint16_t frame_buffer[IMG_ROWS * IMG_COLUMNS];

const uint8_t OV7670_reg[][2] = { { 0x12, 0x80 },

// Image format
		{ 0x12, 0x8 },		// 0x14 = QVGA size, RGB mode; 0x8 = QCIF, YUV, 0xc = QCIF (RGB)
		{ 0x11, 0b1000000 }
};

void Delay(volatile long nCount) {
	while (nCount--) {
	}
}

void MCO1_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_ClockSecuritySystemCmd(ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// GPIO config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		//PA8 - XCLK
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// GPIO AF config
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

	// MCO clock source
//	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4); // Using the fast PLL clock results in garbage output, using HSI (at 16Mhz works fine)
	RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1);
}

void SCCB_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// GPIO config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// GPIO AF config
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_I2C2);

	// I2C config
	I2C_DeInit(I2C2);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 90000;
	I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);
	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_Cmd(I2C2, ENABLE);
}

bool SCCB_read_reg(uint8_t reg_addr, uint8_t* data) {
	uint32_t timeout = 0x7FFFFF;

	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)) {
		if ((timeout--) == 0) {
			Serial_log("Busy Timeout\r\n");
			return true;
		}
	}

	I2C_GenerateSTART(I2C2, ENABLE);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			Serial_log("Start bit Timeout\r\n");
			return true;
		}
	}

	I2C_Send7bitAddress(I2C2, OV7670_WRITE_ADDR, I2C_Direction_Transmitter);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			Serial_log("Slave address timeout\r\n");
			return true;
		}
	}

	I2C_SendData(I2C2, reg_addr);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			Serial_log("Register timeout\r\n");
			return true;
		}
	}

	I2C_GenerateSTOP(I2C2, ENABLE);
    Delay(0x07FF);
	I2C_GenerateSTART(I2C2, ENABLE);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			Serial_log("read phase: Start bit Timeout\r\n");
			return true;
		}
	}

	I2C_Send7bitAddress(I2C2, OV7670_WRITE_ADDR, I2C_Direction_Receiver);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			Serial_log("read phase: Slave address timeout\r\n");
			return true;
		}
	}

    I2C_AcknowledgeConfig(I2C2,DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);

    while(I2C_GetFlagStatus(I2C2,I2C_FLAG_RXNE) == RESET){
		if ((timeout--) == 0) {
			Serial_log("read phase: read timeout\r\n");
			return true;
		}
	}

	*data = I2C_ReceiveData(I2C2);
    I2C_AcknowledgeConfig(I2C2, ENABLE);

	return false;
}


bool SCCB_write_reg(uint8_t reg_addr, uint8_t* data) {
	uint32_t timeout = 0x7FFFFF;

	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)) {
		if ((timeout--) == 0) {
			Serial_log("Busy Timeout\r\n");
			return true;
		}
	}

	// Send start bit
	I2C_GenerateSTART(I2C2, ENABLE);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			Serial_log("Start bit Timeout\r\n");
			return true;
		}
	}

	// Send slave address (camera write address)
	I2C_Send7bitAddress(I2C2, OV7670_WRITE_ADDR, I2C_Direction_Transmitter);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			Serial_log("Slave address timeout\r\n");
			return true;
		}
	}

	// Send register address
	I2C_SendData(I2C2, reg_addr);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			Serial_log("Register timeout\r\n");
			return true;
		}
	}

	// Send new register value
	I2C_SendData(I2C2, *data);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			Serial_log("Value timeout\r\n");
			return true;
		}
	}

	// Send stop bit
	I2C_GenerateSTOP(I2C2, ENABLE);
	return false;
}


bool OV7670_init(void) {
	uint8_t data, i = 0;
	bool err;
    int OV7670_REG_NUM = sizeof(OV7670_reg) / sizeof(OV7670_reg[0]);
    {
        bool ok = 1;
		err = SCCB_read_reg(0x0a, &data);
		Serial_log("reading register 0x0a: ");
		Serial_logi(data);
        ok = data == 0x76;
		Serial_log("\r\n");
		err = SCCB_read_reg(0x0b, &data);
		Serial_log("reading register 0x0b: ");
		Serial_logi(data);
		Serial_log("\r\n");
        if (err || !ok) return true;
    }

	// Configure camera registers
	for (i = 0; i < OV7670_REG_NUM; i++) {
		data = OV7670_reg[i][1];
		err = SCCB_write_reg(OV7670_reg[i][0], &data);
		Serial_log("Writing register: ");
		Serial_logi(i);
		Serial_log("\r\n");

		if (err == true) {
			Serial_log("Failed to update register\r\n");
			break;
		}

		Delay(0xFFFF);
	}

	return err;
}

void DCMI_DMA_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	DCMI_InitTypeDef DCMI_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	// GPIO config

	// PA4 - HREF (HSYNC), PA6 - PCLK (PIXCLK)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;	//PA4 - HREF (HSYNC)
															//PA6 - PCLK (PIXCLK)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PB6 - D5, PB7 - VSYNC, PB8 - D6, PB9 - D7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PC6 - D0, PC7 - D1, PC8 - D2, PC9 - D3, PC11 - D4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// GPIO AF config
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_DCMI);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_DCMI);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_DCMI);

	// DCMI config
	DCMI_DeInit();
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot; //DCMI_CaptureMode_SnapShot
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame; //DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_Init(&DCMI_InitStructure);
	DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
	DCMI_ITConfig(DCMI_IT_OVF, ENABLE);
	DCMI_ITConfig(DCMI_IT_ERR, ENABLE);

	// DMA config
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&DCMI->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) frame_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = IMG_ROWS * IMG_COLUMNS / 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TE, ENABLE);

	/* DMA2 IRQ channel Configuration */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
}
