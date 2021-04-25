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

#define GPIO_MODE             0x00000003U

void i2c_sda_set_io_mode(bool x) {
  // true: output mode; false: input mode
  uint32_t temp = GPIOC->MODER;
  uint32_t position = GPIO_Pin_12;
  uint32_t mode = x;
  temp &= ~(GPIO_MODER_MODER0 << (position * 2U));
  temp |= ((mode & GPIO_MODE) << (position * 2U));
  GPIOC->MODER = temp;
}

void i2c_scl_set(bool x) {
  if (x) {
    GPIOB->BSRRL = GPIO_Pin_10;
    GPIOB->BSRRH = 0;
  } else {
    GPIOB->BSRRL = 0;
    GPIOB->BSRRH = GPIO_Pin_10;
  }
}

void i2c_sda_set(bool x) {
  if (x) {
    GPIOC->BSRRL = GPIO_Pin_12;
    GPIOC->BSRRH = 0;
  } else {
    GPIOC->BSRRL = 0;
    GPIOC->BSRRH = GPIO_Pin_12;
  }
}


// Image buffer
volatile uint16_t frame_buffer[IMG_ROWS * IMG_COLUMNS];

const uint8_t OV7670_reg[][2] = { { 0x12, 0x80 },

// Image format
		{ 0x12, 0x8 },		// 0x14 = QVGA size, RGB mode; 0x8 = QCIF, YUV, 0xc = QCIF (RGB)
		{ 0x11, 0b1000000 }, //

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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

    i2c_sda_set_io_mode(1);
    i2c_sda_set(1);
    i2c_scl_set(1);

    // // don't use alternate function in bit bang version
	// GPIO AF config
	// GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	// GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_I2C2);

	// I2C config
	// I2C_DeInit(I2C2);
	// I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	// I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	// I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	// I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; // Changed
	// I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	// I2C_InitStructure.I2C_ClockSpeed = 100000;
	// I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);
	// I2C_Init(I2C2, &I2C_InitStructure);
	// I2C_Cmd(I2C2, ENABLE);
}

static u8 fac_us = 0;
static u16 fac_ms = 0;

void delay_init(u8 SYSCLK)
{
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); 
  fac_us=SYSCLK/8;
  fac_ms=(u16)fac_us*1000;
}								    

void delay_us(u32 nus)
{		
  u32 temp;	    	 
  SysTick->LOAD=nus*fac_us;
  SysTick->VAL=0x00;
  SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
  do {
    temp=SysTick->CTRL;
  } while((temp&0x01)&&!(temp&(1<<16)));
  SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
  SysTick->VAL =0X00;
}

bool i2c_sda_read() {
  return GPIOC->IDR & GPIO_Pin_12;
}

bool SCCB_read_reg(uint8_t reg_addr, uint8_t* data_out) {
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
			Serial_log("Read: Start bit Timeout\r\n");
			return true;
		}
	}

	// Send slave address (camera write address)
	I2C_Send7bitAddress(I2C2, OV7670_WRITE_ADDR, I2C_Direction_Transmitter);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			Serial_log("Read: write Slave address timeout\r\n");
			return true;
		}
	}
    
    I2C_ClearFlag(I2C2, I2C_FLAG_AF); // XXX
	// Send register address
	I2C_SendData(I2C2, 0x0a);

	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			Serial_log("Register timeout\r\n");
			return true;
		}
    }


    I2C_AcknowledgeConfig(I2C2, DISABLE);

    uint8_t data = I2C_ReceiveData(I2C2);
	// Send stop bit
	I2C_GenerateSTOP(I2C2, ENABLE);
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

void sccb_start(void) {
  i2c_sda_set(1);
  i2c_scl_set(1);
  delay_us(5);  
  i2c_sda_set(0);
  delay_us(5);	 
  i2c_scl_set(0);
}

void sccb_stop(void) {
  i2c_sda_set(0);
  delay_us(5);	 
  i2c_scl_set(1);
  delay_us(5); 
  i2c_sda_set(1);
  delay_us(5);
}  

void sccb_no_ack(void) {
  delay_us(5);
  i2c_sda_set(1);
  i2c_scl_set(1);
  delay_us(5);
  i2c_scl_set(0);
  delay_us(5);
  i2c_sda_set(0);
  delay_us(5);
}


u8 sccb_write_byte(u8 dat) {
  u8 j,res;	 
  for(j=0;j<8;j++)
  {
    if(dat&0x80)i2c_sda_set(1);	
    else i2c_sda_set(0);
    dat<<=1;
    delay_us(5);
    i2c_scl_set(1);	
    delay_us(5);
    i2c_scl_set(0);		   
  }			 
  i2c_sda_set_io_mode(0);
  delay_us(5);
  i2c_scl_set(1);	
  delay_us(5);
  res = i2c_sda_read(); // can check if ack or not
  i2c_scl_set(0);		   
  i2c_sda_set_io_mode(1);
  return res;  
}	 

u8 sccb_read_byte() {
  u8 temp=0,j;    
  i2c_sda_set_io_mode(0);
  for(j=8;j>0;j--) {		     	  
    delay_us(5);
    i2c_scl_set(1);
    temp=temp<<1;
    if(i2c_sda_read())temp++;   
    delay_us(5);
    i2c_scl_set(0);
  }	
  i2c_sda_set_io_mode(1);
  return temp;
} 							    

#define SCCB_ID 0x42

u8 sccb_write_reg(u8 reg,u8 data) {
  u8 res=0;
  sccb_start();
  if(sccb_write_byte(SCCB_ID))res=1;
  if(sccb_write_byte(reg))res=1;
  if(sccb_write_byte(data))res=1;
  sccb_stop();	  
  return res;
}		  					    

u8 sccb_read_reg(u8 reg) {
  u8 val=0;
  sccb_start();
  sccb_write_byte(SCCB_ID);
  sccb_write_byte(reg);
  sccb_stop();
  delay_us(20);	   
  sccb_start();
  sccb_write_byte(SCCB_ID|0X01);
  val=sccb_read_byte();
  sccb_no_ack();
  sccb_stop();
  return val;
}



bool OV7670_init(void) {
	uint8_t data, i = 0;
	bool err;

    delay_init(170);
    i2c_sda_set_io_mode(1);
    // i2c_sda_set(1);
    // i2c_scl_set(1);
    delay_us(20);

    {
      u8 res = sccb_read_reg(0x13);
      u8 res2 = sccb_read_reg(0x0b);
      u8 res3 = sccb_read_reg(0x0a);
      Serial_log("res:");
      Serial_logi(res);
      Serial_log("\r\n");
      Serial_log("res2:");
      Serial_logi(res2);
      Serial_log("\r\n");
      Serial_log("res3:");
      Serial_logi(res3);
      Serial_log("\r\n");
      if (res3 != 0x76) return true;
    }

	// Configure camera registers
    int retries = 100;
    int OV7670_REG_NUM  = sizeof(OV7670_reg) / sizeof(OV7670_reg[0]);
    Serial_log("OV7670_REG_NUM ");
    Serial_logi(OV7670_REG_NUM);
    Serial_log("\r\n");
    do {
      Serial_log("retries left ");
      Serial_logi(retries);
      Serial_log("\r\n");
      for (i = 0; i < OV7670_REG_NUM; i++) {
          data = OV7670_reg[i][1];
            // Serial_log("update register ");
            // Serial_logi(i);
            // Serial_log(", ");
            // Serial_logih(OV7670_reg[i][0]);
            // Serial_log("\r\n");
            err = sccb_write_reg(OV7670_reg[i][0], data);
            if (err == true) {
                Serial_log("-- Failed to update register ");
                Serial_logi(i);
                Serial_log("\r\n");
                break;
            }
            u8 res = sccb_read_reg(OV7670_reg[i][0]);
            if (OV7670_reg[i][0] != 0x12 && res != data) {
                err = true;
                Serial_log("register not consistent ");
                Serial_logih(OV7670_reg[i][0]);
                Serial_log("\r\n");
                Serial_log("res = ");
                Serial_logih(res);
                Serial_log("\r\n");
                break;
            }

          Delay(0xFFFF);
      }
      if (retries-- == 0) return true;
    } while (err);
    Serial_log("register update done\r\n");

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
