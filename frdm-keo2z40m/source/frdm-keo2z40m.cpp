/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    frdm-keo2z40m.cpp
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKE02Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "fsl_adc.h"
#include "fsl_i2c.h"
#include "fsl_uart.h"
#include "uart_messages.h"
#include "UartReadBuffer.h"
#include "UartWriteBuffer.h"
#include <Errors.h>

/* TODO: insert other definitions and declarations here. */
#define ACCEL_I2C_CLK_SRC  kCLOCK_BusClk
#define ACCEL_I2C_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)

#define I2C_RELEASE_SDA_PORT  PORTA
#define I2C_RELEASE_SCL_PORT  PORTA
#define I2C_RELEASE_SDA_GPIO  GPIOA
#define I2C_RELEASE_SDA_PIN   2U
#define I2C_RELEASE_SCL_GPIO  GPIOA
#define I2C_RELEASE_SCL_PIN   3U
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_BAUDRATE       100000U
#define FXOS8700_WHOAMI    0xC7U
#define MMA8451_WHOAMI     0x1AU
#define MMA8652_WHOAMI     0x4AU
#define ACCEL_STATUS       0x00U
#define ACCEL_XYZ_DATA_CFG 0x0EU
#define ACCEL_CTRL_REG1    0x2AU
/* FXOS8700 and MMA8451 have the same who_am_i register address. */
#define ACCEL_WHOAMI_REG 0x0DU
#define ACCEL_READ_TIMES 10U


UartReadBuffer read_buffer;
UartWriteBuffer write_buffer;
status_t receive_status;
Command received_command;
Reply outgoing_reply;

adc_config_t config;
adc_channel_config_t channel_config;

uint8_t MSBshift = 8U;
uint8_t LSBshift = 2U;
/* FXOS8700, MMA8652 and MMA8451 device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

i2c_master_handle_t g_m_handle;

uint8_t g_accel_addr_found = 0x00;

volatile bool completionFlag = false;
volatile bool nakFlag        = false;

void process_command(const Command& command, Reply& reply);

uint32_t read_thermistor(void);
void read_accelerometer(int32_t &x, int32_t &y, int32_t &z);
void BOARD_I2C_ReleaseBus(void);
static bool I2C_ReadAccelWhoAmI(void);
static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);


static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    i2c_release_bus_delay();
}

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
    {
        nakFlag = true;
    }
}

static bool I2C_ReadAccelWhoAmI(void)
{
    /*
    How to read the device who_am_I value ?
    Start + Device_address_Write , who_am_I_register;
    Repeart_Start + Device_address_Read , who_am_I_value.
    */
    uint8_t who_am_i_reg          = ACCEL_WHOAMI_REG;
    uint8_t who_am_i_value        = 0x00;
    uint8_t accel_addr_array_size = 0x00;
    bool find_device              = false;
    uint8_t i                     = 0;
    uint32_t sourceClock          = 0;

    i2c_master_config_t masterConfig;

    /*
     * masterConfig.baudRate_Bps = 100000U;
     * masterConfig.enableStopHold = false;
     * masterConfig.glitchFilterWidth = 0U;
     * masterConfig.enableMaster = true;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);

    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    sourceClock = ACCEL_I2C_CLK_FREQ;

    I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &masterConfig, sourceClock);

    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress   = g_accel_address[0];
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = &who_am_i_reg;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferNoStopFlag;

    accel_addr_array_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);

    for (i = 0; i < accel_addr_array_size; i++)
    {
        masterXfer.slaveAddress = g_accel_address[i];

        I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag     = false;
            find_device        = true;
            g_accel_addr_found = masterXfer.slaveAddress;
            break;
        }
    }

    if (find_device == true)
    {
        masterXfer.direction      = kI2C_Read;
        masterXfer.subaddress     = 0;
        masterXfer.subaddressSize = 0;
        masterXfer.data           = &who_am_i_value;
        masterXfer.dataSize       = 1;
        masterXfer.flags          = kI2C_TransferRepeatedStartFlag;

        I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;
            if (who_am_i_value == FXOS8700_WHOAMI)
            {
                //PRINTF("Found an FXOS8700 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else if (who_am_i_value == MMA8451_WHOAMI)
            {
                //PRINTF("Found an MMA8451 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else if (who_am_i_value == MMA8652_WHOAMI)
            {
                LSBshift = 4U;
                //PRINTF("Found an MMA8652 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else
            {
                //PRINTF("Found a device, the WhoAmI value is 0x%x\r\n", who_am_i_value);
                //PRINTF("It's not MMA8451 or FXOS8700 or MMA8652. \r\n");
                //PRINTF("The device address is 0x%x. \r\n", masterXfer.slaveAddress);
                return false;
            }
        }
        else
        {
            //PRINTF("Not a successful i2c communication \r\n");
            return false;
        }
    }
    else
    {
        //PRINTF("\r\n Do not find an accelerometer device ! \r\n");
        return false;
    }
}

static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress   = device_addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = &value;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = device_addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = rxBuff;
    masterXfer.dataSize       = rxSize;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}


/*
 * @brief   Application entry point.
 */
int main(void) {

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	BOARD_I2C_ReleaseBus();
	//BOARD_I2C_ConfigurePins();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
#endif

	LED_RED1_INIT(true);
	LED_GREEN1_INIT(true);
	LED_BLUE_INIT(true);

	config.ResolutionMode = kADC_Resolution8BitMode;
	config.clockDivider = kADC_ClockDivider8;
	config.clockSource =  kADC_ClockSourceAlt0;
	config.enableLongSampleTime = false;
	config.enableLowPower =false;
	config.referenceVoltageSource = kADC_ReferenceVoltageSourceAlt0 ;

	ADC_Init(ADC, &config);

	channel_config.channelNumber = 12;
	channel_config.enableContinuousConversion = false;
	channel_config.enableInterruptOnConversionCompleted =false;

	ADC_EnableAnalogInput(ADC, 1U << 12, true);
	ADC_EnableAnalogInput(ADC, 1U << 13, true);

	while(1){

		// Read the first byte from uart. The first byte indicates how many bytes will follow.
		uint8_t n_bytes = 0;
		receive_status = UART_ReadBlocking(UART1, &n_bytes, 1);
		if(kStatus_Success == receive_status)
		{
			// Read the actual data to be deserialized.
			uint8_t byte;
			for(uint8_t i = 0; (i < n_bytes) && (kStatus_Success == receive_status); ++i)
			{
				receive_status = UART_ReadBlocking(UART1, &byte, 1);
				read_buffer.push(byte);
			}

			if(kStatus_Success == receive_status)
			{
				// Deserialize the data received.
				auto deserialize_status = received_command.deserialize(read_buffer);
				if(::EmbeddedProto::Error::NO_ERRORS == deserialize_status) {
					// Process the command.
					process_command(received_command, outgoing_reply);
					// Serialize the data.
					auto serialization_status = outgoing_reply.serialize(write_buffer);
					if(::EmbeddedProto::Error::NO_ERRORS == serialization_status)
					{
						// first transmit the number of bytes in the message.
						n_bytes = write_buffer.get_size();
						UART_WriteBlocking(UART1, &n_bytes, 1);
						// Now transmit the actual data.
						UART_WriteBlocking(UART1, write_buffer.get_data(), write_buffer.get_size());
					}
				}
				// Clear the buffers after we are done.
				read_buffer.clear();
				write_buffer.clear();

				received_command.clear();

			}
		}
	}

	return 0 ;
}

//! The functions takes a command and responds to it.
/*!
 * \param[in] command The received command.
 * \param[out] reply The reply to be send over uart.
 */
void process_command(const Command& command, Reply& reply)
{
  switch(command.action())
  {
    case Command::Action::GetAccelerometer:
    {
    	int32_t x,y,z;
    	read_accelerometer(x,y,z);
    	reply.mutable_accelerometer().set_x(x);
    	reply.mutable_accelerometer().set_y(y);
    	reply.mutable_accelerometer().set_z(z);
    	reply.set_action(Reply::Action::SendAccelerometer);
    	break;
    }

    case Command::Action::GetThermistor:
    {
    	uint32_t value = read_thermistor();
    	reply.set_thermistor(value);
    	reply.set_action(Reply::Action::SendThermistor);
    	break;
    }

    case Command::Action::SetLed:
    {
    	if(command.get_led().get_red())
    	{
    		LED_RED1_ON();
    	}
    	else{
    		LED_RED1_OFF();
    	}

    	if(command.get_led().get_green())
    	{
    		LED_GREEN1_ON();
    	}
    	else{
    		LED_GREEN1_OFF();
    	}

    	if(command.get_led().get_blue())
    	{
    		LED_BLUE_ON();
    	}
    	else{
    		LED_BLUE_OFF();
    	}

    	reply.set_action(Reply::Action::DoNothing);

    	break;
    }

    default:

    	break;
  }

}

uint32_t read_thermistor(void)
{
	channel_config.channelNumber = 12;
	ADC_SetChannelConfig(ADC, &channel_config); //This starts a ADC conversion
	while (!ADC_GetChannelStatusFlags(ADC))
	{
	}

	uint32_t val1 = ADC_GetChannelConversionValue(ADC);

	channel_config.channelNumber = 13;
	ADC_SetChannelConfig(ADC, &channel_config); //This starts a ADC conversion
	while (!ADC_GetChannelStatusFlags(ADC))
	{
	}

	uint32_t val2 = ADC_GetChannelConversionValue(ADC);

	uint32_t thermistor_value = val1-val2;
	//PRINTF("ADC Value: %u\r\n", thermistor_value);

	return thermistor_value;
}

void read_accelerometer(int32_t &x, int32_t &y, int32_t &z)
{
	bool isThereAccel;

	I2C_MasterTransferCreateHandle(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, i2c_master_callback, NULL);
	isThereAccel = I2C_ReadAccelWhoAmI();

	/*  read the accel xyz value if there is accel device on board */
	if (true == isThereAccel)
	{
		uint8_t databyte  = 0;
		uint8_t write_reg = 0;
		uint8_t readBuff[7];
		uint8_t status0_value = 0;

		/*  please refer to the "example FXOS8700CQ Driver Code" in FXOS8700 datasheet. */
		/*  write 0000 0000 = 0x00 to accelerometer control register 1 */
		/*  standby */
		/*  [7-1] = 0000 000 */
		/*  [0]: active=0 */
		write_reg = ACCEL_CTRL_REG1;
		databyte  = 0;
		I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

		/*  write 0000 0001= 0x01 to XYZ_DATA_CFG register */
		/*  [7]: reserved */
		/*  [6]: reserved */
		/*  [5]: reserved */
		/*  [4]: hpf_out=0 */
		/*  [3]: reserved */
		/*  [2]: reserved */
		/*  [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB */
		/*  databyte = 0x01; */
		write_reg = ACCEL_XYZ_DATA_CFG;
		databyte  = 0x01;
		I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

		/*  write 0000 1101 = 0x0D to accelerometer control register 1 */
		/*  [7-6]: aslp_rate=00 */
		/*  [5-3]: dr=001 for 200Hz data rate (when in hybrid mode) */
		/*  [2]: lnoise=1 for low noise mode */
		/*  [1]: f_read=0 for normal 16 bit reads */
		/*  [0]: active=1 to take the part out of standby and enable sampling */
		/*   databyte = 0x0D; */
		write_reg = ACCEL_CTRL_REG1;
		databyte  = 0x0d;
		I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);
		//PRINTF("The accel values:\r\n");
		status0_value = 0;
		/*  wait for new data are ready. */
		while (status0_value != 0xff)
		{
			I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, ACCEL_STATUS, &status0_value, 1);
		}

		/*  Multiple-byte Read from STATUS (0x00) register */
		I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, ACCEL_STATUS, readBuff, 7);

		status0_value = readBuff[0];
		x             = ((int16_t)(((readBuff[1] << MSBshift) | readBuff[2]))) >> LSBshift;
		y             = ((int16_t)(((readBuff[3] << MSBshift) | readBuff[4]))) >> LSBshift;
		z             = ((int16_t)(((readBuff[5] << MSBshift) | readBuff[6]))) >> LSBshift;

		//PRINTF("status_reg = 0x%x , x = %5d , y = %5d , z = %5d \r\n", status0_value, x, y, z);

	}
}
