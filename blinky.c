//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// Copyright (c) 2012-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.2.0.295 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif
//*********************************************************
// Function: I2C0_Init
// Description: Initializes I2C0 module on TM4C123G for Master mode
//*********************************************************
void I2C0_Init(void){
    // Enable I2C0 and GPIOB peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Wait until peripherals are ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    // Configure PB2 as I2C SCL and PB3 as I2C SDA
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);  // Configure PB2 -> SCL
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);  // Configure PB3 -> SDA

    // Initialize I2C master with system clock
    // false = standard mode (100kHz), true = fast mode (400kHz)
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    // Clear any previous I2C status
    I2CMasterIntClear(I2C0_BASE);
}
void PCA9685_Init(void){
    //
    // MODE1 register = normal mode
    //
    PCA9685_Write(0x00, 0x00);

    SysCtlDelay(SysCtlClockGet() / 100);
}
//*****************************************************************************
//
// Function: PCA9685_Write
//
// Description:
// Writes a single byte into a register of the PCA9685 PWM driver
// using I2C communication.
//
// Parameters:
// reg  - Register address inside the PCA9685
// data - Data byte to write into the register
//
// Example:
// PCA9685_Write(0x00, 0x00);
//
// This writes 0x00 into MODE1 register.
//
//*****************************************************************************
void PCA9685_Write(uint8_t reg, uint8_t data){
    //
    // Variable used to store I2C error status
    //
    uint32_t status;

    //
    // Configure I2C0 master to communicate with slave address 0x40
    //
    // false = WRITE operation
    // true  = READ operation
    //
    I2CMasterSlaveAddrSet(
        I2C0_BASE,
        0x40,
        false
    );

    //
    // Load register address into I2C transmit FIFO
    //
    // Example:
    // reg = 0x00 -> MODE1 register
    //
    I2CMasterDataPut(
        I2C0_BASE,
        reg
    );

    //
    // Generate:
    // START condition
    // Send slave address
    // Send register byte
    //
    I2CMasterControl(
        I2C0_BASE,
        I2C_MASTER_CMD_BURST_SEND_START
    );

    //
    // Wait until I2C transaction completes
    //
    while(I2CMasterBusy(I2C0_BASE));

    //
    // Load data byte into transmit FIFO
    //
    // Example:
    // data = 0x00
    //
    I2CMasterDataPut(
        I2C0_BASE,
        data
    );

    //
    // Generate:
    // Send data byte
    // STOP condition
    //
    I2CMasterControl(
        I2C0_BASE,
        I2C_MASTER_CMD_BURST_SEND_FINISH
    );

    //
    // Wait until transaction completes
    //
    while(I2CMasterBusy(I2C0_BASE));

    //
    // Read I2C error status
    //
    status = I2CMasterErr(I2C0_BASE);

    //
    // Check if communication succeeded
    //
    if(status == I2C_MASTER_ERR_NONE)
    {
        //
        // Transmission successful
        //
        UARTprintf("WRITE OK\n");
    }
    else
    {
        //
        // Transmission failed
        //
        // Prints numeric error code
        //
        UARTprintf("I2C ERROR: %u\n", status);
    }
}

//*****************************************************************************
//
// Function: PCA9685_SetPWM
//
// Description:
// Configures PWM timing values for one PCA9685 output channel.
//
// Parameters:
// channel - PWM output channel (0 to 15)
// on      - Counter value where signal becomes HIGH
// off     - Counter value where signal becomes LOW
//
// The PCA9685 uses a 12-bit counter:
//
// 0    -> beginning of PWM period
// 4095 -> end of PWM period
//
// Example:
//
// PCA9685_SetPWM(1, 0, 2048);
//
// Channel 1:
// Starts HIGH at count 0
// Goes LOW at count 2048
//
// Result:
// ~50% duty cycle
//
// Register Mapping:
//
// LED0_ON_L   = 0x06
// LED0_ON_H   = 0x07
// LED0_OFF_L  = 0x08
// LED0_OFF_H  = 0x09
//
// Each channel occupies 4 consecutive registers.
//
//*****************************************************************************
void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off){
    uint8_t reg;

    //
    // Each channel uses 4 registers
    //
    reg = 0x06 + (4 * channel);

    //
    // ON low byte
    //
    PCA9685_Write(reg, on & 0xFF);

    //
    // ON high byte
    //
    PCA9685_Write(reg + 1, on >> 8);

    //
    // OFF low byte
    //
    PCA9685_Write(reg + 2, off & 0xFF);

    //
    // OFF high byte
    //
    PCA9685_Write(reg + 3, off >> 8);
}
//*****************************************************************************
//
// Function: PCA9685_SetPWMFreq
//
// Description:
// Sets the global PWM frequency of the PCA9685.
//
// Parameters:
// freq - Desired PWM frequency in Hz
//
// Example:
// PCA9685_SetPWMFreq(50);
//
// Sets PWM frequency to 50Hz for servos.
//
//*****************************************************************************
void PCA9685_SetPWMFreq(uint16_t freq){
    uint8_t prescale;

    //
    // Calculate prescale value
    //
    prescale = (25000000 / (4096 * freq)) - 1;

    //
    // Enter sleep mode
    //
    PCA9685_Write(0x00, 0x10);

    //
    // Write prescaler value
    //
    PCA9685_Write(0xFE, prescale);

    //
    // Wake up PCA9685
    //
    PCA9685_Write(0x00, 0x00);

    //
    // Wait for oscillator stabilization
    //
    SysCtlDelay(SysCtlClockGet() / 100);

    //
    // Restart
    //
    PCA9685_Write(0x00, 0x80);
}
//*****************************************************************************
// Function: ConfigureUART
//
// Configures UART0 for 115200 baud, 8N1
//*****************************************************************************
void ConfigureUART(void){
    //Enable UART0 and GPIO Port A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure UART0 for 115200 baud
    UARTClockSourceSet( UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0,115200,16000000);
}

int main(void) 
{
  
    SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ);
    ConfigureUART();
    I2C0_Init();
    PCA9685_Init();
    PCA9685_SetPWMFreq(200);

    while(1)
    {
        PCA9685_SetPWM(1,0,2048);  
    }
}
