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
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pwm.h"

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif


#define TURN_RIGHT  GPIO_PIN_6
#define TURN_LEFT  GPIO_PIN_7
#define TURN_RIGHT_A GPIO_PIN_6
#define TURN_LEFT_A GPIO_PIN_4

//*****************************************************************************
// Global variables
//*****************************************************************************
//*****************************************************************************
uint32_t g_ui32SysClock = 0;
uint32_t g_ui32PWMFrequency = 0;
uint32_t g_ui32PWMDutyCycle = 0;
// SBUS GLOBAL VARIABLES ******************************************************
uint16_t channels[16]; 
uint8_t sbusFrame[25];
uint8_t sbusIndex = 0;
//*****************************************************************************
// Function: SetPWM
//
// Configures PWM0 on PB6 (M0PWM0)
// sets frequency and duty cycle 
//*****************************************************************************
void SetPWM(uint32_t ui32Frequency, uint32_t ui32DutyCycle){
    uint32_t ui32Period;
    uint32_t ui32PulseWidth;

    //
    // Protect against invalid values
    //
    if(ui32Frequency == 0)
    {
        ui32Frequency = 1;
    }

    if(ui32DutyCycle > 100)
    {
        ui32DutyCycle = 100;
    }

    //
    // Calculate PWM period
    // Divide by 64, since the PWM gen uses a 64 div for sysclk
    ui32Period = (g_ui32SysClock/64) / ui32Frequency;

    //
    // Calculate pulse width
    //
    ui32PulseWidth = (ui32Period * ui32DutyCycle) / 100;

    //
    // Apply PWM settings
    //
    PWMGenPeriodSet(PWM0_BASE,
                    PWM_GEN_0,
                    ui32Period);

    PWMPulseWidthSet(PWM0_BASE,
                     PWM_OUT_0,
                     ui32PulseWidth);

    //
    // Save values
    //
    g_ui32PWMFrequency = ui32Frequency;
    g_ui32PWMDutyCycle = ui32DutyCycle;
}
//*****************************************************************************
// Function: ConfigurePWM
//
// Configures PWM0 on PB6 (M0PWM0)
// Initial: 1 kHz frequency, 50% duty cycle
//*****************************************************************************
void ConfigurePWM(void){
    //
    // PWM clock = system clock / 64 for greater precision 
    //
    MAP_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    //
    // Enable peripherals
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    //
    // PB6 -> M0PWM0
    //
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    //
    // Configure PWM generator
    //
    PWMGenConfigure(PWM0_BASE,
                    PWM_GEN_0,
                    PWM_GEN_MODE_DOWN);

    //
    // Initial PWM configuration
    //
    // SetPWM(250, 25); 

    //
    // Enable PWM output
    //
    PWMOutputState(PWM0_BASE,
                   PWM_OUT_0_BIT,
                   true);

    //
    // Start PWM generator
    //
    PWMGenEnable(PWM0_BASE,
                 PWM_GEN_0);
}
//*****************************************************************************
// Function: Configure_OUTPUT_PINS
//
// Configs output pins for high or low value 
//*****************************************************************************
//*****************************************************************************
// Function: Configure_OUTPUT_PINS
//
// Configs output pins for high or low value 
//*****************************************************************************
void Configure_OUTPUT_PINS(void){
    //
    // Enable the GPIO port for C, GPIO D, F 
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //
    // Enable the GPIO pins (PC6,PC7,PD6, PF4) as an output.
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, TURN_RIGHT); //PC6 TURN_RIGHT
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, TURN_LEFT); //PC7 TURN_LEFT
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, TURN_RIGHT_A); //PD6 TURN_RIGHT_A
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, TURN_LEFT_A); //PF4 TURN_LEFT_A
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3); //LED
}
//*****************************************************************************
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
        //UARTprintf("WRITE OK\n");
    }
    else
    {
        //
        // Transmission failed
        //
        // Prints numeric error code
        //
        //UARTprintf("I2C ERROR: %u\n", status);
    }
}
//*****************************************************************************
//
// Function: PCA9685_Init
//
// Description:
// Initializes the PCA9685 PWM controller.
//
// This function places the PCA9685 into normal operating mode
// by configuring the MODE1 register.
//
// After initialization, the internal oscillator starts running
// and PWM generation becomes available.
//*****************************************************************************
void PCA9685_Init(void){
    //
    // MODE1 register = normal mode
    //
    PCA9685_Write(0x00, 0x00);

    SysCtlDelay(SysCtlClockGet() / 100);
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
    // @50Hz 310 -> Center 
    // @50Hz 410 -> 0°
    // @50Hz 205 -> 90°
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
    PCA9685_Write(reg + 1, on >> 8); // shift 8 bits 

    //
    // OFF low byte
    //
    PCA9685_Write(reg + 2, off & 0xFF);

    //
    // OFF high byte
    //
    PCA9685_Write(reg + 3, off >> 8); // shift 8 bits
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
// ARDUINO LOGIC...
int32_t MapValue(int32_t x,int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
void channel10(uint16_t value);
void channel4(uint16_t value);
void channel14(uint16_t value);
//*****************************************************************************
//
// Function: ConfigureSBUSUART
//
// Description:
// Configures UART1 for SBUS communication on the TM4C123G.
//
// - Baud rate: 100000
// - Data bits: 8
// - Parity: Even
// - Stop bits: 2
//
// UART1 is mapped to:
//
// PB0 -> U1RX  (SBUS input)
// PB1 -> U1TX  (optional transmit)
//
// Note:
// Standard SBUS signals are inverted.
// An external inverter circuit or inverter-capable UART hardware
// may be required depending on the receiver used.
//
//*****************************************************************************
void ConfigureSBUSUART(void){
    //
    // Enable peripherals
    // UART1 and GPIO Port B
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Wait until peripherals are ready
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    //
    // Configure UART pins
    //
    // PB0 -> U1RX
    // PB1 -> U1TX
    //
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);

    GPIOPinTypeUART(
        GPIO_PORTB_BASE,
        GPIO_PIN_0 | GPIO_PIN_1
    );

    //
    // Configure UART1 for SBUS:
    //
    // Baud Rate : 100000
    // Data Bits : 8
    // Parity    : Even
    // Stop Bits : 2
    //
    UARTConfigSetExpClk(
        UART1_BASE,
        SysCtlClockGet(),
        100000,
        UART_CONFIG_WLEN_8 |
        UART_CONFIG_STOP_TWO|
        UART_CONFIG_PAR_EVEN
    );
}
void DecodeSBUS(void){
    channels[0]  = ((sbusFrame[1]      | sbusFrame[2]  << 8) & 0x07FF);
    channels[1]  = ((sbusFrame[2] >> 3 | sbusFrame[3]  << 5) & 0x07FF);
    channels[2]  = ((sbusFrame[3] >> 6 | sbusFrame[4]  << 2 |
                    sbusFrame[5] << 10) & 0x07FF);

    channels[3]  = ((sbusFrame[5] >> 1 | sbusFrame[6]  << 7) & 0x07FF);

    channels[4]  = ((sbusFrame[6] >> 4 | sbusFrame[7]  << 4) & 0x07FF);

    channels[5]  = ((sbusFrame[7] >> 7 | sbusFrame[8]  << 1 |
                    sbusFrame[9] << 9) & 0x07FF);

    channels[6]  = ((sbusFrame[9] >> 2 | sbusFrame[10] << 6) & 0x07FF);

    channels[7]  = ((sbusFrame[10] >> 5 | sbusFrame[11] << 3) & 0x07FF);

    channels[8]  = ((sbusFrame[12]     | sbusFrame[13] << 8) & 0x07FF);

    channels[9]  = ((sbusFrame[13] >> 3 | sbusFrame[14] << 5) & 0x07FF);

    channels[10] = ((sbusFrame[14] >> 6 | sbusFrame[15] << 2 |
                    sbusFrame[16] << 10) & 0x07FF);

    channels[11] = ((sbusFrame[16] >> 1 | sbusFrame[17] << 7) & 0x07FF);

    channels[12] = ((sbusFrame[17] >> 4 | sbusFrame[18] << 4) & 0x07FF);

    channels[13] = ((sbusFrame[18] >> 7 | sbusFrame[19] << 1 |
                    sbusFrame[20] << 9) & 0x07FF);

    channels[14] = ((sbusFrame[20] >> 2 | sbusFrame[21] << 6) & 0x07FF);

    channels[15] = ((sbusFrame[21] >> 5 | sbusFrame[22] << 3) & 0x07FF);
}
void SBUSFRAMES(void){
    uint8_t cData;
    int i;

    while(UARTCharsAvail(UART1_BASE))
    {
        cData = UARTCharGet(UART1_BASE);

        //
        // WAIT FOR START BYTE
        //
        if(sbusIndex == 0)
        {
            if(cData == 0x0F)
            {
                sbusFrame[sbusIndex++] = cData;
            }
        }
        else
        {
            //
            // RESYNC:
            // New frame start appeared too early
            //
            if(cData == 0x0F && sbusIndex < 24)
            {
                sbusIndex = 0;
                sbusFrame[sbusIndex++] = cData;
                continue;
            }

            //
            // STORE BYTE
            //
            sbusFrame[sbusIndex++] = cData;

            //
            // FRAME COMPLETE
            //
            if(sbusIndex == 25)
            {
                //
                // VALIDATE FRAME
                //
                if(sbusFrame[0] == 0x0F &&
                  (sbusFrame[24] == 0x00 ||
                   sbusFrame[24] == 0x04 ||
                   sbusFrame[24] == 0x14 ||
                   sbusFrame[24] == 0x24))
                {
                    //UARTprintf("\nVALID FRAME\n");
                    /*PRINT EACH BIT FROM THE FRAME*/
                   /* for(i = 0; i < 25; i++)
                    {
                        UARTprintf("0x%02X ", sbusFrame[i]);
                    } */

                    UARTprintf("\n");

                    //
                    // DECODE HERE
                        //
    // DECODE CHANNELS
    //
    DecodeSBUS();

    UARTprintf("CH08: %4d  ", channels[7]); //switch SE
    UARTprintf("CH09: %4d  ", channels[8]); //switch SE
    UARTprintf("CH014: %4d  ", channels[13]); // KNOB LD1
    UARTprintf("CH05: %4d  ", channels[4]);     // KNOB RD1
    // CONTROL FUNCTIONS
    //
    
    channel10(channels[8]);   // CH10
    channel4(channels[4]);    // CH4
    channel14(channels[13]);  // CH14
    
                    //
                }
                else
                {
                    UARTprintf("\nINVALID FRAME\n");
                }

                //
                // RESET PARSER
                //
                sbusIndex = 0;
            }
        }
    }
}
void DelayMs(uint32_t ms){
    SysCtlDelay((g_ui32SysClock / 3 / 1000) * ms);
}
int main(void) 
{
    //CLOCK CONFIG
    SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ);
    g_ui32SysClock = SysCtlClockGet();
    ConfigurePWM();
    ConfigureUART();
    ConfigureSBUSUART();
    Configure_OUTPUT_PINS();
    I2C0_Init();
    PCA9685_Init();
    PCA9685_SetPWMFreq(50);
    UARTprintf("OUTPUT PINS... \n");
    UARTprintf("IDK \n");
    
    //SetPWM(50,25);
    while(1)
    {
     SBUSFRAMES();

/*///////////////////////////////////////////////////////
//  PASS        
    //
    // TEST 1
    // Neutral position
    //
    UARTprintf("\nTEST: 992\n");
    channel10(992);
    channel4(992);
    channel14(992);
 
 // PASS
    //
    // TEST 2
    // Right position
    //
    UARTprintf("\nTEST: 1712\n");

    channel10(1712);
    channel4(1712);
    channel14(1712);
 //   SysCtlDelay(g_ui32SysClock / 3);
//PASS
    //
    // TEST 3
    // Left position
    //
    UARTprintf("\nTEST: 272\n");

    channel10(272);
    channel4(272);
    channel14(272);

*/
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3,GPIO_PIN_3 ); // HIGH LED
 
    }
}
// ARDUINO LOGIC...
int32_t MapValue(int32_t x,int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
    int32_t result;
    result = (x - in_min) *
           (out_max - out_min) /
           (in_max - in_min) +
           out_min;
    return result;
}
void channel10(uint16_t value){
    if(value == 992) 
    {    
        MAP_GPIOPinWrite(GPIO_PORTC_BASE, TURN_LEFT,0x00); // LOW PC7
        
        MAP_GPIOPinWrite(GPIO_PORTC_BASE, TURN_RIGHT,0x00); // LOW PC6 
    }
    else if( value == 1712)
    {
        MAP_GPIOPinWrite(GPIO_PORTC_BASE, TURN_LEFT,0x00); // LOW PC7 

        MAP_GPIOPinWrite(GPIO_PORTC_BASE, TURN_RIGHT,TURN_RIGHT); // HIGH PC6
    }
    else if( value == 272)
    {
        MAP_GPIOPinWrite(GPIO_PORTC_BASE, TURN_LEFT,TURN_LEFT); // HIGH PC7 

        MAP_GPIOPinWrite(GPIO_PORTC_BASE, TURN_RIGHT,0x00); // LOW PC6       
    }    
    else
    {
        MAP_GPIOPinWrite(GPIO_PORTC_BASE, TURN_LEFT,0x00); // LOW PC7 

        MAP_GPIOPinWrite(GPIO_PORTC_BASE, TURN_RIGHT,0x00); // LOW PC6
    }
}
void channel4(uint16_t value){
    if(value == 992)
    {
        PCA9685_SetPWM(1,0,2048); //duty cycle 50%

        MAP_GPIOPinWrite(GPIO_PORTF_BASE, TURN_LEFT_A,0x00); // LOW PF4

        MAP_GPIOPinWrite(GPIO_PORTD_BASE, TURN_RIGHT_A,TURN_RIGHT_A); // HIGH PD6
    }
    else if (value == 1712)
    {
        PCA9685_SetPWM(1,0,4095); // duty cycle 100%

        MAP_GPIOPinWrite(GPIO_PORTF_BASE, TURN_LEFT_A,0x00); // LOW PF4

        MAP_GPIOPinWrite(GPIO_PORTD_BASE, TURN_RIGHT_A,TURN_RIGHT_A); // HIGH PD6

    }
    else if (value == 272)
    {
        PCA9685_SetPWM(1,0,0); // duty cycle 0%

        MAP_GPIOPinWrite(GPIO_PORTF_BASE, TURN_LEFT_A,0x00); // LOW PF4

        MAP_GPIOPinWrite(GPIO_PORTD_BASE, TURN_RIGHT_A,0x00); // LOW PD6
    }
    else 
    {
        PCA9685_SetPWM(1,0,0); // duty cycle 0%
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, TURN_LEFT_A,0x00); // LOW PF4

        MAP_GPIOPinWrite(GPIO_PORTD_BASE, TURN_RIGHT_A,0x00); // LOW PD6
    }
}
void channel14(uint16_t value){
    int32_t angle;
    int32_t pulseWidthUs;
    int32_t dutyCycle;
    if(value >= 272 && value <= 1712)
    {
        // Convert SBUS -> angle
        angle = MapValue(value,272,1712,0,180);
        // Convert Angle -> pulse width
        // Around 1000us to 2000us
        pulseWidthUs = MapValue(angle,0,180,1000,2000);
        //Convert pulse width -> duty cycle
        dutyCycle = (pulseWidthUs * 100) / 20000;
        //Apply PWM

        SetPWM(50,dutyCycle);
        //UARTprintf("Angle: %d Duty: %d%% \n", angle,dutyCycle);
       
    }
}