/* *********************************************************************
 * (c) 2017 Microchip Technology Inc. and its subsidiaries. You may use
 * this software and any derivatives exclusively with Microchip products.
 *
 * This software and any accompanying information is for suggestion only.
 * It does not modify Microchips standard warranty for its products.
 * You agree that you are solely responsible for testing the software and
 * determining its suitability.  Microchip has no obligation to modify,
 * test, certify, or support the software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH
 * MICROCHIP PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY
 * APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL,
 * PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF
 * ANY KIND WHATSOEVER RELATED TO THE USE OF THIS SOFTWARE, THE
 * motorBench(TM) DEVELOPMENT SUITE TOOL, PARAMETERS AND GENERATED CODE,
 * HOWEVER CAUSED, BY END USERS, WHETHER MICROCHIP'S CUSTOMERS OR
 * CUSTOMER'S CUSTOMERS, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
 * CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
 * OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
 * SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
 * THESE TERMS.
 *
* *****************************************************************************/
#include "defs.h"

//PI Controller
uint32_t SpeedControl_P = PI_P_TERM; // The P term for the PI speed control loop
uint32_t SpeedControl_I = PI_I_TERM; // The I term for the PI speed control loop

#ifdef CLASSIC
const uint32_t PWM_STATE_CLKW[6] =	
            {0x20000000,0x20000000,0x08000000,0x08000000,0x02000000,0x02000000};  //
const uint8_t LOW_SIDE_CLKW[6] =
            {0x02,0x01,0x01,0x04,0x04,0x02};
uint8_t LOW_SIDE[6];
#else
const uint32_t PWM_STATE_CLKW[6] =	// 1/15/20 verified does step CLKW
            {0x24000000,0x21000000,0x09000000,0x18000000,0x12000000,0x06000000};  // okay
#endif
uint32_t PWM_STATE[6];

// to equate with schematic equate {U,V,W} == {A,B,C}
// 0==Gnd, 1==open, BEMF_VDDMAX==driven
const uint32_t MotorPhaseAState_CLKW[6] = {BEMF_VDDMAX, BEMF_VDDMAX, 1, 0, 0, 1}; // U, check
const uint32_t MotorPhaseBState_CLKW[6] = {0, 1, BEMF_VDDMAX, BEMF_VDDMAX, 1, 0}; // V, check
const uint32_t MotorPhaseCState_CLKW[6] = {1, 0, 0, 1, BEMF_VDDMAX, BEMF_VDDMAX}; // W, check

uint32_t MotorPhaseAState[6];
uint32_t MotorPhaseBState[6];
uint32_t MotorPhaseCState[6];

// define ADC channels #'s used for reading U, V & W as well as neutral
const uint16_t ADC_CHANNEL_CLKW[6] = {0x0004,0x0003,0x0002,0x0004,0x0003,0x0002}; // okay
uint16_t ADC_CHANNEL[6];

const uint16_t ADC_MASK_CLKW[6]	=   {0x0004,0x0002,0x0001,0x0004,0x0002,0x0001};  // check
uint16_t ADC_MASK[6];

const uint16_t ADC_XOR_CLKW[6]	=   {0x0000,0xFFFF,0x0000,0xFFFF,0x0000,0xFFFF};  // check 1/24
uint16_t ADC_XOR[6];

const uint8_t ADC_BEMF_FILTER_CLKW[64]=                                           // check
    {0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x1C,0x1E,
     0x20,0x22,0x24,0x26,0x28,0x2A,0x2C,0x2E,0x01,0x01,0x01,0x36,0x01,0x3A,0x3C,0x3E,
     0x00,0x02,0x04,0x06,0x08,0x0A,0x0C,0x0E,0x01,0x01,0x01,0x16,0x01,0x1A,0x1C,0x1E,
     0x01,0x01,0x01,0x26,0x01,0x2A,0x2C,0x2E,0x01,0x01,0x01,0x36,0x01,0x3A,0x3C,0x3E};

uint8_t ADC_BEMF_FILTER[64];

uint8_t bemf_filter[64];
uint8_t bemf_pntr;

uint16_t adcA_buffer[256];
uint16_t adc_pntr;
bool Fg;

uint8_t ADCCommState;
uint8_t adcBackEMFFilter;
uint16_t Phase_Advance_Degrees = PHASE_ADVANCE_DEGREES;
uint16_t PhaseAdvanceTicks;
uint8_t BlankingCounter;
volatile uint32_t stallCount;
uint32_t RampDelay;
volatile uint8_t openLoopSteps;
uint32_t PIticks;

uint32_t MotorNeutralVoltage;
uint32_t MotorPhaseA;
uint32_t MotorPhaseB;
uint32_t MotorPhaseC;
uint16_t ComparatorOutputs;

uint32_t SCCP2Value;
uint32_t SCCP3Value;
uint32_t SCCP3Average;

volatile int32_t delay_counter;

uint32_t CurrentSpeed, DesiredSpeed, DesiredRPM, CurrentDuty, DesiredDuty; 	//speed definitions

tPIParm PIDStructureVel;
tPIParm PIDStructurePha;

/***************************************************
 SpeedPILoopController
 
 Closed loop PI Controller routine - calls CalcPI
 and returns the output to PWM's duty cycle
***************************************************/
void SpeedPILoopController(void)
{
    // Original Method: Normalizing SCCP3 Timer counts to electrical RPS expressed in PWM counts
    //                  SCCP3 Timer Counts are converted to PWM counts
    //                  and then multiplied by the number of sector
    //                  required to complete 1 electrical RPS
    // Modified method: Period of Fg is captured using Counter0
    //
    //
    CurrentSpeed = (uint16_t)(SPEEDMULT/SCCP3Average);     
    PIDStructureVel.InRef = DesiredSpeed;
    PIDStructureVel.InMeas = CurrentSpeed;

    CalcPI(&PIDStructureVel);

    CurrentSpeed =  PIDStructureVel.Out;   //set PID output

    //Assigning new duty cycle to the PWM channels
    CCP1RBbits.CMPB = CurrentSpeed;
}

/***************************************************
 OpenLoopController
 
 Open loop controller routine
 Increases or decreases the duty cycle according
 to a reference duty cycle
***************************************************/
void OpenLoopController(void)
{
    //Update the duty cycle according to the POT value, a POT follower is implemented here
    if(CurrentSpeed != DesiredSpeed)
    {
        if(CurrentSpeed < DesiredSpeed)
            CurrentSpeed += 1;
        if(CurrentSpeed > DesiredSpeed)
            CurrentSpeed -= 1;
    }
    // Min and Max PWM duty cycle limits
    if (CurrentSpeed < MIN_MOTOR_SPEED_REF)
            CurrentSpeed = MIN_MOTOR_SPEED_REF;
    if (CurrentSpeed > MAX_MOTOR_SPEED_REF)
            CurrentSpeed = MAX_MOTOR_SPEED_REF;

    //Assigning new duty cycles to the PWM channels
    CCP1RBbits.CMPB = CurrentSpeed;
} 

void DelayNmSec(uint32_t N)
{
    // busy-waiting delay using PWM ISR, count in milli-seconds
    // original code was configured as 100us by mistake (?).    
    while(N--)
    {
        delay_counter = 0;
        while(delay_counter < PWM_1000us_FACTOR);
    }
}

// for debug without isr's or timers, use core timer
// increments on every 2nd instruction
void DelayCycles(uint32_t n)
{
    uint32_t StartTime;
    StartTime = _CP0_GET_COUNT();
    while( (uint32_t) (_CP0_GET_COUNT() - StartTime) < n) {};
    
}

//end of definitions
