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
#ifndef DEFS_H
#define DEFS_H

#include "pi.h"
#include "periph.h"
#include <xc.h>
#include <stdint.h>

/* CURIOSITY development board can be selected based on the requirement.
 * At any time, only one should be defined.
 * Based on the selection the recommended configurations will be enabled */

#define CURIOS_DEV      1
#define POT 11          // AN channel for pot 

#define FCY 24000000    //Internal 8MHz clock x 4
#define FPWM 25000		//25,000 Hz PWM

// ####### SPEED CONTROLLER. Choose only one of the two #######################

#if 1
#define OPEN_LOOP_CONTROL			//Openloop Control
#else
#define PI_CLOSED_LOOP_CONTROL		//Closedloop Control
#endif

#define PHASE_ADVANCE_DEGREES	0		//degrees for phase advancing

//########### Motor Control Definitions #################
#if defined CURIOS_DEV  
    #define STARTUP_DUTY            240     //sets the starting motor speed in forced commutation mode; Fsys & Fpwm dependent
    #define STARTUP_RPM             1000	//final RPM after startup. this becomes the minimum RPM#define MIN_RPM 
    #define MIN_RPM                 750     // motor RPM at MIN_MOTOR_SPEED_REF
    #define MAX_RPM                 2400    // motor RPM at MAX_MOTOR_SPEED_REF
    #define POLEPAIRS               6       // Number of pole pairs of the motor

    #define RAMPDELAY_START         40      //in ms; the starting sector comutation period
    #define RAMPDELAY_MIN           4       //in ms; minimum period for startup ramp; when reaching this value, it will start looking for BEMF

    #define BLANKING_COUNT          2       // Blanking count expressed in PWM periods used to avoid false zero-crossing detection after commutating motor
    #define BEMF_STALL_LIMIT        5000     // If no BEMF signal is detected for (BEMF_STALL_LIMIT*BLANKING_COUNT * 50us) then it is assumed the rotor is stalled

    #define MAX_MOTOR_SPEED_REF     2000    // corresponds to MAX_RPM
    #define MIN_MOTOR_SPEED_REF     400     // decrease or increase this value to set the minimum motor speed
                                            // The minimum motor speed in closed loop is MAX_RPM*MIN_MOTOR_SPEED_REF/MAX_MOTOR_SPEED_REF

    #define RPM_PWM_FACTOR (uint16_t)(32768 * ((float)MAX_MOTOR_SPEED_REF / (float)MAX_RPM))	//PWM Duty cycle = RPM_PWM_FACTOR * Speed_in_RPM

    #define BEMF_VDDMAX             400          
    /*	on CURIOS_DEV:
        R10/(R10+R14) * DC Voltage / 3.3 V * 1024
        2K/32K * 24 / 3.3 * 1024 = 465 
    */
#endif

/* Switch configuration based on the board selected */
#if defined CURIOS_DEV
    #define S1	!PORTBbits.RB7	    //S1 button
    #define S2	!PORTBbits.RB13	    //S2 button (direction))
#endif
    #define LED2 PortCbits.RC9      // LED #2

/* ####################  PI loop constants ############################## */
#define PI_P_TERM	2000
#define PI_I_TERM	200
#define PI_ANTI_WINDUP  0x7FFF

#define PWM_1000us_FACTOR            (FPWM/1000)

/*******************  Derived Definitions  - Do not change*******************/
#define PI_TICKS        80                        // Speed Controller frequency ->  80 ADC periods
#define MAX_DUTY_CYCLE  (int32_t)((FCY/FPWM)-1)   // 100% duty cycle

/* SCCP3 Timer measures the motor speed by measuring the time the rotor takes 
 * to make a 60 degree electrical rotation angle.
 * SCCP3 Timer minimum value is: 1/(MAX_RPM/60)/POLEPAIRS*FCY/SCCP3Prescaler/(360/60) */

#define SCCP3_MIN       (int64_t)60/MAX_RPM/POLEPAIRS*FCY/64/6
#define SCCP3_MAX       (uint16_t)(SPEEDMULT/MIN_MOTOR_SPEED_REF)

// CONVERSION SPEED FACTOR - SPEEDMULT
#define SPEEDMULT       (int32_t)(MAX_MOTOR_SPEED_REF * SCCP3_MIN)  //Factor used to scale the Desired speed to the actual motor speed

//###################### Flags, State Machine, etc #############################

//application flags and state machine
enum {
   STATE_STOPPED,   //motor is stopped
   STATE_STOPPING,  //event to tell the motor to stop
   STATE_STARTING,  //event to tell the motor to start, and startup sequence
   STATE_STARTED,   //motor is running
   STATE_FAULT      //motor fault
} states_t;

 typedef struct
{
    unsigned RunMotor : 1;
    unsigned Startup : 1;
    unsigned CLKW : 1;
    unsigned newCLKW : 1;
    unsigned PreCommutationState : 1;
    unsigned PotRead : 1;
    unsigned DMCI_Control_SW : 1;
    unsigned current_state : 3;
    unsigned unused : 1;
} TFlags;

TFlags volatile Flags;

/* ## Clockwise and Counter-Clockwise rotation constants, and
      majority detection filter, etc                          */

/* override values for each sector */
extern const uint32_t PWM_STATE_CLKW[6];
extern uint32_t PWM_STATE[6];

/*ADC Channel AN Select*/
extern const uint16_t ADC_CHANNEL_CLKW[6];
extern uint16_t ADC_CHANNEL[6];

/*Motor Phases*/
extern const uint32_t MotorPhaseAState_CLKW[6];
extern const uint32_t MotorPhaseBState_CLKW[6];
extern const uint32_t MotorPhaseCState_CLKW[6];
extern uint32_t MotorPhaseAState[6];
extern uint32_t MotorPhaseBState[6];
extern uint32_t MotorPhaseCState[6];

/*AND & OR operators for masking the active BEMF signal*/
extern const uint16_t ADC_MASK_CLKW[6];
extern uint16_t ADC_MASK[6];

extern const uint16_t ADC_XOR_CLKW[6];
extern uint16_t ADC_XOR[6];

/*BEMF Majority Function Filter values*/
extern const uint8_t ADC_BEMF_FILTER_CLKW[64];
extern uint8_t ADC_BEMF_FILTER[64];

//###################### Miscellaneous Variables and Defines ####################
extern uint8_t ADCCommState;            // state for current motor sector
extern uint8_t adcBackEMFFilter;        // stores value for each ADC filtering
extern uint16_t Phase_Advance_Degrees;	// stores value for Phase Advance degrees. Modify it through DMCI
extern uint16_t PhaseAdvanceTicks;  	// counter for ticks to be extracted out of Timer2 when commuting to add Phase Advance
extern uint8_t BlankingCounter;         // blanking counter, for rejecting some values out of the filter
extern uint32_t PIticks;                // counter for skipping PI calculation
extern volatile uint32_t stallCount;             // counter for stalling
extern uint32_t RampDelay;              // variable used to create the startup ramp delay
extern volatile int32_t delay_counter;	// used for delays. is incremented automaticalLy in MCPWM interrupt, each 50 us
extern uint32_t MotorNeutralVoltage;	// Motor Neutral Voltage calculAtion
extern uint32_t MotorPhaseA;			// Motor Phase A current voltage
extern uint32_t MotorPhaseB;			// Motor Phase B current voltage
extern uint32_t MotorPhaseC;			// Motor Phase C current voltage
extern uint16_t ComparatorOutputs;

extern uint32_t SCCP3Value;             //Used for calculating next commutation occurring
extern uint32_t SCCP3Average;			//Used for calculating next commutation occurring
extern uint32_t SCCP2Value;

extern uint32_t CurrentSpeed, DesiredSpeed, DesiredRPM, CurrentDuty, DesiredDuty; 	//speed definitions

//PI Controller definitions
extern uint32_t SpeedControl_P;         // The P term for the PI speed control loop. Modify in defs.c
extern uint32_t SpeedControl_I;         // The I term for the PI speed control loop. Modify in defs.c
extern tPIParm PIDStructureVel;         // PID Structure inner velocity loop
extern tPIParm PIDStructurePha;         // PID Structure outer phase loop

void SpeedPILoopController(void);		// PI Loop Controller
void OpenLoopController(void);			// Open Loop Controller

//other functions
void DelayNmSec(uint32_t N);	//delays N milLi seconds. MCCP Timer Based blocking delay.

//motor control functions. defined here, implemented in main.c
void Init_Motor(void);		//just initialize motor and PI
void Start_Motor(void);		//sequence to attempt to force start of the motor
void Stop_Motor(void);		//stops motor

#endif
