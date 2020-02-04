
 /* *********************************************************************
 * (c) 2017 Microchip Technology Inc. and its subsidiaries. You may use
 * this software and any derivatives exclusively with Microchip products.
 *
 * This software and any accompanying information is for suggestion only.
 * It does not modify Microchips standard warranty for its products.
 * 
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
 * 11/14/2019 Prysm Inc. Brian Tremaine
 * Code base AN1160_PIC32MM0064GPL_MCLV_MCLV taken as starting point for
 * Java project (Gen3).
 * Rebuild for the PIC32MM Curiosity Development Board. (CURIOS_DEV0
 *   S1: start/stop switch
 *   S2: not used
 * 
* ****************************************************************************/
#include "defs.h"		//defines, function headers, pi, etc
#include <xc.h>
#include <sys/attribs.h>

int32_t main(void)
{
    Init_Peripheral();	// Clock , Port Configuration
    Init_MCCP();    	// MCCP initialization
    Init_ADC();         // Analog-Digital Converter initialization
    Init_Timers();  	// Timer1, SCCP2, SCCP3 initialization
       
    //defaults: CLKW rotation, motor stopped
    Flags.RunMotor = 0;
    Flags.Startup = 0;
    Flags.CLKW = 0;                 // Java runs CCW
    Flags.newCLKW = 0;
    Flags.DMCI_Control_SW = 0;		//default potentiometer read
    Flags.current_state = STATE_STOPPED;
    DesiredRPM = STARTUP_RPM;
    
    debug_test(0); // if flag set in debug test will execute while() debug
    
    // for remote debug:
    //Flags.current_state = STATE_STARTING;
    Flags.current_state = STATE_STOPPING;
    
    while(1) {
        
        #if defined CURIOS_DEV
        if(S1)
		{ //start/stop switch
            while(S1) //debounce
                DelayNmSec(DEBOUNCE_DELAY);
            
            if(Flags.current_state == STATE_STOPPED)
                Flags.current_state = STATE_STARTING;
            else
                Flags.current_state = STATE_STOPPING;
        }
        #endif

        switch(Flags.current_state)
        {
            case STATE_STOPPED:     // 0
                // do nothing here
                break;
                
            case STATE_STOPPING:    // 1
                Stop_Motor();
                Flags.current_state = STATE_STOPPED;
                break;

            case STATE_STARTING:    // 2
                if(Flags.Startup == 0)
                    Init_Motor();
                else
                    Start_Motor();
                break;
            
            case STATE_STARTED:     // 3
                // gets changed in ADC ISR
                // original code checked direction command change
                break;

            case STATE_FAULT:       // 4
                Stop_Motor();
                DelayNmSec(600);   //delay for motor to actually stop
                Flags.current_state = STATE_STOPPED; // don't start automatically foe debug
                Flags.Startup == 0;   // added bpt 1/20/20
                break;
            
        } // end case
    
    }  // end while
    return 0;
}

/*******************************************************************
Init_Motor()
	Procedure used to initialize all params for the motor and
for the AN1160 algorithm.
	Also rotor alignment is done here.
*******************************************************************/
void Init_Motor()
{
    int32_t i;	//auxiliary counter

    T1CONbits.TON = 0;
    CCP2CON1bits.ON = 0;
    CCP3CON1bits.ON = 0;
    TMR1 = 0;
    CCP2TMRbits.TMRL = 0;
    CCP3TMRbits.TMRL = 0;
    
    //Setting direction CLKW or CCLKW ( <change at compile time only> )
    if (Flags.CLKW == 0)
    {
        for (i = 0; i < 6; i++)
        {
            PWM_STATE[i] = PWM_STATE_CLKW[5 - i];
#ifdef CLASSIC
            LOW_SIDE[i] = LOW_SIDE_CLKW[5-i];
#endif
            MotorPhaseAState[i] = MotorPhaseAState_CLKW[5 - i];
            MotorPhaseBState[i] = MotorPhaseBState_CLKW[5 - i];
            MotorPhaseCState[i] = MotorPhaseCState_CLKW[5 - i];
            ADC_CHANNEL[i] = ADC_CHANNEL_CLKW[5 - i];
            ADC_MASK[i] = ADC_MASK_CLKW[5 - i];
            ADC_XOR[i] = ADC_XOR_CLKW[5 - i];
        }
        for (i = 0; i < 64; i++)
            ADC_BEMF_FILTER[i] = ADC_BEMF_FILTER_CLKW[63 - i];
    }
    else
    {
        for (i = 0; i < 6; i++)
        {
           PWM_STATE[i] = PWM_STATE_CLKW[i];
#ifdef CLASSIC
           LOW_SIDE[i] = LOW_SIDE_CLKW[i];
#endif
           MotorPhaseAState[i] = MotorPhaseAState_CLKW[i];
           MotorPhaseBState[i] = MotorPhaseBState_CLKW[i];
           MotorPhaseCState[i] = MotorPhaseCState_CLKW[i];
           ADC_CHANNEL[i] = ADC_CHANNEL_CLKW[i];
           ADC_MASK[i] = ADC_MASK_CLKW[i];
           ADC_XOR[i] = ADC_XOR_CLKW[5-i];
        }
       for (i = 0; i < 64; i++)
           ADC_BEMF_FILTER[i] = ADC_BEMF_FILTER_CLKW[63-i];
    }

    stallCount = 0;
    openLoopSteps = 0;

    InitPI(&PIDStructureVel,SpeedControl_P,SpeedControl_I,PI_ANTI_WINDUP,MAX_MOTOR_SPEED_REF,MIN_MOTOR_SPEED_REF,MIN_MOTOR_SPEED_REF);

    ADCCommState = 5;	//Always start with sector 6 forced
    RampDelay = RAMPDELAY_START;	//startup initial delay. also the delay used to hold the rotor for the first sector
        
    CCP1CON2 = ((CCP1CON2 & PWM_MASK) | PWM_STATE[ADCCommState]); //set PWM overdrive according to the PWM channel                                     
#ifdef CLASSIC
    LATACLR =  (0b000000000101);                        // clear low-side ports
    LATBCLR = (0b0000100000000); 
    
           switch(LOW_SIDE[ADCCommState])               // set low-side ports 
        {
            case 0x1:
                LATBSET = (0b0000100000000);            // RB8
                break;
                
            case 0x2:
                LATASET = (0b000000000100);             // RA2
                break;

            case 0x4:
                LATASET = (0b000000000001);             // RA0
                break;                       
        } // end case
#endif
    
    CurrentSpeed = STARTUP_DUTY;   //Initialize PWM duty cycle value to minimum duty allowed for startup
    CCP1RBbits.CMPB = CurrentSpeed;
    
    Set_LED2();
    DelayNmSec(40);  // ?? what purpose is this delay???  
    Clr_LED2();
    
    SCCP3Average = SCCP3Value = CCP3TMRbits.TMRL = SCCP3_MAX;
    SCCP2Value = 0;
    Flags.RunMotor = 1;         // Turn the motor ON
    Flags.Startup = 1;          // Motor initialized, go to starting sequence
}

/**********************************************************************
Start_Motor()
	Procedure for starting the motor according to the implemented
startup ramp. After the ramp, PI loop training will begin.
**********************************************************************/
void Start_Motor()
{
    if(++ADCCommState >5)	// Change The Six-Step Commutation Sector
        ADCCommState = 0;
    //Change ADC Channel AN Selection
    AD1CHSbits.CH0SA = ADC_CHANNEL[ADCCommState];	
    
    // non-linear RAMP for startup.
    // ===============================================================
    if(RampDelay > 10)
        RampDelay -= 1; // was 10
    else
	//if(RampDelay > 3)
    //        RampDelay -= 2;

    if(RampDelay <= RAMPDELAY_MIN)
    {
        //if RampDelay is smaller than RAMPDELAY_MIN then we'll switch on BEMF detection.
        //BEMF Detection is started by starting Timer1, which will trigger ADC
        RampDelay = RAMPDELAY_MIN;
        CurrentSpeed = MIN_MOTOR_SPEED_REF;		//this is mainly for OpenLoopController, to start from a lower value
        
        T1CONbits.TON == 0;   // ADC ISR off
    }
    
    if(++openLoopSteps > MIN_OPEN_LOOP_STEPS)
        openLoopSteps = MIN_OPEN_LOOP_STEPS;       
    
    CCP1CON2 = ((CCP1CON2 & PWM_MASK) | PWM_STATE[ADCCommState]);	//overdrive and output next motor sector
#ifdef CLASSIC
    LATACLR =  (0b000000000101);                         // clear low-side ports
    LATBCLR = (0b0000100000000); 
    
           switch(LOW_SIDE[ADCCommState])               // set low-side ports 
        {
            case 0x1:
                LATBSET = (0b0000100000000);            // RB8
                break;
                
            case 0x2:
                LATASET = (0b000000000100);             // RA2
                break;

            case 0x4:
                LATASET = (0b000000000001);             // RA0
                break;                       
        } // end case
#endif
    
    //Set_LED2();
    DelayNmSec(RampDelay);      //the delay for rotor to move
    //Clr_LED2();
    adcBackEMFFilter = 0;       //clear the BEMF filter
}

/*********************************************************************
Stop_Motor()
	Procedure for stopping the motor from running and resetting
critical variables.
**********************************************************************/
void Stop_Motor()
{
    CCP1CON2 = (CCP1CON2 & PWM_MASK) | 0x00000000;     // override PWM pins low
#ifdef CLASSIC
    LATACLR =  (0b000000000101);                       // clear low-side ports
    LATBCLR = (0b0000100000000);
#endif
    
    adcBackEMFFilter = 0;
    PIticks = 0;

    Flags.RunMotor = 0;         		// reset run flag
    Flags.Startup = 0;					// no startup sequence
    Flags.PreCommutationState = 0;		// clear pre-commutation state flag
    Flags.PotRead = 0;					// clear potentiometer read flag

    T1CONbits.TON = 0;      // Stop TIMER1
    CCP3CON1bits.ON = 0;    // Stop SCCP3 Timer
    CCP2CON1bits.ON = 0;	// Stop SCCP2 Timer
    
    TMR1 = 0;
    CCP2TMRbits.TMRL = 0;
    CCP3TMRbits.TMRL = 0;
}

/******************************************************************************
ADC Interrupt Service Routine()
Actual implementation of the BLDC sensor-less BEMF zero-crossing
detection algorithm. In this routine, the ADC will read the corresponding
phase, apply the majority detection filter, and find the zero-crossing point.
Here we also read the potentiometer and execute the control loop for
the algorithm ( PI or Open loop ).
******************************************************************************/
void __ISR(_ADC_VECTOR, IPL7SOFT) ADC1Interrupt(void) 
{  
    // trigger t.p. for ADC
    // Toggle_LED2(0);  
    
    // one sample stop timer 1
    T1CONbits.TON = 0;
       
    if (Flags.PreCommutationState == 0)
    {   // this code assumes neutral is predefined with GND==0 & BEMF_VDDMAX
        // not as accurate but saves processor time.
        MotorPhaseA = MotorPhaseAState[ADCCommState];
		MotorPhaseB = MotorPhaseBState[ADCCommState];
		MotorPhaseC = MotorPhaseCState[ADCCommState];
        
        // trigger t.p. for ADC
        //Toggle_LED2();   
        
        if(MotorPhaseA == 1) {
            MotorPhaseA = ADC1BUF0;
            adcA_buffer[adc_pntr++] = MotorPhaseA;
            if(adc_pntr>255) adc_pntr = 0;
            Toggle_LED3();
        }

        if(MotorPhaseB == 1)
            MotorPhaseB = ADC1BUF0;

        if(MotorPhaseC == 1)
            MotorPhaseC = ADC1BUF0;
         
		MotorNeutralVoltage = (MotorPhaseA + MotorPhaseB + MotorPhaseC) / 3;  
         
		/********************* ADC SAMPLING & BMEF signals comparison ****************/
        BlankingCounter++;
		if(BlankingCounter > BLANKING_COUNT) 
        {
            // Toggle_LED3();
            
			ComparatorOutputs = 0;						// Precondition all comparator bits as zeros
			if(MotorPhaseA > MotorNeutralVoltage)
				ComparatorOutputs += 1;					// Set bit 0 when Phase A is higher than Neutural
			if(MotorPhaseB > MotorNeutralVoltage)
				ComparatorOutputs += 2;					// Set bit 1 when Phase B is higher than Neutural
			if(MotorPhaseC > MotorNeutralVoltage)
				ComparatorOutputs += 4;					// Set bit 2 when Phase C is higher than Neutral

        // Masking the BEMF signals according to the SECTOR in order to determine the ACTIVE BEMF signal
        // XOR operator helps to determine the direction of the upcoming zero-crossing slope
            // ADC_XOR is 0x0000 or 0xFFFF; ADC_MASK selects bit 0, 1, or 2 that is active ADC 
            if((ComparatorOutputs^ADC_XOR[ADCCommState]) & ADC_MASK[ADCCommState])
                adcBackEMFFilter |= 0x01;

            bemf_filter[bemf_pntr++] = ComparatorOutputs; //adcBackEMFFilter;
            if(bemf_pntr>63) bemf_pntr = 0;
            
            adcBackEMFFilter = ADC_BEMF_FILTER[adcBackEMFFilter];	//Majority detection filter

            if (adcBackEMFFilter&0b00000001) {

				if(Flags.current_state == STATE_STARTING)
                {	//When a valid BEMF zero crossing event has been detected, disable the motor start-up sequence
					Flags.current_state = STATE_STARTED;
					Flags.Startup = 0;
				}

                stallCount = 0;					//clear the stall counter whenever the BEMF signal is detected
				Flags.PreCommutationState = 1;	//set pre-commutation state flag
                
                    if ( ADCCommState == 0) {
                        Fg = true;
                        Set_LED2();
                    }
                    else {
                        Fg = false;
                        Clr_LED2();
                    }

				// Calculate the time proportional to the 60 electrical degrees
    			CCP3CON1bits.ON = 0;  // Stop SCCP3 Timer 
    			SCCP3Average = ((SCCP3Average+ SCCP3Value + 2*CCP3TMRbits.TMRL)>>2);
    			SCCP3Value = CCP3TMRbits.TMRL;
    			CCP3TMRbits.TMRL = 0;
    			CCP3CON1bits.ON = 1;  // Start SCCP3 Timer

    			PhaseAdvanceTicks = ((SCCP3Average*Phase_Advance_Degrees)/60);	//Calculate the delay in TIMER1 counts proportional to the Phase Adv angle

    			// Calculate the time proportional to the 30 electrical degrees
    			// Load the SCCP2 with  the SCCP3 counts proportional to 30 deg minus the PHASE ADV angle delay
    			SCCP2Value = (((SCCP3Average)>>1)+PhaseAdvanceTicks);

    			if(SCCP2Value>1)
        			CCP2PRbits.PRL = SCCP2Value;
    			else
                    CCP2PRbits.PRL = SCCP2Value = 1;
            
    			CCP2CON1bits.ON = 1;	// Start SCCP2
            }
            else
                ++stallCount;	//if a BEMF zero crossing was not detected increment the stall counter
        }
        
        #if defined OPEN_LOOP_CONTROL
        //Call the open loop speed controller at a fixed frequency, which is (PI_TICKS*50us)
        if((++PIticks >= PI_TICKS) && (Flags.current_state == STATE_STARTED))
        {
            OpenLoopController();
            PIticks = 0;
        }
        #endif
	}
    else
    {   // PreCommutationState == 1
        #if defined PI_CLOSED_LOOP_CONTROL
        SpeedPILoopController();
        #endif
        if(Flags.DMCI_Control_SW == 0)
        {
            if(Flags.PotRead == 0)
            {
				AD1CHSbits.CH0SA = POT;		//Select Potentiometer on AN11
				Flags.PotRead = 1;          //Set potentiometer read flag
			}
            else
            {
                DesiredRPM = (unsigned int)(( (unsigned long)ADC1BUF0 * (MAX_RPM-MIN_RPM) ) >> 10 ) + MIN_RPM; // value in RPM
                DesiredSpeed = (unsigned int)(((unsigned long)DesiredRPM * RPM_PWM_FACTOR) >> 15);  // value of PWM
            }
		}
        else
			DesiredSpeed = (unsigned int)(((unsigned long)DesiredRPM * RPM_PWM_FACTOR) >> 15);  // value of PWM
    }
     
	AD1CON1bits.DONE = 0;
	IFS0bits.AD1IF = 0;
}


/**********************************************************************
MCCP Interrupt Service Routine()
	Occurs every PWM period, (verified 40us, 25kHz)
	Used to increment delay_counter ( for DelayNmSec )
	detects stalling
 * Note: same as PWM interrupt in dsPIC33 demo code
**********************************************************************/
void __attribute__ ((vector(_CCP1_VECTOR), interrupt(IPL3SOFT), micromips)) _CCP1Interrupt(void)
{
    //LATCINV = (0b1000000000);  // test: toggle LED2 in atomic operation  
    
    delay_counter++;
    if ((T1CONbits.TON == 0) && (openLoopSteps >= MIN_OPEN_LOOP_STEPS))     // should this be qualified with STATE_STARTED ?????
        T1CONbits.TON = 1;       // should this even be here or in Start_Motor instead
    //rotor stall detection
    if ((stallCount > BEMF_STALL_LIMIT) && (Flags.current_state == STATE_STARTED))
    {
        Flags.current_state = STATE_FAULT;      //go to FAULT state and restart the motor without pushing the button
        stallCount = 0;     //clear the stall counter
    }
    IFS0bits.CCP1IF = 0;	//clear MCCP Interrupt Flag
}


/**********************************************************************
SCCP2 Interrupt Service Routine()
	- used to switch (commute) the current driving sector of the motor
	- ends PreCommutationState
 * Note: same as Timer1 Interrupt in dsPIC33 demo code
**********************************************************************/
void __attribute__ ((vector(_CCT2_VECTOR), interrupt(IPL5SOFT), micromips)) _CCT2Interrupt(void)
{
    adcBackEMFFilter = 0;
    BlankingCounter = 0;
       
    bemf_filter[bemf_pntr++] = 0xff;
    if(bemf_pntr>63) bemf_pntr = 0;
      
    if (++ADCCommState > 5)
        ADCCommState = 0;
    
    AD1CHSbits.CH0SA = ADC_CHANNEL[ADCCommState];	//switch adc channel
    CCP1CON2 = ((CCP1CON2 & PWM_MASK) | PWM_STATE[ADCCommState]); //switch sector
#ifdef CLASSIC
    LATACLR =  (0b000000000101);                         // clear low-side ports
    LATBCLR = (0b0000100000000); 
    
        switch(LOW_SIDE[ADCCommState])                   // set low-side ports 
        {
            case 0x1:
                LATBSET = (0b0000100000000);            // RB8
                break;
                
            case 0x2:
                LATASET = (0b000000000100);             // RA2
                break;

            case 0x4:
                LATASET = (0b000000000001);             // RA0
                break;                       
        } // end case
#endif
    
    Flags.PreCommutationState = 0;  //clear pre-commutation state flag
    Flags.PotRead = 0;              //clear potentiometer read flag
    IFS1bits.CCT2IF = 0;            //Clear SCCP2 Interrupt Flag
    CCP2CON1bits.ON = 0;            //Stop SCCP2
    CCP2TMRbits.TMRL = 0;
}

