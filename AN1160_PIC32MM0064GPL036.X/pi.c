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
 * 12/3/2019 Prysm Inc. Brian Tremaine
 * Code base AN1160_PIC32MM0064GPL_MCLV_MCLV taken as starting point for
 * Java project (Gen3).
 * Rebuild for the PIC32MM Curiosity Development Board.
 * Add dual loop PI + phase detector
 * 
* *****************************************************************************/

#include "pi.h"
#include "defs.h"
/************************************************************************
InitPI - function to init the PI Controller
    tPIParam    //see definition in pi.h
    Kp			//Proportional Gain
    Ki			//Integral Gain
    Kc			//Anti-windup Gain
    max 		//PI Output maximum limit
    min 	 	//PI Output minimum limit
    out			//PI initial output
************************************************************************/
void InitPI(tPIParm *pParm,int16_t Kp,int16_t Ki,int16_t Kc,int16_t max,int16_t min,int16_t out)
{
    pParm->Sum = 0;
    pParm->Kp = Kp;
    pParm->Ki = Ki;
    pParm->Kc = Kc;
    pParm->OutMax = max;
    pParm->OutMin = min;
    pParm->Out = out;

}

/* CalcPI - function to calculate the output of the PI */
void CalcPI( tPIParm *pParm)
{
    // need to implement Q.N fixed point arithmetic
    // use this PI for inner & outer loop compensation.
 /*
;    Error  = Reference - Measurement
;    U  = Sum + Kp * Error
;    if( U > Outmax )
;        Out = Outmax
;    else if( U < OutMin )
;        Out = OutMin
;    else
;        Out = U
;    Exc = U - Out
;    Sum = Sum + Ki * Err - Kc * Exc
*/
    int16_t currentError;
    int32_t U;
    int16_t outTemp;

    //Error  = Reference - Measurement
    currentError = pParm->InRef - pParm->InMeas;
    //U  = Sum + Kp * Error
    U = (currentError*pParm->Kp);    
    U = U + pParm->Sum;


    //limit the output between the allowed limits
    //pParm->qOut is the PI output
    outTemp = (int32_t)(U>>15);
    if(outTemp >  pParm->OutMax)
        pParm->Out=  pParm->OutMax;
    else if(outTemp < pParm->OutMin)
        pParm->Out =  pParm->OutMin;
    else
        pParm->Out = outTemp;

    //U = Ki * Err
    U = (currentError*pParm->Ki);

    //compute the difference between the limited and not limited output
    //currentError is used as a temporary variable
    currentError = outTemp - pParm->Out;

    //U = U - Kc * Err = Ki * Err - Kc * Exc
    U -= (currentError*pParm->Kc);

    //Sum = Sum + U = Sum + Ki * Err - Kc * Exc
    pParm->Sum = pParm->Sum + U;
}

/* phase_det - function to calculate phase error */
void phase_det( )
{
    
    
}

/* pi_outer */
void pi_outer( )
{
  // PI for outer loop, result is Fref   
    
}

/* fg_det */
void fg_det( )
{
  /*  frequency error detector between motor Fg and outer loop Fref
  */
    
    
}

/* pi_inner */
void pi_inner( )
{
 // PI for inner loop, result is correction error to pwm
    
}

/* update pwm value*/
update_pwm()
{
    /* update pwm value to drive motor */
    
}

/* ISRs for servo PI*/

/* Fref input edge*/
Fref_ISR()
{
  // Fref rising edge, set PU and call phase_det()  
}

/* Fg input edge */
Fg_ISR()
{
    // Fg rising edge, increment Ffb
    // compute average Fg
    // if Ffb rising edge set PD and call phase_det())
    
}