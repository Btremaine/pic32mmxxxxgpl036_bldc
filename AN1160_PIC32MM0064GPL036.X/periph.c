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

#include <proc/p32mm0064gpl036.h>

#include "periph.h"

// PIC32MM0064GPL036 Configuration Bit Settings
// for curiosity board use internal 8MHz clock scaled up 4x to 24MHz
// product will use primary osc 8MHz scaled up 4x to 24MHz.
// FDEVOPT
#pragma config SOSCHP = OFF             // Secondary Oscillator High Power Enable bit (SOSC oprerates in normal power mode.)

// FICD
#pragma config JTAGEN = OFF              // JTAG Enable bit (JTAG is enabled)
#pragma config ICS = PGx3                // ICE/ICD Communication Channel Selection bits (Communicate on PGEC3/PGED3)

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
#pragma config RETVR = OFF              // Retention Voltage Regulator Enable bit (Retention regulator is disabled)
#pragma config LPBOREN = ON             // Downside Voltage Protection Enable bit (Downside protection is enabled)

// FWDT
#pragma config SWDTPS = PS1048576       // Sleep Mode Watchdog Timer Postscale Selection bits (1:1048576)
#pragma config FWDTWINSZ = PS25_0       // Watchdog Timer Window Size bits (Watchdog timer window size is 25%)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Enable bit (Watchdog timer is in non-window mode)
#pragma config RWDTPS = PS1048576       // Run Mode Watchdog Timer Postscale Selection bits (1:1048576)
#pragma config RCLKSEL = LPRC           // Run Mode Watchdog Timer Clock Source Selection bits (Clock source is LPRC (same as for sleep mode))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (WDT is disabled)

// FOSCSEL
#pragma config FNOSC = PLL
#pragma config PLLSRC = FRC             // System PLL Input Clock Selection bit (FRC oscillator is selected as PLL reference input on device reset)
#pragma config SOSCEN = ON
#pragma config IESO = ON                // Two Speed Startup Enable bit (Two speed startup is enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Selection bit (Primary oscillator is disabled)
#pragma config OSCIOFNC = OFF           // System Clock on CLKO Pin Enable bit (OSCO pin operates as a normal I/O)
#pragma config SOSCSEL = ON             // Enable input on pins shared wth SOSC
#pragma config FCKSM = CSECME           // Clock Switching and Fail-Safe Clock Monitor Enable bits (Clock switching is enabled; Fail-safe clock monitor is enabled)

// FSEC
#pragma config CP = OFF                 // Code Protection Enable bit (Code protection is disabled)

/* Below code is required to enable Multivectored Interrupts in PIC32MM */
void EnableInterrupts()
{
    INTCONbits.MVEC = 1;
    asm volatile("ei $0");
}

void DisableInterrupts()
{
    asm volatile("di $0");
}

void Init_Peripheral(void)
{
    DisableInterrupts();
    SYSKEY = 0;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    OSCCONbits.FRCDIV = 0;
    OSCCONbits.COSC = 0;      // internal fast RC oscillator
    SPLLCONbits.PLLODIV = 0;  // div 1
    SPLLCONbits.PLLMULT = 1;  // x3
    SPLLCONbits.PLLICLK = 1;
    REFO1CONbits.ON = 1;     // debug, output sys clock
    REFO1CONbits.OE = 1;
    SYSKEY = 0x0;
    while(OSCCON & 0x0001);
    Nop();
    Nop();
    Nop();

    while(OSCCON & 0x0001);
    Nop();
    Nop();
    Nop();

     // Configure Digital PORTS multiplexed with MCCP as outputs
    LATAbits.LATA1 = 0;     //PWM1HU                              OCM1F MCCP
    TRISAbits.TRISA1 = 0;
    LATAbits.LATA0 = 0;     //PWM1LU, <also LED1 on Dev Board>    OCM1E port
    TRISAbits.TRISA0 = 0;
    LATAbits.LATA3 = 0;     //PWM1HV                              OCM1D MCCP
    TRISAbits.TRISA3 = 0;
    LATAbits.LATA2 = 0;     //PWM1LV, <also RGB_BLE on Dev Board> OCM1C port
    TRISAbits.TRISA2 = 0;
    LATBbits.LATB9 = 0;     //PWM1HW                              OCM1B MCCP
    TRISBbits.TRISB9 = 0;
    LATBbits.LATB8 = 0;     //PWM1LW, <also RB8_SCK on Dev Board> OCM1A port
    TRISBbits.TRISB8 = 0;

    // Push Button pins
#if defined CURIOS_DEV    
    LATBbits.LATB7 = 0;     // S1
    TRISBbits.TRISB7 = 1;
 
    LATBbits.LATB13 = 0;     // S2
    TRISBbits.TRISB13 = 1;
    
    //LATAbits.LATA0 = 0;      // LED1 see digital ports
    //TRISAbits.TRISA0 = 0;  
    
    LATBbits.LATB12 = 0;     // LED3 Green
    TRISBbits.TRISB12 = 0;   
    
    LATCbits.LATC9 = 0;      // LED2
    TRISCbits.TRISC9 = 0;
      
#endif
  
    EnableInterrupts();
}


void Init_ADC(void)
{
    // clear all ADC's
    AD1CON1 = 0;
    AD1CON2 = 0;               // VCFG RefH: AVdd, RefL: AVss 
    AD1CON3 = 0;
    AD1CON5 = 0;

    AD1CON1bits.SIDL = 0;      // continues operation in idle mode
    AD1CON1bits.FORM = 0;      // int 16bit format
    AD1CON1bits.SSRC = 5;      // timer1 ends sampling starts conversion
    AD1CON1bits.MODE12 = 0;    // 10-bit mode
    AD1CON1bits.ASAM = 1;      // automatic sampling
    
    AD1CHSbits.CH0SA = POT;    // temporary channel setting for now
    
    AD1CON3bits.ADCS = 12;     // conversion 12*Tsrc (1 per bit + 2)
    AD1CON3bits.SAMC = 4;      // sample 4*Tsrc
    AD1CON3bits.EXTSAM = 0;    // stop sampling when SAMP=0
    AD1CON3bits.ADRC = 0;      // clock derived from peripheral bus
    
    AD1CON1bits.DONE = 0;
    
    IPC3bits.AD1IP = 7;         // Set ADC interrupt priority
    IPC3bits.AD1IS = 3;         // Set ADC interrupt sub priority
    IEC0bits.AD1IE = 1;         // Enable ADC interrupt
    AD1CON1bits.ON = 1;
}
void Init_MCCP(void)
{
    CCP1PRbits.PRL = ((FCY/FPWM) - 1);  //Setting PWM Period

    CCP1CON1bits.SIDL = 1;          // MCCP time base halted in CPU IDLE mode

    // Set MCCP operating mode
    CCP1CON1bits.CCSEL = 0;         // Set MCCP operating mode (OC/timer mode)
    CCP1CON1bits.MOD = 5;           // Set mode (Buffered Dual-Compare/PWM mode)

    //Configure MCCP Timebase
    CCP1CON1bits.T32 = 0;           // Set timebase width (16-bit)
    CCP1CON1bits.TMRSYNC = 0;       // Set timebase synchronization (Synchronized)
    CCP1CON1bits.CLKSEL = 0b000;    // Set the clock source (Tcy)
    CCP1CON1bits.TMRPS = 0b00;      // Set the clock pre-scaler (1:1)
    CCP1CON1bits.TRIGEN = 0;        // Set Sync/Triggered mode (Synchronous)
    CCP1CON1bits.SYNC = 0b00000;    // Select Sync/Trigger source (Self-sync)
    
    //Configure MCCP output for PWM signal
    // MCCP enables set in code, ports manually written
    CCP1CON2 = 0x0000;

    //Configure MCCP output for PWM signal
    CCP1CON2bits.OCAEN = 0;         // Control desired output signals (OC1A)
    CCP1CON2bits.OCBEN = 0;         // Control desired output signals (OC1B)
    CCP1CON2bits.OCCEN = 0;         // Control desired output signals (OC1C)
    CCP1CON2bits.OCDEN = 0;         // Control desired output signals (OC1D)
    CCP1CON2bits.OCEEN = 0;         // Control desired output signals (OC1E)
    CCP1CON2bits.OCFEN = 0;         // Control desired output signals (OC1F)
    CCP1CON2bits.OENSYNC = 1;

    CCP1CON3bits.OUTM = 0;          // Steerable Single output mode)
    CCP1CON3bits.POLACE = 0;        // Configure output polarity (Active High)
    CCP1CON3bits.DT = 0;            // dead-time logic disabled
    
    CCP1TMRbits.TMRL = 0x0000;      //Initialize timer prior to enable module.
    CCP1RAbits.CMPA = 0;
    CCP1RBbits.CMPB = 10;           //Small Duty Cycle to enable MCCP Interrupt Run for Delay function

    IPC7bits.CCP1IP = 3;            // MCCP Interrupt Priority 4
    IPC7bits.CCP1IS = 2;
    IFS0bits.CCP1IF = 0;            // Clearing the MCCP Interrupt Flag
    IEC0bits.CCP1IE = 1;            // Enabling the MCCP interrupt

    CCP1CON1bits.ON = 1;            // Turn on MCCP module

}

void Init_Timers(void)
{
    // ======================== // Timer 1
    T1CONbits.ON = 0;
    TMR1 = 0;                   // Resetting timer 1
    PR1 = ((FCY/FPWM)/7 - 1);   // Frequency at which Timer1 triggers ADC. About 6 ADCs every PWM.
    T1CONbits.TCS = 0;          // internal peripheral clock
    
    // ======================== // SCCP2:
    CCP2CON1bits.ON = 0;        // Stop SCCP2
    CCP2CON1bits.CCSEL = 0;     // Set SCCP2 operating mode, Output Compare/PWM or Timer mode 
    CCP2CON1bits.MOD = 0;       // Set mode to 16/32 bit timer mode
    CCP2CON1bits.T32 = 0;       // Set timebase width (16-bit)
    CCP2CON1bits.TMRSYNC = 0;   // Set timebase synchronization (Synchronized)
    CCP2CON1bits.CLKSEL = 0b000;// Set the clock source (Tcy) Fsys
    CCP2CON1bits.TMRPS = 0b11;  // Set the clock pre-scaler (1:64)
    CCP2CON1bits.TRIGEN = 0;    // Set Sync/Triggered mode (Synchronous)
    CCP2CON1bits.SYNC = 0b00000;// Timer is in the Free-Running mode and rolls over at FFFFh (Timer Period register is ignored)
    CCP2TMRbits.TMRH = 0;       // Initialize timer prior to enable module.
    CCP2TMRbits.TMRL = 0;
    CCP2PRbits.PRH = 0;
    CCP2PRbits.PRL = 10;        // Configure timebase period. This is always changed in pre-commutation state.
    // ========================= // SCCP3:
    CCP3CON1bits.CCSEL = 0;     // Set SCCP3 operating mode
    CCP3CON1bits.MOD = 0b0000;  // Set mode to 16/32 bit timer mode
    CCP3CON1bits.T32 = 0;       // Set timebase width (16-bit)
    CCP3CON1bits.TMRSYNC = 0;   // Set timebase synchronization (Synchronized)
    CCP3CON1bits.CLKSEL = 0b000;// Set the clock source (Tcy)
    CCP3CON1bits.TMRPS = 0b11;  // Set the clock pre-scaler (1:64)
    CCP3CON1bits.TRIGEN = 0;    // Set Sync/Triggered mode (Synchronous)
    CCP3CON1bits.SYNC = 0b00000;// No external synchronization; timer rolls over at FFFFh or matches with the Timer Period register
    CCP3TMRbits.TMRH = 0;       // Initialize timer prior to enable module.
    CCP3TMRbits.TMRL = 0;
    CCP3PRbits.PRH = 0;
    CCP3PRbits.PRL = 0xffff;    // Configure timebase period. This timer does not need to overflow. Used only for counting time.
    // ======================== // ?
    IPC8bits.CCT2IP = 5;        // Set SCCP2 Interrupt Priority Level
    IPC8bits.CCT2IS = 2;
    IFS1bits.CCT2IF = 0;        // Clear SCCP2 Interrupt Flag
    IEC1bits.CCT2IE = 1;        // Enable SCCP2 interrupt

    CCP2CON1bits.ON = 0;        // Stop SCCP2
    T1CONbits.ON = 0;           // Stop Timer1
}

void Toggle_LED2(void)
{
   LATCSET = (0b1000000000);  // test: SET LED2 in atomic operation 
   Nop(); Nop(); Nop(); Nop();Nop(); Nop(); Nop(); Nop();
   LATCCLR = (0b1000000000);  // test: CLR LED2 in atomic operation  
}

void Toggle_LED3(void)
{
    LATBSET = (0b1000000000000);  // test: SET LED3 in atomic operation 
    Nop(); Nop(); Nop(); Nop(); 
    LATBCLR = (0b1000000000000);  // test: CLR LED3 in atomic operation          
}

void Set_LED2(void)
{
   LATCSET = (0b1000000000);  // test: SET LED2 in atomic operation  
}

void Clr_LED2(void)
{
    LATCCLR = (0b1000000000);  // test: CLR LED2 in atomic operation          
}