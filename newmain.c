/*
 * File:   newmain.c
 * Author: aldoa
 *
 * Created on 17 de octubre de 2022, 06:43 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 500000

int ADC;
int serv;
void setup(void);
void setupINTOSC(void);
void setupADC(void);
void setupPWM(void);
void servo(int valor);



//******************************************************************************
// CÃ³digo Principal
//******************************************************************************
void main(void) {
    
    setup();
    setupINTOSC();
    setupPWM();
    setupADC();
    while(1){
//******************************************************************************
// ADC
//******************************************************************************
        ADCON0bits.GO = 1;  // enciendo la bandera
        while(ADCON0bits.GO == 1){
            ;
        }
        ADIF = 0;           // apago la bandera
        ADC = ADRESH;
        servo (ADC);
            CCPR1L = serv;
            __delay_us(100);
        
    
        
    }
    return;
}

void servo(int valor){
    serv = (unsigned short) (7+( (float)(13)/(255) ) * (valor-0));
}
//******************************************************************************
// FunciÃ³n para configurar GPIOs
//******************************************************************************
void setup(void){
    ANSELH = 0;
    TRISB = 0;
    PORTB = 0;
    PORTC = 0;
}
//******************************************************************************
// FunciÃ³n para configurar PWM
//******************************************************************************
void setupINTOSC(void){
    OSCCONbits.IRCF = 0b011;       // 500 KHz
    OSCCONbits.SCS = 1;
}
//******************************************************************************
// FunciÃ³n para configurar ADC
//******************************************************************************
void setupADC(void){
    
    // Paso 1 Seleccionar puerto de entrada
    //TRISAbits.TRISA0 = 1;
    TRISA = TRISA | 0x01;
    ANSEL = ANSEL | 0x01;
    
    // Paso 2 Configurar mÃ³dulo ADC
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;       // Fosc/ 8
    
    ADCON1bits.VCFG1 = 0;       // Ref VSS
    ADCON1bits.VCFG0 = 0;       // Ref VDD
    
    ADCON1bits.ADFM = 0;        // Justificado hacia izquierda
    
    ADCON0bits.CHS3 = 0;
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS0 = 0;        // Canal AN0
    
    ADCON0bits.ADON = 1;        // Habilitamos el ADC
    __delay_us(100);
    
}
//******************************************************************************
// FunciÃ³n para configurar PWM
//******************************************************************************
void setupPWM(void){
    // Paso 1
    TRISCbits.TRISC2 = 1;
    
    // Paso 2
    PR2 = 155;      // Periodo de 20mS
    
    // Paso 3
    CCP1CON = 0b00001100;        // P1A como PWM 
            
   // Paso 4
    CCP1CONbits.DC1B = 0b11;        // CCPxCON<5:4>
    CCPR1L = 11;        // CCPR1L 
                        // CALCULO PARA 1.5mS de ancho de pulso
    
    // Paso 5
    TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11;      // Prescaler de 1:16
    TMR2ON = 1;         // Encender timer 2
    
    // Paso 6
    while(!TMR2IF);
    TRISCbits.TRISC2 = 0;   // Habilitamos la salida del PWM
    
}