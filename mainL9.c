/* 
 * File:   mainL9.c
 * Author: Jeferson Noj
 *
 * Created on 26 de abril de 2022, 09:29 PM
 */

// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT enabled)
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
#include <stdio.h>
#include <stdlib.h>

// Constantes
#define _XTAL_FREQ 500000
#define IN_MIN 0               // Valor minimo de entrada del potenciometro
#define IN_MAX 255             // Valor máximo de entrada del potenciometro
#define OUT_MIN 31             // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 63             // Valor máximo de ancho de pulso de señal PWM

// Variables
unsigned short CCPR = 0;        // Variable para almacenar ancho de pulso al hacer la interpolación lineal

// Prototipo de funciones
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

// Interrupciones
void __interrupt() isr (void){
    if(PIR1bits.ADIF){                      // Verificar si ocurrió interrupción del ADC
        if(ADCON0bits.CHS == 0){            // Verificar que AN0 sea el canal seleccionado
            CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de anchode pulso
            CCPR1L = (uint8_t)(CCPR>>2);    // Guardar los 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11; // Guardar los 2 bits menos significativos en DC1B
        }
        PIR1bits.ADIF = 0;                  // Limpiar bandera de interrupción
    }
    return;
}

// Ciclo principal
void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion
            ADCON0bits.GO = 1;              // Iniciamos proceso de conversi n?
        }
    }
    return;
}

// Configuración 
void setup(void){
    ANSEL = 0b1;                // AN0 como entrada analógica
    ANSELH = 0;                 // I/O digitales
    TRISA = 0b1;                // AN0 como entrada
    PORTA = 0;
    
    // Reloj interno?
    OSCCONbits.IRCF = 0b011;    // 500 KHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionar el AN0
    ADCON1bits.ADFM = 0;        // Justificar a la izquierda
    ADCON0bits.ADON = 1;        // Habilitar modulo ADC
    __delay_us(40);             // Sample time
    
    // PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitar salida de CCP1
    PR2 = 155;                  // Periodo de 20ms
    
    // CCP1
    CCP1CON = 0;                // Apagar CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM
    
    CCPR1L = 63>>2;
    CCP1CONbits.DC1B = 63 & 0b11;    // 2 ms ancho de pulso / 10% ciclo de trabajo
    
    PIR1bits.TMR2IF = 0;        // Limpiar bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encender TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiar bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Habilitar salida de PWM
    // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpir bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitar interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitar int. de perifericos
    INTCONbits.GIE = 1;         // Habilitar int. globales
    
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}