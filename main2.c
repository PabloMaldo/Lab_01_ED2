/*
 * File:   main2.c
 * Author: pablo
 *
 * Created on July 18, 2023, 4:53 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
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

// LIBRERIAS
#include <xc.h>
#include <stdint.h>
#include "Interrupciones.h"
#include "adc.h"
#include "display.h"

// VARIABLES GLOBALES
char adc_value;
int flags;

/*
 * Constantes
 */
#define _XTAL_FREQ 4000000
#define _tmr0_value 200

/*
 * Prototipos de funcion
 */
void setup(void);

void __interrupt() isr(void)
{
    if(INTCONbits.RBIF)
    {
        if (PORTBbits.RB0 == 0){
            PORTA++;
        }
        
        if (PORTBbits.RB1 == 0){
            PORTA--;
        }
        
        INTCONbits.RBIF = 0;
    }
    
    if (PIR1bits.ADIF) {  //Si se activa la bandera de interrupcion del ADC
        adc_value =  adc_read();
        __delay_ms(10);
        ADCON0bits.GO = 1;
        PIR1bits.ADIF = 0; //Limpiar la bandera de la interrupcion del ADC
    }
    
    if (INTCONbits.T0IF){ // Si se activa la bandera del tmr0
        
        PORTD = 0;
        if (flags == 0){
            char up_bit = upper_bits(adc_value);
            PORTC = show_display(up_bit);
            PORTD = 0b0001;
            flags = 1;
        }
        else if (flags == 1){
            char low_bit = lower_bits(adc_value);
            PORTC = show_display(low_bit);
            PORTD = 0b0010;
            flags = 0;
        }
        
        
        INTCONbits.T0IF = 0;
        TMR0 = _tmr0_value;
    }
}    

void main(void) {
    
    setup();
    ADCON0bits.GO = 1;
    while(1){
        if (PORTA == ADRESH){
            PORTD = 0b10000000;
        }
    }
}

void setup(void){
    // CONFIGURACION DE ENTRADAS Y SALIDAS
    // Pines digitales
    ANSEL = 0;
    ANSELH = 0;
    
    TRISA = 0;
    PORTA = 0;
    TRISD = 0;
    
    TRISC = 0;
    PORTC = 0;
    
    //Configuracion TMR0
    OPTION_REGbits.T0CS = 0; //Se selecciona el timer como temporizador
    OPTION_REGbits.PSA = 0; // Prescaler activado para TMR0
    OPTION_REGbits.PS = 0b111; // Prescaler 1:256
    TMR0 = _tmr0_value;
    
    ioc_init(0);
    ioc_init(1);
    
    adc_init(6);
    return;
}
