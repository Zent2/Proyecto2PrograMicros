/*
 * File:   main.c
 * Author: Christian Campos 21760
 *
 * Created on 18 de mayo de 2023, 06:48 AM
 */
// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

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

#include <xc.h>
#include <string.h>
#include <stdint.h>
#define _XTAL_FREQ 500000
//Espacios para guardar valores en la EEPROM
#define dirEEPROM1 0x04
#define dirEEPROM2 0x08
#define dirEEPROM3 0x12
#define dirEEPROM4 0x16
//Valor precargado Timer0
#define TMR0_Value 238
int contador, bandera = 1, contador2; //Banderas
char  adc1, adc2, adc3, adc4 ; //Valores convertidos de Analogico a Digital
char work_value1, work_value2, work_value3, work_value4; //Valores mapeados basados en la adc
int preload; 
int i; 

//Funciones generales
void setup (void);
void writeToEEPROM(uint8_t data, uint8_t address); 
uint8_t readFromEEPROM (uint8_t address);

//RUTINAS DE LA EEPROM (enviamos el valor a guardar y el espacio donde se guardara)
void writeToEEPROM (uint8_t data, uint8_t address){
    EEADR=address; 
    EEDAT=data;
    
    EECON1bits.EEPGD=0; //Escribir a memoria de datos
    EECON1bits.WREN=1; //Habilitar modo escritura EEPROM
    
    INTCONbits.GIE=0; //Deshabilitar interrupciones
    
    EECON2=0x55; //SECUENCIA OBLIGATORIA
    EECON2=0xAA;
    EECON1bits.WR=1; //Habilitar escritrua
    
    
    EECON1bits.WREN=0; //Deshabilitar escritura de EEPROM
    INTCONbits.GIE=1; //Habilitar interrupciones
}

uint8_t readFromEEPROM(uint8_t address){
    EEADR=address; //Ir a la dirección indicada
    EECON1bits.EEPGD=0; 
    EECON1bits.RD=1;
    return EEDAT; //Retornar el valor guardado en la direccion indicada
}
//Rutina de interrupciones
void __interrupt () isr (void){
    //Interrupción por el Timer0
    if (INTCONbits.TMR0IF){
        //Ver si el valor del Timer2 (no del Timer0) es mayor o igual al work_value 1 y 4
        //Con ayuda del TMR2 y de su interrupción controlamos el motor 1 y 4
        if (TMR2>=work_value1){
            RC0=0; //Apagamos el puerto RC0 para generar el work duty
        }
        if (TMR2>=(work_value4)){
            RC3=0;//Apagamos el puerto RC3 para generar el work duty
        }
        if (TMR2>=(work_value4) && TMR2>=work_value1){
            INTCONbits.TMR0IE=0; //Si es mayor que ambos, apagamos la interrupción porque no nos sirve más
        } 
        TMR0 = TMR0_Value; //Reiniciamos el Timer0
        INTCONbits.TMR0IF=0; //Apagamos la bandera
    }
    //Interrupción por el Timer2
    if (PIR1bits.TMR2IF) {
        RC0=1; //Encendemos los 2 puertos para generar el periodo
        RC3=1;
        PIR1bits.TMR2IF=0; //Apagamos la bandera
        TMR2 = 0; // Reiniciar timer2
        PR2 = 155.25; // Valor de período para 20kHz (500kHz / (4 * (PR2 + 1)))
        INTCONbits.TMR0IE=1; //Volvemos a habilitar la interrupción del Timer0
        }
    // Interrupción por el puerto B (pull-ups)
    if (INTCONbits.RBIF==1 && INTCONbits.TMR0IF==0){
        INTCONbits.RBIF=0; //Limpiamos bandera
        ADCON0bits.GO=0;  //Apagar el adc
        //En cualquier modo estos botones sirven para cambiar de modo
        if (RB0==0){
                PORTE=0b001; //Modo 1, Potenciometros/Manual
            }
        if (RB1==0){
                PORTE=0b010; //Modo 2, ADAFRUIT o EUSART (Serial)
            }
        if (RB2==0){
                PORTE=0b100; //Modo 3, EEPROM
            }
        // En dado caso no estemos en modo EEPROM (o sea Manual o Serial)
        if (PORTEbits.RE1==0){
        if (RB3==0){
                PORTD=0b0001; //Mostramos que se guardo el valor del primer motor
                writeToEEPROM(work_value1, dirEEPROM1); //Escribimos el valor en la EEPROM
            }
            if (RB4==0){
                PORTD=0b0010; //Mostramos que se guardo el valor del segundo motor
                writeToEEPROM(work_value2, dirEEPROM2);//Escribimos el valor en la EEPROM
            }
            if (RB5==0){
                PORTD=0b0100;//Mostramos que se guardo el valor del tercer motor
                writeToEEPROM(work_value3, dirEEPROM3);//Escribimos el valor en la EEPROM
            }
            if (RB6==0){
                PORTD=0b1000;//Mostramos que se guardo el valor del cuarto motor
                writeToEEPROM(work_value4, dirEEPROM4);//Escribimos el valor en la EEPROM
            }
        }
        if (PORTEbits.RE1==1) {
            if (RB3==0){
                PORTD=0b0001;//Mostramos que se cargo el valor del primer motor
                work_value1=readFromEEPROM(dirEEPROM1);//Leemos el valor en la EEPROM
            }
            if (RB4==0){
               PORTD=0b0010; //Mostramos que se cargo el valor del segundo motor
               work_value2=readFromEEPROM(dirEEPROM2); //Leemos el valor en la EEPROM
               CCPR2L=(work_value2);//Mandamos el valor al CCP2 
            }
            if (RB5==0){
                PORTD=0b0100;//Mostramos que se cargo el valor del tercer motor
                work_value3=readFromEEPROM(dirEEPROM3);//Leemos el valor en la EEPROM
                CCPR1L=(work_value3); // Mandamos el valor al CCP1; estos 2 motores sirven con el CCP
            }
            if (RB6==0){
                PORTD=0b1000;//Mostramos que se cargo el valor del cuarto motor
                work_value4=readFromEEPROM(dirEEPROM4);//Leemos el valor en la EEPROM
            }
        }
       ADCON0bits.GO=1;     
    }
     // En el modo Serial
    if(PIR1bits.RCIF && PORTEbits.RE2)       //Interrupcion entrada EUSART
        {
        if(bandera == 1){       // Centenas
            adc1 = 0;
            adc1 += (RCREG - 48)*100;      //Sumar valor centenas
            bandera = 2;
        }
        if(bandera == 2 && PIR1bits.RCIF){  // Decenas
            adc1 += (RCREG - 48)*10;       // Sumar valor denceas
            bandera = 3;
        }
        if(bandera == 3 && PIR1bits.RCIF){  // Unidades
            adc1 += (RCREG - 48);          // Sumar valor unidades
            bandera = 4;
            work_value1=(adc1*(8.75/255))+1.6;            
        }
        if(bandera == 4 && PIR1bits.RCIF){       // Centenas
            __delay_us(10);
            adc2 = 0;
            adc2 += (RCREG - 48)*100;      //Sumar valor centenas
            bandera = 5;
        }
        if(bandera == 5 && PIR1bits.RCIF){  // Decenas
            adc2 += (RCREG - 48)*10;       // Sumar valor denceas
            bandera = 6;
        }
        if(bandera == 6 && PIR1bits.RCIF){  // Unidades
            adc2 += (RCREG - 48);          // Sumar valor unidades
            bandera = 7;
            work_value2=(adc2*(7.25/255))+6.9;            
        }
        if(bandera == 7 && PIR1bits.RCIF){       // Centenas
            __delay_us(10);
            adc3 = 0;
            adc3 += (RCREG - 48)*100;      //Sumar valor centenas
            bandera = 8;
        }
        if(bandera == 8 && PIR1bits.RCIF){  // Decenas
            adc3 += (RCREG - 48)*10;       // Sumar valor denceas
            bandera = 9;
        }
        if(bandera == 9 && PIR1bits.RCIF){  // Unidades
            adc3 += (RCREG - 48);          // Sumar valor unidades
            bandera = 10;
            work_value3=(adc3*(7.25/255))+6.9;            
        }
        if(bandera == 10 && PIR1bits.RCIF){       // Centenas
            __delay_us(10);
            adc4 = 0;
            adc4 += (RCREG - 48)*100;      //Sumar valor centenas
            bandera = 11;
        }
        if(bandera == 11 && PIR1bits.RCIF){  // Decenas
            adc4 += (RCREG - 48)*10;       // Sumar valor denceas
            bandera = 12;
        }
        if(bandera == 12 && PIR1bits.RCIF ){  // Unidades
            adc4 += (RCREG - 48);          // Sumar valor unidades
            bandera = 1;
            work_value4=((adc4)*(8.75/255))+9.9;            
        }
        __delay_us(10);
        PORTD=adc2;
    }
}

void main(void) {
    setup();
    while(1){
       
        //Primer modo
        if(PORTEbits.RE0) {
            RCSTAbits.CREN = 0;             //Deshabilitar para recibir datos
            TXSTAbits.TXEN = 1;             //Habilitar para enviar datos
            PIE1bits.ADIE=0;                //Sin interrupción ADC
            ADCON0bits.GO=1;                //Iniciamos conversión ADC
        } 
        // Primer modo Manual y conversión ADC
        if (ADIF==1 && PORTEbits.RE0==1) {
            ADIF=0; //Limpiamos bandera
            //Primero verificamos cual de las 4 entradas analógicas es la que está haciendo la conversión
            if (ADCON0bits.CHS==0b0000){
        
            //La primera entrada controla el primer motor
            __delay_us(4);
            adc1=ADRESH;
            
            __delay_us(4);
            work_value1=(adc1*(8.75/255))+1.6;//Mapeo del ADC
            __delay_us(4);
            ADCON0bits.CHS=1; //Cambiamos la entraga analógica
            ADCON0bits.GO=1;
            }
            if (ADCON0bits.CHS==0b0001){
            //La segunda entrada controla el segundo motor
            __delay_us(4);
            adc2=ADRESH;
            
            __delay_us(4);
            //
            work_value2=(adc2*(7.25/255))+6.9;//Mapeo del ADC
            
            CCPR2L=work_value2; //Mandamos el valor al CCP2
           
            __delay_us(4);
            ADCON0bits.CHS=2; //Cambiamos la entraga analógica
            ADCON0bits.GO=1;
             }
            if(ADCON0bits.CHS==0b0010) {
            //La segunda tercera entrada controla el tercer motor
            __delay_us(4);
            adc3=ADRESH;
            
            __delay_us(4);
            work_value3=(adc3*(7.25/255))+6.9; //Mapeo del ADC
            CCPR1L=(work_value3); //Mandamos el valor al CCP1
            __delay_us(4);
            ADCON0bits.CHS=3; //Cambiamos la entrada analógica
            ADCON0bits.GO=1;
            }
            if (ADCON0bits.CHS==0b0011){
            //La cuarta entrada controla el cuarto motor
            __delay_us(4);
            adc4=ADRESH;
            
            __delay_us(4);
            work_value4=((adc4)*(8.75/255))+9.9; //Mapeo del ADC
            
            __delay_us(4);
            ADCON0bits.CHS=0; //Cambiamos la entrada analógica
            ADCON0bits.GO=1;
            }
        }
        //Segundo modo (EEPROM)
        if(PORTEbits.RE1) {
            
            RCSTAbits.CREN = 0;             //Deshabilitar para recibir datos
            TXSTAbits.TXEN = 1;             //Habilitar para enviar datos
            PIE1bits.ADIE=0;
            ADCON0bits.GO=0;
            CCPR1L=(work_value3); //Mandamos los valores a sus respectivos CCPs
            CCPR2L=(work_value2);
        } 
        //Tercer modo (Serial)
        if(PORTEbits.RE2) {
            
            RCSTAbits.CREN = 1;             //Habilitar para recibir datos
            TXSTAbits.TXEN = 1;             //Deshabilitar para enviar datos
            PIE1bits.ADIE=0;
            ADCON0bits.GO=0;
            CCPR1L=(work_value3); //Mandamos los valores a sus respectivos CCPs
            CCPR2L=(work_value2);
        }    
    }
    return; 
}
void setup (void){
    //Configuracion I/O
    ANSEL = 0;             
    ANSELH = 0;
    TRISD = 0;
    TRISB = 255;
    TRISC=0b10000000;
    TRISE=0;
    PORTB = 0;
    PORTD = 0;
    PORTE=0b001;
    
    //Configuracion EUSART
    TXSTAbits.SYNC = 0;             //Modo asincrono
    TXSTAbits.BRGH = 1;             //High speed baud rate

    BAUDCTLbits.BRG16 = 1;          //16-bit Baud Rate
    SPBRG = 12.5;
    SPBRGH = 0;
    
    RCSTAbits.SPEN = 1;             //Serial port enable
    RCSTAbits.RX9 = 0;              //8 bits de datos
    RCSTAbits.CREN = 1;             //Habilitar para recibir datos
    
    TXSTAbits.TXEN = 1;             //Habilitar para enviar datos
    
    //Interrupciones EUSART
    PIR1bits.RCIF = 0;            //Bandera RX
    PIE1bits.RCIE = 1;            //INT EUSART RC
    
    //Configuracion de interrupciones
    INTCONbits.GIE = 1;         //INT globales
    INTCONbits.PEIE=1;          //INT perifericas
    
    //Configuracion de pullups para el PORTB
    OPTION_REGbits.nRBPU = 0;
    WPUBbits.WPUB0 = 1;
    WPUBbits.WPUB1 = 1;
    WPUBbits.WPUB2 = 1;
    WPUBbits.WPUB3 = 1;
    WPUBbits.WPUB4 = 1;
    WPUBbits.WPUB5 = 1;
    WPUBbits.WPUB6 = 1;
    
    //Configuracion de interrupciones PORTB
    INTCONbits.RBIE = 1;
    INTCONbits.RBIF = 0;
    IOCBbits.IOCB0 = 1;
    IOCBbits.IOCB1 = 1;
    IOCBbits.IOCB2 = 1;
    IOCBbits.IOCB3 = 1;
    IOCBbits.IOCB4 = 1;
    IOCBbits.IOCB5 = 1;
    IOCBbits.IOCB6 = 1;
    
    
    //Configuraacion de oscilador interno
    OSCCONbits.IRCF=0b011;      //500kHz oscilador interno   
    OSCCONbits.SCS=1;
    
    //Configuración Conversor ADC
    ADIF=0; //Bandera inicia en 0
    
    
    // Configuración del puerto CCP1 y CCP2 PWM
    CCP1CON = 0b00001100; // Modo PWM, salida activa alta
    CCP2CON = 0b00001100; // Modo PWM, salida activa alta
    T2CON = 0b00000111; // Timer2 configurado para 1:16
    TMR2 = 0; // Reiniciar timer2
    PR2 = 155.25; // Valor de período para 20Hz (500kHz / (4 * (PR2 + 1)))
    CCPR1L=11.6;
    CCPR2L=11.6;
    
    //config ADC
    ANSEL=0b00001111; //Entradas analógicas (AN0-AN3)
    TRISA=0b00001111;
    PORTA=0b00001111;
    ADCON0bits.ADCS=0;
    ADCON0bits.CHS= 1;
    ADCON1bits.ADFM=0;
    __delay_ms(10);    
    ADCON1bits.VCFG0=0;
    ADCON1bits.VCFG1=0;
    ADCON0bits.ADON=1;  
    
    T2CONbits.TMR2ON = 1;       //Encender TMR2
    PIE1bits.TMR2IE=1;
    PIR1bits.TMR2IF=0;
    
    //Timer0 Registers Prescaler= 1 - TMR0 Preset = 243 - Freq = 9615.38 Hz - Period = 0.000104 seconds
    INTCONbits.TMR0IE=1;
    OPTION_REGbits.T0CS = 0;  // bit 5  TMR0 Clock Source Select bit...0 = Internal Clock (CLKO) 1 = Transition on T0CKI pin
    OPTION_REGbits.T0SE = 0;  // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
    OPTION_REGbits.PSA = 0;   // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to the WDT
    OPTION_REGbits.PS2 = 0;   // bits 2-0  PS2:PS0: Prescaler Rate Select bits
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 0;
    TMR0 = TMR0_Value;             // preset for timer register
    INTCONbits.TMR0IF=0;
    

    return;
    
} 

