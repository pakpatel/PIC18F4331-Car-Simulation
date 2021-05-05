#ifndef CONFIG_H
#define	CONFIG_H
#include <xc.h>

#define _XTAL_FREQ 8000000

// PIC18F4331 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = IRCIO      // Oscillator Selection bits (Internal oscillator block, port function on RA6 and port function on RA7)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal External Oscillator Switchover bit (Internal External Switchover mode enabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDPS = 32768     // Watchdog Timer Postscale Select bits (1:32768)
#pragma config WINEN = OFF      // Watchdog Timer Window Enable bit (WDT window disabled)

// CONFIG3L
#pragma config PWMPIN = OFF     // PWM output pins Reset state control (PWM outputs disabled upon Reset (default))
#pragma config LPOL = HIGH      // Low-Side Transistors Polarity (PWM0, 2, 4 and 6 are active-high)
#pragma config HPOL = HIGH      // High-Side Transistors Polarity (PWM1, 3, 5 and 7 are active-high)
#pragma config T1OSCMX = ON     // Timer1 Oscillator MUX (Low-power Timer1 operation when microcontroller is in Sleep mode)

// CONFIG3H
#pragma config FLTAMX = RC1     // FLTA MUX bit (FLTA input is multiplexed with RC1)
#pragma config SSPMX = RC7      // SSP I/O MUX bit (SCK/SCL clocks and SDA/SDI data are multiplexed with RC5 and RC4, respectively. SDO output is multiplexed with RC7.)
#pragma config PWM4MX = RB5     // PWM4 MUX bit (PWM4 output is multiplexed with RB5)
#pragma config EXCLKMX = RC3    // TMR0/T5CKI External clock MUX bit (TMR0/T5CKI external clock input is multiplexed with RC3)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (Enabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Low-Voltage ICSP Enable bit (Low-voltage ICSP disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000200-000FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (001000-001FFF) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000200-000FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (001000-001FFF) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (001000-001FFF) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#endif	/ XC_HEADER_TEMPLATE_H /

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define LCD_DATA    LATB
#define LCD_READ    PORTB
#define LCD_IO_BF   TRISB
#define LCD_BF      PORTDbits.RD4
#define LCD_RS      LATDbits.LD4
#define LCD_RW      LATDbits.LD3
#define LCD_EN      LATDbits.LD2

uint8_t print_buffer[33] = {0}; // buffer to print stuff to serial

volatile uint8_t uart_char = 0;
volatile bool uart_rcv_data = false;

uint16_t count = 1000; //counter for timer
uint16_t light_count= 100;
uint16_t flag = 0; //flag that tells the program timer is done
uint16_t pot_value; //will hold the original value
uint8_t voltage = 0; //converted value ranging from 0-5
uint8_t reset_flag = 0; //reset flag
uint8_t speed_value = 0; //speed value
uint8_t speed_count = 0; //speed count for speeding
uint8_t light_value = 0; //int that keeps track of light color

void LCD_clock(void){ //LCD commands were made using data sheet with very some modification for this pic
    LCD_EN = 1;
    __delay_ms(1);

    LCD_EN = 0;
    __delay_ms(1);
}

void LCD_command(uint8_t command){
    LCD_RS = 0;
    LCD_RW = 0;
    
    LCD_DATA = command;
    LCD_clock();
    __delay_ms(10);
}

void LCD_command_write(char data)  //Send 8-bits through 4-bit mode
{
    LCD_DATA = data;
    LCD_RW = 0;
    LCD_RS = 0;
    LCD_RS = 1;             // => RS = 1
    __delay_ms(50);
    LCD_EN = 0;
    LCD_EN = 1;
    for(int i=2130483; i<=0; i--)  NOP(); 
    LCD_EN = 0;
}

void LCD_init_8bits(void){
    LCD_RS = 0;
    LCD_RW = 0;
    
    __delay_ms(50);
    LCD_command(0x38);
    LCD_command(0x38);
    LCD_command(0x38);

    LCD_command(0x0C); //turn on cursor blink on
    LCD_command(0x06); //set auto increment right mode no shifting
    LCD_command(0x01); // go to first position
}

uint8_t LCD_set_pos(uint8_t x,uint8_t y){
    if (y == 1){
        x+=0x40;
    }
    __delay_ms(100);
    LCD_command(0x80 | x);
    
    return x;
}

static uint8_t pos_track = 0;
static uint8_t line_track = 0;

void LCD_print(uint8_t x){
    LCD_RS = 0;
    LCD_RS = 1;             // => RS = 1
    __delay_ms(50);
    LCD_DATA = x;
    LCD_EN = 0;
    LCD_EN = 1;
    for(int i=2130483; i<=0; i--)  NOP(); 
    LCD_EN = 0;
    pos_track++;
}


void LCD_print_string(const char *str){
    int i = 0;
    do{
        LCD_print(str[i]);
        i++;
    }while(str[i] != '_');
}

void reset_sim(void){ //infitinte loop when simulation ends
     LATDbits.LD0 =0; //clear police lights if reset flag is off
     LATDbits.LD1=0;
    if(reset_flag==1){ //flash lights if reset is on
        while(1){
     __delay_ms(85);
     LATDbits.LD0 =~LATDbits.LD0;
     __delay_ms(85);
     LATDbits.LD1=~LATDbits.LD1;
     __delay_ms(100);
     
     while(PORTCbits.RC4){ //reset the simulation once button is pressed
         light_value=0;
         speed_value=0;
         LATA=0x00;
         LATDbits.LD0 =0;
         LATDbits.LD1=0;
         LCD_command(0x01); //clear lcd on reset
         reset_flag=0;
         speed_count=0;
         count=1000;
         light_count=125;
         return;}
     }
        }
    
    else{
     LATDbits.LD0 =0;
     LATDbits.LD1=0;
     return;
    }
     
}
    

void stoplight(void){ //in charge of stoplight and speed limit checking
    if(flag ==1){
        if(light_count == 100){
            //led changes to green
           light_value=3; 
           LATA=0x04;
           
        }
         else if(light_count == 60){ //if counting
           //led changes to yellow  
            light_value=2;
            LATA=0x02;
           
        }
        else if(light_count== 40){
            //led changes to red
            light_value=1;
            LATA=0x01;
           
        }
        else if(light_count ==0){
            //led changes to green
            light_value =3;
            LATA=0x04;
            light_count =100;
          
        }
        else{ //when done set flag and reset timer flag
        count--;    
        //check speed
        }
       
       if(light_value==1){
        if(speed_value==0){//red car crashes when speed >0
            speed_count=0;
            reset_flag=0;
            reset_sim();
            
        }
        else if (speed_value==1){ //crash senerio
             LCD_set_pos(0,0);
            __delay_ms(200);   
            LCD_print_string("Car crashed_"); 
            reset_flag=1;
            reset_sim(); 
        }
        else if (speed_value==2){
             LCD_set_pos(0,0);
            __delay_ms(200);   
            LCD_print_string("Car crashed_"); 
            reset_flag=1;
            reset_sim(); 
        }
        else if (speed_value==3){
             LCD_set_pos(0,0);
            __delay_ms(200);   
            LCD_print_string("Car crashed_"); 
            reset_flag=1;
            reset_sim(); 
        }
        else if (speed_value==4){
             LCD_set_pos(0,0);
            __delay_ms(200);   
            LCD_print_string("Car crashed_"); 
            reset_flag=1;
            reset_sim(); 
        }
        else if (speed_value==5){
             LCD_set_pos(0,0);
            __delay_ms(200);   
            LCD_print_string("Car crashed_"); 
            reset_flag=1;
            reset_sim(); 
        }
    }
       else if(light_value==2){ //yellow pulled over >3
        if(speed_value==0){
            speed_count=0;
            reset_flag=0;
            reset_sim(); 
        }
        else if (speed_value==1){
            speed_count=0;
            reset_flag=0;
            reset_sim(); 
        }
        else if (speed_value==2){
            speed_count=0;
            reset_flag=0;
            reset_sim();  
        }
        else if (speed_value==3){
            speed_count=0;
            reset_flag=0;
            reset_sim();  
        }
        else if (speed_value==4){ //speed count up 1 
           speed_count++;
           
        }
        else if (speed_value==5){ //speed count up 2 for faster speed
           speed_count=speed_count +2;
           
        }
    }
       else if(light_value==3){ //green pulled over >3
        if(speed_value==0){
            speed_count=0;
            reset_flag=0;
            reset_sim(); 
        }
        else if (speed_value==1){
            speed_count=0;
            reset_flag=0;
            reset_sim();  
        }
        else if (speed_value==2){
            speed_count=0;
            reset_flag=0;
            reset_sim();  
        }
        else if (speed_value==3){
            speed_count=0;
            reset_flag=0;
            reset_sim(); 
        }
        else if (speed_value==4){
            speed_count++;
             
        }
        else if (speed_value==5){
            speed_count= speed_count +2;
           
        }
    }
        if(speed_count==40){ //if caught speeding when max speed_count is reached
           LCD_set_pos(0,0);
            __delay_ms(200);   
            LCD_print_string("Pulled over_"); 
            reset_flag=1;
            reset_sim(); 
        }
   
        count =1000; //resets count value that controls timing
        flag=0; //resets flag
        }
}


void main(void){
 
    OSCCON=0x73; //osciallator setup to 4MHz
    TRISD = 0x00; //set D as output
    TRISB = 0x00;
    TRISCbits.RC4=1;        //Set RC4 as input
    TRISCbits.RC3=1;        //RC3 as input
    TRISAbits.RA0=0;        //Set RA0 as output
    TRISAbits.RA1=0;        //Set RA0 as output
    TRISAbits.RA2=0;        //Set RA0 as output
    TRISDbits.RD4 = 0;      //RD4 as output
    TRISDbits.RD3 = 0;      //RD3 as output
    TRISDbits.RD2= 0;       //RD2 as output
    LATA = 0x00;       //all leds start off
    LCD_RS = 0; 
    LCD_RW = 0;
    LCD_EN = 0;
    LATD = 0x00;
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;// base interrupt setup
    LCD_init_8bits();   
    TRISAbits.RA4 = 1; //AN4 as input
    TMR0=0xBFA; //clear timer0
    T0CON=0x82; //sets bits for T0CON register 10000010
    ADCON0 = 0x01; //intialize ADC
    ADCON1 = 0x10; 
    ADCON2 = 0x00; 
    ADCON3 = 0xA0; 
    ADCHS = 0x01;
    ANSEL0 = 0x00;  
    ADRESH=0;          //reset values
    ADRESL=0; 
    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1; 
    INTCONbits.GIEL = 1; 
    INTCONbits.TMR0IE = 1; //enable timer0
    INTCON2bits.TMR0IP = 1;
    //LCD_set_pos(0,0);
      //      __delay_ms(200);   
        //    LCD_print_string("Push to start_"); was printing infinitly 
    while(PORTCbits.RC3){ //wait for start button     
    while(1){
    ADCON0bits.ACMOD0 = 0;
    ADCON0bits.ACMOD1 = 0; 
    ADCON0bits.GO = 1; 
    while(ADCON0bits.GO == 1); //gets pot value and converts
    pot_value = 0x00; //reset
    pot_value = ADRESH; //gets pot value
    voltage = pot_value / 51;  //255 max pot value / 5 potential options
        if (voltage < 1){ //if less then 1 volt
            LCD_set_pos(0,0);
            __delay_ms(200);   
            LCD_print_string("Speed: 0_");
            speed_value=0;
        }
        else if (voltage < 2){ //if less then 2 volts but more then 1
        
            LCD_set_pos(0,0);
            __delay_ms(200);   
            LCD_print_string("Speed: 1_");
            speed_value=1;
        }
        else if (voltage < 3){ //between 2 and 3
        
            LCD_set_pos(0,0);
            __delay_ms(200);   
            LCD_print_string("Speed: 2_");
            speed_value=2;
        }
        else if (voltage < 4){ //between 3 and 4
           
            LCD_set_pos(0,0);
            __delay_ms(200);   
            LCD_print_string("Speed: 3_");
            speed_value=3;
        }
        else if (voltage == 4){ //exactly 4
           
            LCD_set_pos(0,0);
            __delay_ms(200);   
            LCD_print_string("Speed: 4_");
            speed_value=4;
        }
        else { // max value
            LCD_set_pos(0,0);
            __delay_ms(200);   
            LCD_print_string("Speed: 5_");
            speed_value=5;
        }      
    } 
}
}


void __interrupt() high_isr(void){ 
    if(TMR0IF){
        if(count > 0){ //if counting
           count--;    
        }
        else{ //when done set flag and reset timer flag
        flag=1;    //this should happen every second to verify timing
        stoplight(); //calls stoplight and speed checker
        light_count--;
        TMR0=0x0BFA;
        TMR0IF=0;
        }
    }
}

void __interrupt(low_priority) low_isr(void){
    INTCONbits.GIEH = 0;

    if(0){

    }

    INTCONbits.GIEH = 1;
}



