/*Author: Rajat Yadav
 Semester Project*/

#include <avr/io.h>     //AVR library
#include <util/delay.h> //Delay library
#include "USART.h"      //USART library
#include <util/setbaud.h> //setbaud library

void PWM(void) {   //PWM initialization
    //Setting the correct wave form generation mode (WGM) for fast PWM, 8-bit
    
    TCCR1A |= (1<<WGM10);   //Setting WGM10 as 1 for timer/counter control register A
    TCCR1A &= ~(1<<WGM11);  //Setting WGM11 as 0
    TCCR1B |= (1<<WGM12);   //Setting WGM12 as 1
    TCCR1B &= ~(1<<WGM13);  //Setting WGM13 as 0
    
    //Setting the PWM frequency via scaler
    
    TCCR1B |=(1<<CS11);     //Seting CS11 bit of TCCR1B as 1
    TCCR1B &=~((1<<CS12) | (1<<CS10)); //Setting CS12 and CS 10 as 0
    
    //Setting which pins are used
    TCCR1A |= (1<<COM1A1);  //Seting COM1A1 for PWM under PB2
    TCCR1A &= ~(1<<COM1A0); //Seting COM1A0 for PWM under PB2
    TCCR1A |= (1<<COM1B1);  //Seting COM1B1 for PWM under PB2
    TCCR1A &= ~(1<<COM1B0); //Seting COM1B0 for PWM under PB2
}

void ADC_init(void) {                   //ADC initialization
    
    // Set reference voltage to AVCC
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);

    // Set clock prescaler (PS) to 1/8th
    ADCSRA |= (1 << ADPS0) | (1 << ADPS1);
    ADCSRA &= ~(1 << ADPS2);

    // Enables ADC
    ADCSRA |= (1 << ADEN);
    
}

void initUSART(void) {                      //USART initialization
    // Set baud rate
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    // Adjust baud rate for CPU clock speed
  #if USE_2X
    UCSR0A |= (1 << U2X0);
  #else
    UCSR0A &= ~(1 << U2X0);
  #endif

    // Enable USART transmitter/receiver
    UCSR0B |= (1 << TXEN0) | (1 << RXEN0);

    // Set packet size to 8 bits
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
    UCSR0B &=  ~(1 << UCSZ02);

    // Set stop bit (1)
    UCSR0C &= ~(1 << USBS0);
}

int main(void) {                            //Main function
    
    
    DDRC |=(1<<PC1);     //PC1 Initialisation
    DDRC &= ~(1 << PC2); //PC2 Initialisation
    DDRC |=(1<<PC5);     //PC5 initialization
    DDRC |=(1<<PC4);     //PC4 initialization
    DDRB |=(1<<PB1);     //PB1 Initialisation
    DDRD |=(1<<PD6);     //PD6 Initialisation
    DDRD |=(1<<PD7);     //PD7 Initialisation
    DDRB |=(1<<PB0);     //PB0 Initialisation
    
    PWM();          //Calling PWM initialization function
    ADC_init();     //Calling ADC initialization function
    initUSART();        //Calling USART initialization function
    
    
    while (1) {         //Event loop
        
        
        ADMUX |= (1 << MUX0);                                   //Setting ADC0 as input for notifications
        ADMUX &= ~((1 << MUX0) | (1 << MUX2) | (1 << MUX3));    //Setting ADC0 as input for notifications
        ADCSRA |= (1 << ADSC);                                  // Start conversion
        while((ADCSRA & (1 << ADSC)) != 0){}                    // Busy loop
        uint16_t adc_result = ADC;                              // Get ADC result
        printWord(adc_result);                                  //Output the value to light sensor
        
        ADMUX |= (1 << MUX1);                                   //Setting ADC1 as input
        ADMUX &= ~((1 << MUX0) | (1 << MUX2) | (1 << MUX3));    //Setting ADC1 as input
        ADCSRA |= (1 << ADSC);                                  // Start conversion
        while((ADCSRA & (1 << ADSC)) != 0){}                    // Busy loop
        uint16_t light = ADC;                                   //Storing the value of ADC in variable - light
        
        //Sensor MIN and MAX values collected in previous assignments
        
        if(light>450 && light<730) {            //Safe brightness values: GREEN LIGHT
            PORTD = 0b0;
            PORTB |= (1<<PB0);                  //Turn on green LED
        }
        
        else if(light>=350 && light<=450) {      //Moderately unsafe brightness values: Yellow light with normal buzzer sound
            PORTD = (0<<PD6) | (1<<PD7);        //Turn on yellow LED
            PORTB = 0b0;
            for(int i = 0; i<256; i++) {        //Buzzer sound loop
                OCR1A = i;
                _delay_ms(10);
            }                                   //Loop end
        }
        
        else if(light>=730 && light<900) {      //Moderately unsafe brightness values: Yellow light with normal buzzer sound
            PORTD = (0<<PD6) | (1<<PD7);
            PORTB = 0b0;                        //Turn on yellow LED
            for(int i = 0; i<256; i++) {        //Buzzer sound loop
                OCR1A = i;
                _delay_ms(10);
            }                                  //Loop end
        }
        
        
        else {                                //Highly unsafe brightness values: Red light with fast buzzer sound
            PORTD = 00000100;                 //Turn on red LED
            PORTB = 0b0;
            for(int i = 0; i<256; i++) {     //Buzzer sound loop
                OCR1A = i;
                _delay_ms(1);
            }                                //Loop end
        }

        ADMUX |= (1 << MUX0);                                //Setting ADC0 as input
        ADMUX &= ~((1 << MUX0) | (1 << MUX2) | (1 << MUX3)); //Setting ADC0 as input
        ADCSRA |= (1 << ADSC);                               // Start conversion
        while((ADCSRA & (1 << ADSC)) != 0){}                 // Busy loop
        uint16_t temperature = ADC;                          //Store ADC value
        
        if(temperature<150) {            //unsafe temprature values: WHITE LIGHT
            PORTC |= (1<<PC4);                              //Turn on white LED
        }
        
        
        else {                              //Safe temperature values: Blue light
            PORTC = (0<<PC4)|(1<<PC5);      //Turn on Blue LED
            }
        
        //Further slack notification implementation in the python program
       
    }       //Event loop ends
    
    return 0;   //Line never reached
    
}               //End of main()
