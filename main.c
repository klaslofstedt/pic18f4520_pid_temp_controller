/*
 * File:   main.c
 * Author: KLALOF
 *
 * Created on 8 de junio de 2015, 21:30
 */
//#include <p18F4520.h>     // required by the C18 compiler
#include <xc.h>             // required by the XC8 compiler
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

unsigned int one_cycle = 100;
unsigned int current_cycle = 0;
unsigned int duty_cycle1 = 0; // PID heater. between 0 and 100
unsigned int duty_cycle2 = 100; // non PID heater. always 0 or 100

int desired_temp = 50; // between 0 and 200

// this enum makes the PID algorithm run at a certain speed (2 Hz)
typedef enum {IDLE, READING, READY} status;
status loop_status = IDLE;

typedef struct
{
     double d_state;        // last position input
     double i_state;        // integrator state
     double i_max, i_min;   // maximum and minimum allowable integrator state

     double k_p;            // proportional gain
     double k_i;            // integral gain
     double k_d;            // derivative gain
} PID;

// set the duty cycle of the both heaters
void pwm_set();
// returns the current temperature of the heaters depending on the in argument
double get_current_temp(unsigned char set_reg);
// self exlaining
void pwm_init();
void ADC_init();
void seg_init();
// set the temperature of a 7 segment display used for debugging
void seg_set(double i, double k);
// upadtes the pwm variables with new values
void pwm_update(int new_duty_cycle, double temp);
// the PID function that returns the new pwm for heater1
int pid_calc(PID* pid, double current_temp);


/***************** this block is required for the C18 compiler ****************/
/*void R_Int_Alta (void);

#pragma code Vector_Int_Alta=0x08 // High priority interrupt vector
void Int_Alta (void)
{
    _asm GOTO R_Int_Alta _endasm
}
#pragma code
#pragma interrupt R_Int_Alta // High priority interrupt routine


void R_Int_Alta (void)
{
    if (INTCONbits.TMR0IF == 1) // Check if interrupt is caused by timer 0
    {
        TMR0H = 0xEC;
        TMR0L = 0x77;

        pwm_set(); // Stupid way of generating a PWM, but it's simple

        // if the whole cycle is fulfilled, restart it
        if(current_cycle >= one_cycle){
            current_cycle = 0;
            if(loop_status == IDLE){
                loop_status = READY;
            }
        }
        current_cycle++;
        INTCONbits.TMR0IF = 0;
    }
}*/

/***************** this block is required for the XC8 compiler ****************/
void interrupt tc_int (void);

void interrupt tc_int (void)
{
    if (INTCONbits.TMR0IF == 1) // Check if interrupt is caused by timer 0
    {
        TMR0H = 0xEC;
        TMR0L = 0x77;

        pwm_set(); // Stupid way of generating a PWM, but it's simple

        // if the whole cycle is fulfilled, restart it
        if(current_cycle >= one_cycle){
            current_cycle = 0;
            if(loop_status == IDLE){
                loop_status = READY;
            }
        }
        current_cycle++;
        INTCONbits.TMR0IF = 0;
    }
}

void pwm_set()
{
    // It's not pretty but it works
    if(current_cycle <= duty_cycle1){
        PORTDbits.RD1 = 1;
    }
    else{
        PORTDbits.RD1 = 0;
    }
    if(current_cycle <= duty_cycle2){
        PORTDbits.RD2 = 1;
    }
    else{
        PORTDbits.RD2 = 0;
    }
}

double get_current_temp(unsigned char set_reg)
{
    // set either AN0 or AN1 active
    ADCON0 = set_reg;

    volatile int adc_read = 0;
    volatile double adc_voltage = 0;

    // Start conversion
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO == 1){
        // wait until bit is set to 0, which means conversion is finished
        // The result can then be read from ADRESH/L
    }
    adc_read = (ADRESH * 256) + ADRESL;
    // convert from analog to digital value
    adc_voltage = (double)adc_read/1024*5;

    // return the real temperature
    return adc_voltage*40;
}

void pwm_init()
{
    //timer on:               1
    //16-bit mode:            0
    //timer mode:             0
    //raising edge:           0
    //prescaler 1 (disabled): 1
    //presc = 2:              000 BUT it's disabled, so presc = 1.
    T0CON = 0x88; //1000 1000

    // T = (4/Fosc)*Presc*(resolution - preload)
    // Period time of 500ms, with increasements of 1%.
    // T = 500ms * 1% = 0.5 * 0.01 = 0.005.
    // 0.005 = (4/4000000)*1*(65535 - preload) -> preload = 60535
    // Set TMR0 to 60535 (dec) = EC77 (hex)
    TMR0H = 0xEC;
    TMR0L = 0x77;
}

void ADC_init()
{
    //Two empty bits:               00
    //AN0 selected:                 0000 by default
    //Conversion status: finished:  0
    //A/D converter ON              1
    ADCON0 = 0b00000001;
    //Two empty bits:               00
    //VREF- connected to VSS:       0
    //VREF+ connected to VSS:       0
    //pin RA0 & RA1 is analog:      1110
    ADCON1 = 0b00001101;

    //ADC Result Right Justified:   1
    //Empty bit:                    0
    //Acquisition Time = 20TAD:     111
    //Conversion Clock = Fosc / 4:  100
    ADCON2 = 0b10111100;
}

void seg_set(double i, double k)
{

    int temp = (int)round(k);
    int output;
    output = (temp/10 <<4)+ temp % 10 ;
    PORTB = output;

    int temp1 = (int)round(i);
    int output1;
    output1 = (temp1/10 <<4)+ temp1 % 10 ;

    PORTC = output1;
}

void seg_init()
{
    TRISB = 0x00;
    TRISC = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
}

void pwm_update(int new_duty_cycle, double temp)
{
    // convert from double to int
    int current_temp = (int)round(temp);
    // set duty cycle either 0 or 100 on heater 2
    if(current_temp > desired_temp){
        duty_cycle2 = 0;
    }else{
        duty_cycle2 = 100;
    }
    // set duty cycle to something between 0 and 100 on heater 1
    duty_cycle1 = new_duty_cycle;
}

int pid_calc(PID* pid, double current_temp)
{
    double error = desired_temp - current_temp;
    double p_term, d_term, i_term;

    // calculate the proportional term
    p_term = pid->k_p * error;

    // calculate the integral state
    pid->i_state += error;
    if (pid->i_state > pid->i_max){
        pid->i_state = pid->i_max;
    }
    // check limits
    else if (pid->i_state < pid->i_min){
        pid->i_state = pid->i_min;
    }
    // calculate the integral term
    i_term = pid->k_i * pid->i_state;
    // calculate the derivative term
    d_term = pid->k_d * (current_temp - pid->d_state);
    pid->d_state = current_temp;

    // set the output value within limits (0 to 100)
    double value = (p_term + i_term - d_term);
    if(value > pid->i_max){
        value = pid->i_max;
    }
    else if(value < pid->i_min){
        value = pid->i_min;
    }
    return (int)round(value);
}

void main(void)
{
    pwm_init();
    ADC_init();
    seg_init();

    TRISD = 0xF8; // Set outputs for the heaters and a LED

    INTCONbits.GIE = 1; // Interrupts are enabled globally
    INTCONbits.TMR0IE = 1; // Timer 0 interrupt is enabled

    while(1)
    {
        // Set PID variables
        PID tempPID;
        tempPID.k_p = 10;
        tempPID.k_i = 0.01;
        tempPID.k_d = 80;
        tempPID.i_max = 100;
        tempPID.i_min = 0;

        // this loop is running at 2Hz
        if(loop_status == READY){
            INTCONbits.GIE = 0; // Interrupts are disabled globally
            INTCONbits.TMR0IE = 0; // Timer 0 interrupt is disabled
            loop_status = READING;

            PORTDbits.RD0 ^= 1; // visualize the frequency

            // read the temperature for the PID heater
            double current_temp1 = get_current_temp(0b00000001);
            // read the temperature for the non PID heater
            double current_temp2 = get_current_temp(0b00000101);
            // display current temperatures
            seg_set(current_temp1, current_temp2);
            // read new pwm for the PID heater from the PID function
            int new_pwm = pid_calc(&tempPID, current_temp1);
            // update the pwm of both heaters
            pwm_update(new_pwm, current_temp2);

            loop_status = IDLE;
            INTCONbits.GIE = 1; // Interrupts are enabled globally
            INTCONbits.TMR0IE = 1; // Timer 0 interrupt is enabled
        }
    }
}
