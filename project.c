//Ulysses Chaparro 1001718774

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "eeprom.h"
#include "nvic.h"
#include "lab5_Emb1.h"

/*
  sensor1    sensor2
  (0,0)      (x,y)

  sensor0
  (x,y)
 */

#define SPKR_OUT PORTB,4
#define IR_IN PORTE,3

#define S0_SIGNAL PORTC,6                       //PC6 pin (sensor 0 signal) using function WT1CCP0
#define S1_SIGNAL PORTD,0                       //PD0 pin (sensor 1 signal) using function WT2CCP0
#define S2_SIGNAL PORTD,2                       //PD2 pin (sensor 2 signal) using function WT3CCP0

#define TRIGGER_LIMIT 1                         //used to control the # of times each sensor timer triggers

uint8_t count_s0, count_s1, count_s2;           //counter for each sensor timer trigger

uint8_t hit_s0, hit_s1, hit_s2;                 //flags to know if sensor edges arrived

int8_t sensor_num;                              //used to check for valid sensor (either 0, 1, or 2)
int16_t x_input, y_input;                       //user input (x,y) for sensor

uint16_t sensor0_x, sensor0_y;                  //variables to save (x,y) of each sensor to memory
uint16_t sensor1_x, sensor1_y;
uint16_t sensor2_x, sensor2_y;

float time_s0 = 0;                              //times from each sensor, used to capture distance from each sensor
float time_s1 = 0;
float time_s2 = 0;

float distance_s0 = 0;
float distance_s1 = 0;
float distance_s2 = 0;

uint8_t se2c, tri;                            //determine which method to use
uint8_t distance_on;                            //flag to show distance from each sensor if on
uint8_t fix;
uint32_t x_fix, y_fix;

char str_s0[20];                                //used for printing distances from each sensor
char str_s1[20];
char str_s2[20];

uint8_t N, N_count, N_reached;                  //for average N
float sum_s0 = 0;
float sum_s1 = 0;
float sum_s2 = 0;
float mean_distance_s0, mean_distance_s1, mean_distance_s2;

uint8_t V, variance_flag;                       //for variance V
uint16_t sample_read;
float sum0_variance = 0;
float sum1_variance = 0;
float sum2_variance = 0;
float variance_s0, variance_s1, variance_s2;

uint16_t k0_input, k1_input;                    //for correction k0 and k1 for each sensor
uint16_t sensor0_k0, sensor1_k0, sensor2_k0;
uint16_t sensor0_k1, sensor1_k1, sensor2_k1;

uint8_t ir_beep_flag;                           //beep flags to know what beeps to play
uint8_t error_beep2_flag, error_beep3_flag;
uint8_t hit2_flag, hit3_flag;
uint8_t fix_beep_flag;

uint8_t ir_beep_on;                             //example: if beep ir ON -> ir_beep_on = 1. if beep ir OFF -> ir_beep_on = 0
uint8_t error_beep2_on, error_beep3_on;
uint8_t beep2_on, beep3_on;
uint8_t fix_beep_on;

typedef struct BEEP_CONFIG
        {
            uint16_t frequency;
            uint16_t duration;
        } BEEP_CONFIG;

BEEP_CONFIG ir_beep;
BEEP_CONFIG error_beep;
BEEP_CONFIG hit2_beep;
BEEP_CONFIG hit3_beep;
BEEP_CONFIG fix_beep;
BEEP_CONFIG blank_beep;

#define TIMES_FIFO_LENGTH 9
uint32_t wideTimer1Fifo[TIMES_FIFO_LENGTH];     //fifo used to store times from wideTimer1
uint8_t wideTimer1ReadIndex = 0;
uint8_t wideTimer1WriteIndex = 0;

uint32_t wideTimer2Fifo[TIMES_FIFO_LENGTH];     //fifo used to store times from wideTimer2
uint8_t wideTimer2ReadIndex = 0;
uint8_t wideTimer2WriteIndex = 0;

uint32_t wideTimer3Fifo[TIMES_FIFO_LENGTH];     //fifo used to store times from wideTimer3
uint8_t wideTimer3ReadIndex = 0;
uint8_t wideTimer3WriteIndex = 0;

#define SAMPLE_FIFO_LENGTH 9
uint16_t sample0Fifo[SAMPLE_FIFO_LENGTH];       //fifo used to store distance samples from sensor 0 (used for variance calculation)
uint8_t sample0WriteIndex = 0;
uint8_t sample0ReadIndex = 0;

uint16_t sample1Fifo[SAMPLE_FIFO_LENGTH];       //fifo used to store distance samples from sensor 1 (used for variance calculation)
uint8_t sample1WriteIndex = 0;
uint8_t sample1ReadIndex = 0;

uint16_t sample2Fifo[SAMPLE_FIFO_LENGTH];       //fifo used to store distance samples from sensor 2 (used for variance calculation)
uint8_t sample2WriteIndex = 0;
uint8_t sample2ReadIndex = 0;

void initHw(void)
{
    initSystemClockTo40Mhz();

    //enable clocks
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R2 | SYSCTL_RCGCWTIMER_R3 | SYSCTL_RCGCWTIMER_R4 | SYSCTL_RCGCWTIMER_R5;
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    _delay_cycles(3);

    enablePort(PORTB);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);

    //setting up PB4 pin to function as M0PWM2
    setPinAuxFunction(SPKR_OUT,4);
    selectPinPushPullOutput(SPKR_OUT);
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;           // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                         // leave reset state
    PWM0_1_CTL_R = 0;                           // turn-off PWM0 generator 2
    PWM0_1_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO;
    PWM0_1_CMPA_R = 0;

    //setting up PC6, PD0, PD2 pins to function as WT1CCP0, WT2CCP0, WT3CCP0
    setPinAuxFunction(S0_SIGNAL,7);
    selectPinDigitalInput(S0_SIGNAL);
    setPinAuxFunction(S1_SIGNAL,7);
    selectPinDigitalInput(S1_SIGNAL);
    setPinAuxFunction(S2_SIGNAL,7);
    selectPinDigitalInput(S2_SIGNAL);

    //setting up IR gpio interrupt
    selectPinDigitalInput(IR_IN);
    disablePinInterrupt(IR_IN);
    disableNvicInterrupt(INT_GPIOE);
    selectPinInterruptFallingEdge(IR_IN);
    enableNvicInterrupt(INT_GPIOE);
    enablePinInterrupt(IR_IN);
}

void configWideTimers(void)
{
    //configure wide timer 1 for sensor 0
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;           //turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                          //configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;    //configure for edge time mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_NEG;      //measure time from positive edge to negative edge
    WTIMER1_IMR_R = TIMER_IMR_CAEIM;            //turn-on interrupts
    enableNvicInterrupt(INT_WTIMER1A);          //turn-on interrupt 112 (WTIMER1A)

    //configure wide timer 2 for sensor 1
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;           //turn-off counter before reconfiguring
    WTIMER2_CFG_R = 4;                          //configure as 32-bit counter (A only)
    WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;    //configure for edge time mode, count up
    WTIMER2_CTL_R = TIMER_CTL_TAEVENT_NEG;      //measure time from positive edge to negative edge
    WTIMER2_IMR_R = TIMER_IMR_CAEIM;            //turn-on interrupts
    enableNvicInterrupt(INT_WTIMER2A);          //turn-on interrupt 114 (WTIMER2A)

    //configure wide timer 3 for sensor 2
    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;           //turn-off counter before reconfiguring
    WTIMER3_CFG_R = 4;                          //configure as 32-bit counter (A only)
    WTIMER3_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;    //configure for edge time mode, count up
    WTIMER3_CTL_R = TIMER_CTL_TAEVENT_NEG;      //measure time from positive edge to negative edge
    WTIMER3_IMR_R = TIMER_IMR_CAEIM;            //turn-on interrupts
    enableNvicInterrupt(INT_WTIMER3A);          //turn-on interrupt 116 (WTIMER3A)

    //configure wide timer 4 for timeout
    WTIMER4_CTL_R &= ~TIMER_CTL_TAEN;           //turn-off counter before reconfiguring
    WTIMER4_CFG_R = 4;                          //configure as 32-bit counter (A only)
    WTIMER4_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;    //configure for one shot timer mode, count down
    WTIMER4_IMR_R = TIMER_IMR_TATOIM;           //turn-on interrupt
    enableNvicInterrupt(INT_WTIMER4A);          //turn-on interrupt 118 (WTIMER4A)

    //configure wide timer 5 for beep duration
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;           //turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                          //configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;    //configure for one shot timer mode, count down
    WTIMER5_IMR_R = TIMER_IMR_TATOIM;           //turn-on interrupt
    enableNvicInterrupt(INT_WTIMER5A);          //turn-on interrupt 120 (WTIMER5A)
}

void retrieveEeprom(void)
{
    uint32_t temp;

    temp = readEeprom(16);                      //reads block 1, offset 0 (sensor0_x)
    if(temp != 0xFFFFFFFF)
    {
        sensor0_x = temp;
    }
    temp = readEeprom(17);                      //reads block 1, offset 1 (sensor0_y)
    if(temp != 0xFFFFFFFF)
    {
        sensor0_y = temp;
    }
    temp = readEeprom(18);                      //reads block 1, offset 2 (sensor1_x)
    if(temp != 0xFFFFFFFF)
    {
        sensor1_x = temp;
    }
    temp = readEeprom(19);                      //reads block 1, offset 3 (sensor1_y)
    if(temp != 0xFFFFFFFF)
    {
        sensor1_y = temp;
    }
    temp = readEeprom(20);                      //reads block 1, offset 4 (sensor2_x)
    if(temp != 0xFFFFFFFF)
    {
        sensor2_x = temp;
    }
    temp = readEeprom(21);                      //reads block 1, offset 5 (sensor2_y)
    if(temp != 0xFFFFFFFF)
    {
        sensor2_y = temp;
    }
    temp = readEeprom(22);                      //reads block 1, offset 6 (ir_beep_on)
    if(temp != 0xFFFFFFFF)
    {
        ir_beep_on = temp;
    }
    temp = readEeprom(23);                      //reads block 1, offset 7 (ir_beep.frequency)
    if(temp != 0xFFFFFFFF)
    {
        ir_beep.frequency = temp;
    }
    temp = readEeprom(24);                      //reads block 1, offset 8 (ir_beep.duration)
    if(temp != 0xFFFFFFFF)
    {
        ir_beep.duration = temp;
    }
    temp = readEeprom(25);                      //reads block 1, offset 9 (error_beep2_on)
    if(temp != 0xFFFFFFFF)
    {
        error_beep2_on = temp;
    }
    temp = readEeprom(26);                      //reads block 1, offset 10 (error_beep3_on)
    if(temp != 0xFFFFFFFF)
    {
        error_beep3_on = temp;
    }
    temp = readEeprom(27);                      //reads block 1, offset 11 (error_beep3_on)
    if(temp != 0xFFFFFFFF)
    {
        error_beep.frequency = temp;
    }
    temp = readEeprom(28);                      //reads block 1, offset 12 (error_beep.frequency)
    if(temp != 0xFFFFFFFF)
    {
        error_beep.duration = temp;
    }
    temp = readEeprom(29);                      //reads block 1, offset 13 (beep2_on.duration)
    if(temp != 0xFFFFFFFF)
    {
        beep2_on = temp;
    }
    temp = readEeprom(30);                      //reads block 1, offset 14 (hit2_beep.frequency)
    if(temp != 0xFFFFFFFF)
    {
        hit2_beep.frequency = temp;
    }
    temp = readEeprom(31);                      //reads block 1, offset 15 (hit2_beep.duration)
    if(temp != 0xFFFFFFFF)
    {
        hit2_beep.duration = temp;
    }
    temp = readEeprom(32);                      //reads block 2, offset 0 (beep3_on)
    if(temp != 0xFFFFFFFF)
    {
        beep3_on = temp;
    }
    temp = readEeprom(33);                      //reads block 2, offset 1 (hit3_beep.frequency)
    if(temp != 0xFFFFFFFF)
    {
        hit3_beep.frequency = temp;
    }
    temp = readEeprom(34);                      //reads block 2, offset 2 (hit3_beep.duration)
    if(temp != 0xFFFFFFFF)
    {
        hit3_beep.duration = temp;
    }
    temp = readEeprom(35);                      //reads block 2, offset 3 (distance_on)
    if(temp != 0xFFFFFFFF)
    {
        distance_on = temp;
    }
    temp = readEeprom(36);                      //reads block 2, offset 4 (N)
    if(temp != 0xFFFFFFFF)
    {
        N = temp;
    }
    temp = readEeprom(37);                      //reads block 2, offset 5 (V)
    if(temp != 0xFFFFFFFF)
    {
        V = temp;
    }
    temp = readEeprom(38);                      //reads block 2, offset 6 (fix_beep_on)
    if(temp != 0xFFFFFFFF)
    {
        fix_beep_on = temp;
    }
    temp = readEeprom(39);                      //reads block 2, offset 7 (fix_beep.frequency)
    if(temp != 0xFFFFFFFF)
    {
        fix_beep.frequency = temp;
    }
    temp = readEeprom(40);                      //reads block 2, offset 8 (fix_beep.duration)
    if(temp != 0xFFFFFFFF)
    {
        fix_beep.duration = temp;
    }
    temp = readEeprom(41);                      //reads block 2, offset 9 (sensor0_k0)
    if(temp != 0xFFFFFFFF)
    {
        sensor0_k0 = temp;
    }
    temp = readEeprom(42);                      //reads block 2, offset 10 (sensor0_k1)
    if(temp != 0xFFFFFFFF)
    {
        sensor0_k1 = temp;
    }
    temp = readEeprom(43);                      //reads block 2, offset 11 (sensor1_k0)
    if(temp != 0xFFFFFFFF)
    {
        sensor1_k0 = temp;
    }
    temp = readEeprom(44);                      //reads block 2, offset 12 (sensor1_k1)
    if(temp != 0xFFFFFFFF)
    {
        sensor1_k1 = temp;
    }
    temp = readEeprom(45);                      //reads block 2, offset 13 (sensor2_k0)
    if(temp != 0xFFFFFFFF)
    {
        sensor2_k0 = temp;
    }
    temp = readEeprom(46);                      //reads block 2, offset 14 (sensor2_k1)
    if(temp != 0xFFFFFFFF)
    {
        sensor2_k1 = temp;
    }
    temp = readEeprom(47);                      //reads block 2, offset 15 (se2c)
    if(temp != 0xFFFFFFFF)
    {
        se2c = temp;
    }
    temp = readEeprom(48);                      //reads block 3, offset 0 (tri)
    if(temp != 0xFFFFFFFF)
    {
        tri = temp;
    }
}

void storeTime(uint32_t time, uint8_t fifo_ID)
{
    bool full;
    switch(fifo_ID)
    {
        case 1:                                 //if wide timer 1 fifo
            full = ((wideTimer1WriteIndex+1) % TIMES_FIFO_LENGTH) == wideTimer1ReadIndex;
            if (!full)
            {
                wideTimer1Fifo[wideTimer1WriteIndex] = time;
                wideTimer1WriteIndex = (wideTimer1WriteIndex + 1) % TIMES_FIFO_LENGTH;
            }
            break;
        case 2:                                 //if wide timer 2 fifo
            full = ((wideTimer2WriteIndex+1) % TIMES_FIFO_LENGTH) == wideTimer2ReadIndex;
            if (!full)
            {
                wideTimer2Fifo[wideTimer2WriteIndex] = time;
                wideTimer2WriteIndex = (wideTimer2WriteIndex + 1) % TIMES_FIFO_LENGTH;
            }
            break;
        case 3:                                 //if wide timer 3 fifo
            full = ((wideTimer3WriteIndex+1) % TIMES_FIFO_LENGTH) == wideTimer3ReadIndex;
            if (!full)
            {
                wideTimer3Fifo[wideTimer3WriteIndex] = time;
                wideTimer3WriteIndex = (wideTimer3WriteIndex + 1) % TIMES_FIFO_LENGTH;
            }
            break;
    }
}

uint32_t readTime(uint8_t fifo_ID)
{
    uint32_t time;
    switch(fifo_ID)
    {
        case 1:                                 //if wide timer 1 fifo
            time = wideTimer1Fifo[wideTimer1ReadIndex];
            wideTimer1ReadIndex = (wideTimer1ReadIndex + 1) % TIMES_FIFO_LENGTH;
            break;
        case 2:                                 //if wide timer 2 fifo
            time = wideTimer2Fifo[wideTimer2ReadIndex];
            wideTimer2ReadIndex = (wideTimer2ReadIndex + 1) % TIMES_FIFO_LENGTH;
            break;
        case 3:                                 //if wide timer 3 fifo
            time = wideTimer3Fifo[wideTimer3ReadIndex];
            wideTimer3ReadIndex = (wideTimer3ReadIndex + 1) % TIMES_FIFO_LENGTH;
            break;
    }
    return time;
}

void storeSample(uint16_t sample, uint8_t fifo_ID)
{
    bool full;
    switch(fifo_ID)
    {
        case 0:                                 //if sensor 0 samples fifo
            full = ((sample0WriteIndex+1) % SAMPLE_FIFO_LENGTH) == sample0ReadIndex;
            if (!full)
            {
                sample0Fifo[sample0WriteIndex] = sample;
                sample0WriteIndex = (sample0WriteIndex + 1) % SAMPLE_FIFO_LENGTH;
            }
            break;
        case 1:                                 //if sensor 1 samples fifo
            full = ((sample1WriteIndex+1) % SAMPLE_FIFO_LENGTH) == sample1ReadIndex;
            if (!full)
            {
                sample1Fifo[sample1WriteIndex] = sample;
                sample1WriteIndex = (sample1WriteIndex + 1) % SAMPLE_FIFO_LENGTH;
            }
            break;
        case 2:                                 //if sensor 2 samples fifo
            full = ((sample2WriteIndex+1) % SAMPLE_FIFO_LENGTH) == sample2ReadIndex;
            if (!full)
            {
                sample2Fifo[sample2WriteIndex] = sample;
                sample2WriteIndex = (sample2WriteIndex + 1) % SAMPLE_FIFO_LENGTH;
            }
            break;
    }
}

uint16_t readSample(uint8_t fifo_ID)
{
    uint16_t sample;
    switch(fifo_ID)
    {
        case 0:                                 //if sensor 0 samples fifo
            sample = sample0Fifo[sample0ReadIndex];
            sample0ReadIndex = (sample0ReadIndex + 1) % SAMPLE_FIFO_LENGTH;
            break;
        case 1:                                 //if sensor 1 samples fifo
            sample = sample1Fifo[sample1ReadIndex];
            sample1ReadIndex = (sample1ReadIndex + 1) % SAMPLE_FIFO_LENGTH;
            break;
        case 2:                                 //if sensor 2 samples fifo
            sample = sample2Fifo[sample2ReadIndex];
            sample2ReadIndex = (sample2ReadIndex + 1) % SAMPLE_FIFO_LENGTH;
            break;
    }
    return sample;
}

void playBeep(BEEP_CONFIG *beep)
{
    if(beep->frequency)                         //if not blank_beep (because blank_beep has frequency as 0)
    {
        PWM0_1_LOAD_R = 20000000 / beep->frequency;
        PWM0_1_CMPA_R = PWM0_1_LOAD_R / 2;

        PWM0_1_CTL_R = PWM_0_CTL_ENABLE;        // turn-on PWM0 generator 0
        PWM0_ENABLE_R = PWM_ENABLE_PWM2EN;      // enable outputs
    }

    WTIMER5_TAILR_R = 4000000 * beep->duration;
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;            //turn on wide timer 5 for beep duration
}

void wideTimer1Isr(void)
{
    if(!(count_s0 == TRIGGER_LIMIT))            //if true, keep capturing and storing time
    {
        storeTime(WTIMER1_TAV_R, 1);            //store time into wideTimer1Fifo
        //putsUart0("0\n");
        count_s0++;

        hit_s0 = 1;
        WTIMER1_TAV_R = 0;                      //zero counter for next edge
    }
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;          //clear interrupt flag
}

void wideTimer2Isr(void)
{
    if(!(count_s1 == TRIGGER_LIMIT))            //if true, keep capturing and storing time
    {
        storeTime(WTIMER2_TAV_R, 2);            //store time into wideTimer2Fifo
        //putsUart0("1\n");
        count_s1++;

        hit_s1 = 1;
        WTIMER2_TAV_R = 0;                      //zero counter for next edge
    }
    WTIMER2_ICR_R = TIMER_ICR_CAECINT;          //clear interrupt flag
}

void wideTimer3Isr(void)
{
    if(!(count_s2 == TRIGGER_LIMIT))            //if true, keep capturing and storing time
    {
        storeTime(WTIMER3_TAV_R, 3);            //store time into wideTimer3Fifo
        //putsUart0("2\n");
        count_s2++;

        hit_s2 = 1;
        WTIMER3_TAV_R = 0;                      //zero counter for next edge
    }
    WTIMER3_ICR_R = TIMER_ICR_CAECINT;          //clear interrupt flag
}

void wideTimer4Isr(void)
{
    //putsUart0("Reset\n");

    if(hit_s0 & hit_s1 || hit_s0 & hit_s2 || hit_s1 & hit_s2)   //set hit2_flag if 2 edges arrived, else set error_beep2_flag
    {
        if(beep2_on) hit2_flag = 1;
    }
    else if(error_beep2_on) error_beep2_flag = 1;

    if(hit_s0 & hit_s1 & hit_s2)                                //set hit3_flag if 3 edges arrived, else set error_beep3_flag
    {
        if(beep3_on) hit3_flag = 1;
    }
    else if(error_beep3_on) error_beep3_flag = 1;

    //play beeps
    if(ir_beep_flag)
    {
        //putsUart0("ir_beep\n");
        playBeep(&ir_beep);                     //priming the pump to play any remaining beeps in the 5th ISR
        ir_beep_flag = 0;
    }
    else
    {
        playBeep(&blank_beep);                  //dummy beep to prime the pump to play remaining beeps
    }

    if(N_count < N)
    {
        time_s0 = readTime(1);                  //read time from wide timer 1 to calculate distance from sensor 0
        distance_s0 = (time_s0 / 40000000) * 343000;
        sum_s0 += distance_s0;
        storeSample(distance_s0, 0);

        time_s1 = readTime(2);                  //read time from wide timer 2 to calculate distance from sensor 1
        distance_s1 = (time_s1 / 40000000) * 343000;
        sum_s1 += distance_s1;
        storeSample(distance_s1, 1);

        time_s2 = readTime(3);                  //read time from wide timer 3 to calculate distance from sensor 2
        distance_s2 = (time_s2 / 40000000) * 343000;
        sum_s2 += distance_s2;
        storeSample(distance_s2, 2);

        N_count++;
    }

    if(N_count == N)
    {
        mean_distance_s0 = sum_s0 / N;          //calculate averages
        mean_distance_s1 = sum_s1 / N;
        mean_distance_s2 = sum_s2 / N;

        int i;                                  //calculate variance
        for(i = 0; i < N; i++)
        {
            sample_read = readSample(0);
            sum0_variance += (sample_read - mean_distance_s0) * (sample_read - mean_distance_s0);
        }

        for(i = 0; i < N; i++)
        {
            sample_read = readSample(1);
            sum1_variance += (sample_read - mean_distance_s1) * (sample_read - mean_distance_s1);
        }

        for(i = 0; i < N; i++)
        {
            sample_read = readSample(2);
            sum2_variance += (sample_read - mean_distance_s2) * (sample_read - mean_distance_s2);
        }

        variance_s0 = sqrt(sum0_variance / N);  //calculate variances
        variance_s1 = sqrt(sum1_variance / N);
        variance_s2 = sqrt(sum2_variance / N);

        //setting variance_flag if any of the variances are greater than V
        if(variance_s0 > V || variance_s1 > V || variance_s2 > V)   variance_flag = 1;

        if(!variance_flag & !error_beep2_flag & !error_beep3_flag)
        {
            fix = 1;
            if(fix_beep_on)
            {
                fix_beep_flag = 1;
            }
        }

        N_count = 0;
        N_reached = 1;
    }

    if(distance_on & !variance_flag)
    {
        if(!hit_s0)
        {
            putsUart0("sensor 0 raw time: \t---\n");
            putsUart0("sensor 0 time: \t\t---\n");
            putsUart0("sensor 0 distance: \t---\n\n");
        }
        else
        {
            putsUart0("sensor 0 raw time: ");
            sprintf(str_s0, "\t%.0f", time_s0);
            putsUart0(str_s0);
            putsUart0("\n");
            putsUart0("sensor 0 time: ");
            sprintf(str_s0, "\t\t%.2f", time_s0 / 40); //us
            putsUart0(str_s0);
            putsUart0(" (us)\n");
            putsUart0("sensor 0 distance: ");
            sprintf(str_s0, "\t%.2f", distance_s0);
            putsUart0(str_s0);
            putsUart0(" (mm)\n\n");
        }

        if(!hit_s1)
        {
            putsUart0("sensor 1 raw time: \t---\n");
            putsUart0("sensor 1 time: \t\t---\n");
            putsUart0("sensor 1 distance: \t---\n\n");
        }
        else
        {
            putsUart0("sensor 1 raw time: ");
            sprintf(str_s1, "\t%.0f", time_s1);
            putsUart0(str_s1);
            putsUart0("\n");
            putsUart0("sensor 1 time: ");
            sprintf(str_s1, "\t\t%.2f", time_s1 / 40); //us
            putsUart0(str_s1);
            putsUart0(" (us)\n");
            putsUart0("sensor 1 distance: ");
            sprintf(str_s1, "\t%.2f", distance_s1);
            putsUart0(str_s1);
            putsUart0(" (mm)\n\n");
        }

        if(!hit_s2)
        {
            putsUart0("sensor 2 raw time: \t---\n");
            putsUart0("sensor 2 time: \t\t---\n");
            putsUart0("sensor 2 distance: \t---\n\n");
        }
        else
        {
            putsUart0("sensor 2 raw time: ");
            sprintf(str_s2, "\t%.0f", time_s2);
            putsUart0(str_s2);
            putsUart0("\n");
            putsUart0("sensor 2 time: ");
            sprintf(str_s2, "\t\t%.2f", time_s2 / 40); //us
            putsUart0(str_s2);
            putsUart0(" (us)\n");
            putsUart0("sensor 2 distance: ");
            sprintf(str_s2, "\t%.2f", distance_s2);
            putsUart0(str_s2);
            putsUart0(" (mm)\n\n----------------------------------------\n\n");
        }
    }

    if(N_reached)
    {
        if(distance_on)
        {
            putsUart0("N = \t");
            sprintf(str_s0, "%u", N);
            putsUart0(str_s0);
            putsUart0("\n");
            putsUart0("V = \t");
            sprintf(str_s1, "%u", V);
            putsUart0(str_s1);
            putsUart0("\n");

            putsUart0("sensor 0 average distance over N samples: ");
            sprintf(str_s0, "\t%.2f", mean_distance_s0);
            putsUart0(str_s0);
            putsUart0(" (mm)\n");
            putsUart0("sensor 0 variance over N samples: ");
            sprintf(str_s0, "\t\t%.2f", variance_s0);
            putsUart0(str_s0);
            putsUart0("\n");

            putsUart0("sensor 1 average distance over N samples: ");
            sprintf(str_s1, "\t%.2f", mean_distance_s1);
            putsUart0(str_s1);
            putsUart0(" (mm)\n");
            putsUart0("sensor 1 variance over N samples: ");
            sprintf(str_s1, "\t\t%.2f", variance_s1);
            putsUart0(str_s1);
            putsUart0("\n");

            putsUart0("sensor 2 average distance over N samples: ");
            sprintf(str_s2, "\t%.2f", mean_distance_s2);
            putsUart0(str_s2);
            putsUart0(" (mm)\n");
            putsUart0("sensor 2 variance over N samples: ");
            sprintf(str_s2, "\t\t%.2f", variance_s2);
            putsUart0(str_s2);
            putsUart0("\n\n----------------------------------------\n\n");
        }

        N_reached = 0;
        sum_s0 = 0;
        sum_s1 = 0;
        sum_s2 = 0;

        sum0_variance = 0;
        sum1_variance = 0;
        sum2_variance = 0;
    }

    if(variance_flag)
    {
        putsUart0("\n----------------------------------------\nV = \t");
        sprintf(str_s0, "%u", V);
        putsUart0(str_s0);
        putsUart0("\n");
        putsUart0("Cannot fix (x,y), variance is V or higher.\n\n----------------------------------------\n\n");

        variance_flag = 0;
    }

    if(tri & fix)
    {
        x_fix = ((sensor2_x - sensor1_x) * (sensor2_x - sensor1_x) + (mean_distance_s1) * (mean_distance_s1) - (mean_distance_s2) * (mean_distance_s2))
                /(2 * (sensor2_x - sensor1_x));
        y_fix = ((sensor0_y - sensor1_y) * (sensor0_y - sensor1_y) + (mean_distance_s1) * (mean_distance_s1) - (mean_distance_s0) * (mean_distance_s0))
                /(2 * (sensor0_y - sensor1_y));
        putsUart0("(x,y): ");
        sprintf(str_s0, "\t(%u mm,%u mm)", x_fix, y_fix);
        putsUart0(str_s0);
        putsUart0("\n");

        fix = 0;
    }

    if(se2c & fix)
    {
        x_fix = (((mean_distance_s0 * mean_distance_s0) - (mean_distance_s1 * mean_distance_s1) - (sensor0_x * sensor0_x) + (sensor1_x * sensor1_x)
                  - (sensor0_y * sensor0_y) + (sensor1_y * sensor1_y)) * ((2 * sensor2_y) - (2 * sensor1_y)) - ((mean_distance_s1 * mean_distance_s1)
                  - (mean_distance_s2 * mean_distance_s2) - (sensor1_x * sensor1_x) + (sensor2_x * sensor2_x) - (sensor1_y * sensor1_y)
                  + (sensor2_y * sensor2_y)) * ((2 * sensor1_y) - (2 * sensor0_y)))
                  / (((2 * sensor2_y) - (2 * sensor1_y)) * ((2 * sensor1_x) - (2 * sensor0_x)) - ((2 * sensor1_y) - (2 * sensor0_y))
                  * ((2 * sensor2_x) - (2 * sensor1_x)));
        y_fix = (((mean_distance_s0 * mean_distance_s0) - (mean_distance_s1 * mean_distance_s1) - (sensor0_x * sensor0_x) + (sensor1_x * sensor1_x)
                  - (sensor0_y * sensor0_y) + (sensor1_y * sensor1_y)) * ((2 * sensor2_x) - (2 * sensor1_x)) - ((2 * sensor1_x) - (2 * sensor0_x))
                  * ((mean_distance_s1 * mean_distance_s1) - (mean_distance_s2 * mean_distance_s2) - (sensor1_x * sensor1_x) + (sensor2_x * sensor2_x)
                  - (sensor1_y * sensor1_y) + (sensor2_y * sensor2_y)))
                  / (((2 * sensor1_y) - (2 * sensor0_y)) * ((2 * sensor2_x) - (2 * sensor1_x)) - ((2 * sensor1_x) - (2 * sensor0_x))
                  * ((2 * sensor2_y) - (2 * sensor1_y)));
        putsUart0("(x,y): ");
        sprintf(str_s0, "\t(%u mm,%u mm)", x_fix, y_fix);
        putsUart0(str_s0);
        putsUart0("\n");

        fix = 0;
    }

    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;           //reset the 3 wide timers
    WTIMER1_TAV_R = 0;
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER2_TAV_R = 0;
    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER3_TAV_R = 0;

    WTIMER4_TAILR_R = 80000;
    WTIMER4_ICR_R = TIMER_ICR_TATOCINT;         //clear interrupt flag
    enablePinInterrupt(IR_IN);
}

void wideTimer5Isr(void)
{
    PWM0_0_CTL_R = 0;                           // turn-off PWM0 generator 0
    PWM0_1_CMPA_R = 0;

    if(error_beep2_flag)
    {
        //putsUart0("error_beep2\n");
        playBeep(&error_beep);
        error_beep2_flag = 0;
        hit2_flag = 0;
        hit3_flag = 0;
    }
    else if(error_beep3_flag)
    {
        //putsUart0("error_beep3\n");
        playBeep(&error_beep);
        error_beep3_flag = 0;
        hit2_flag = 0;
        hit3_flag = 0;
    }
    else if(hit2_flag)
    {
        //putsUart0("hit2_beep\n");
        playBeep(&hit2_beep);
        hit2_flag = 0;
    }
    else if(hit3_flag)
    {
        //putsUart0("hit3_beep\n");
        playBeep(&hit3_beep);
        hit3_flag = 0;
    }
    else if(fix_beep_flag)
    {
        //putsUart0("fix_beep\n");
        playBeep(&fix_beep);
        fix_beep_flag = 0;
    }

    WTIMER5_ICR_R = TIMER_ICR_TATOCINT;         //clear interrupt flag
}

void irISR(void)
{
    //putsUart0("IR\n");
    WTIMER1_TAV_R = 0;
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;            //turn on wide timer 1 for sensor 0

    WTIMER2_TAV_R = 0;
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;            //turn on wide timer 2 for sensor 1

    WTIMER3_TAV_R = 0;
    WTIMER3_CTL_R |= TIMER_CTL_TAEN;            //turn on wide timer 3 for sensor 2

    WTIMER4_TAILR_R = 80000;
    WTIMER4_CTL_R |= TIMER_CTL_TAEN;            //turn on wide timer 4 for timeout

    count_s0 = 0;
    count_s1 = 0;
    count_s2 = 0;

    hit_s0 = 0;
    hit_s1 = 0;
    hit_s2 = 0;

    if(ir_beep_on)
    {
        ir_beep_flag = 1;
    }
    clearPinInterrupt(IR_IN);
    disablePinInterrupt(IR_IN);
}

int main(void)
{
    initHw();
	initEeprom();
    retrieveEeprom();                           //this function will retrieve values from EEPROM on powerup
    initUart0();
    setUart0BaudRate(115200, 40e6);
    configWideTimers();
    USER_DATA data;

    blank_beep.frequency = 0;                   //used for dummy beep to prime the pump in case ir beep is off
    blank_beep.duration = 1;

    char *first_arg;                            //used for char* arguments
    char *second_arg;
    char *third_arg;

    while(true)
    {
        if (kbhitUart0())
        {
            getsUart0(&data);
            parseFields(&data);

            if(isCommand(&data, "reset", 1))
            {
                NVIC_APINT_R = NVIC_APINT_SYSRESETREQ | NVIC_APINT_VECTKEY;
            }

            if(isCommand(&data, "sensor", 4))
            {
                sensor_num = getFieldInteger(&data, 1);
                x_input = getFieldInteger(&data, 2);
                y_input = getFieldInteger(&data, 3);
                if(!(sensor_num >= 0 & sensor_num <= 2))
                {
                    putsUart0("Invalid sensor (must be 0, 1, or 2)\n");
                }

                switch(sensor_num)                                      //store (x,y) into sensor variables
                {
                    case 0:
                        sensor0_x = x_input;
                        sensor0_y = y_input;
                        writeEeprom(16, sensor0_x);                     //block 1, offset 0
                        writeEeprom(17, sensor0_y);                     //block 1, offset 1
                        break;
                    case 1:
                        sensor1_x = x_input;
                        sensor1_y = y_input;
                        writeEeprom(18, sensor1_x);                     //block 1, offset 2
                        writeEeprom(19, sensor1_y);                     //block 1, offset 3
                        break;
                    case 2:
                        sensor2_x = x_input;
                        sensor2_y = y_input;
                        writeEeprom(20, sensor2_x);                     //block 1, offset 4
                        writeEeprom(21, sensor2_y);                     //block 1, offset 5
                        break;
                }
            }

            if(isCommand(&data, "beep", 3))
            {
                first_arg = getFieldString(&data, 1);
                if(str_cmp(first_arg, "ir"))
                {
                    second_arg = getFieldString(&data,2);
                    if(str_cmp(second_arg, "ON"))
                    {
                        ir_beep_on = 1;
                        writeEeprom(22, 1);                             //block 1, offset 6
                    }
                    else if(str_cmp(second_arg, "OFF"))
                    {
                        ir_beep_on = 0;
                        writeEeprom(22, 0);                             //block 1, offset 6
                    }
                    else if(str_cmp(second_arg, "freq"))
                    {
                        third_arg = getFieldString(&data, 3);
                        if(str_cmp(third_arg, "duration"))
                        {
                            ir_beep.frequency = getFieldInteger(&data, 4);
                            ir_beep.duration = getFieldInteger(&data, 5);
                            writeEeprom(23, ir_beep.frequency);         //block 1, offset 7
                            writeEeprom(24, ir_beep.duration);          //block 1, offset 8
                        }
                    }
                }

                if(str_cmp(first_arg, "error"))
                {
                    second_arg = getFieldString(&data,2);
                    if(str_cmp(second_arg, "2"))
                    {
                        third_arg = getFieldString(&data, 3);
                        if(str_cmp(third_arg, "ON"))
                        {
                            error_beep2_on = 1;
                            writeEeprom(25, 1);                         //block 1, offset 9
                        }
                        else if(str_cmp(third_arg, "OFF"))
                        {
                            error_beep2_on = 0;
                            writeEeprom(25, 0);                         //block 1, offset 9
                        }
                    }
                    else if(str_cmp(second_arg, "3"))
                    {
                        third_arg = getFieldString(&data, 3);
                        if(str_cmp(third_arg, "ON"))
                        {
                            error_beep3_on = 1;
                            writeEeprom(26, 1);                         //block 1, offset 10
                        }
                        else if(str_cmp(third_arg, "OFF"))
                        {
                            error_beep3_on = 0;
                            writeEeprom(26, 0);                         //block 1, offset 10
                        }
                    }
                    else if(str_cmp(second_arg, "freq"))
                    {
                        third_arg = getFieldString(&data, 3);
                        if(str_cmp(third_arg, "duration"))
                        {
                            error_beep.frequency = getFieldInteger(&data, 4);
                            error_beep.duration = getFieldInteger(&data, 5);
                            writeEeprom(27, error_beep.frequency);      //block 1, offset 11
                            writeEeprom(28, error_beep.duration);       //block 1, offset 12
                        }
                    }
                }

                if(str_cmp(first_arg, "2"))
                {
                    second_arg = getFieldString(&data,2);
                    if(str_cmp(second_arg, "ON"))
                    {
                        beep2_on = 1;
                        writeEeprom(29, 1);                             //block 1, offset 13
                    }
                    else if(str_cmp(second_arg, "OFF"))
                    {
                        beep2_on = 0;
                        writeEeprom(29, 0);                             //block 1, offset 13
                    }
                    else if(str_cmp(second_arg, "freq"))
                    {
                        third_arg = getFieldString(&data, 3);
                        if(str_cmp(third_arg, "duration"))
                        {
                            hit2_beep.frequency = getFieldInteger(&data, 4);
                            hit2_beep.duration = getFieldInteger(&data, 5);
                            writeEeprom(30, hit2_beep.frequency);       //block 1, offset 14
                            writeEeprom(31, hit2_beep.duration);        //block 1, offset 15
                        }
                    }
                }

                if(str_cmp(first_arg, "3"))
                {
                    second_arg = getFieldString(&data,2);
                    if(str_cmp(second_arg, "ON"))
                    {
                        beep3_on = 1;
                        writeEeprom(32, 1);                             //block 2, offset 0
                    }
                    else if(str_cmp(second_arg, "OFF"))
                    {
                        beep3_on = 0;
                        writeEeprom(32, 0);                             //block 2, offset 0
                    }
                    else if(str_cmp(second_arg, "freq"))
                    {
                        third_arg = getFieldString(&data, 3);
                        if(str_cmp(third_arg, "duration"))
                        {
                            hit3_beep.frequency = getFieldInteger(&data, 4);
                            hit3_beep.duration = getFieldInteger(&data, 5);
                            writeEeprom(33, hit3_beep.frequency);       //block 2, offset 1
                            writeEeprom(34, hit3_beep.duration);        //block 2, offset 2
                        }
                    }
                }

                if(str_cmp(first_arg, "fix"))
                {
                    second_arg = getFieldString(&data,2);
                    if(str_cmp(second_arg, "ON"))
                    {
                        fix_beep_on = 1;
                        writeEeprom(38, 1);                             //block 2, offset 6
                    }
                    else if(str_cmp(second_arg, "OFF"))
                    {
                        fix_beep_on = 0;
                        writeEeprom(38, 0);                             //block 2, offset 6
                    }
                    else if(str_cmp(second_arg, "freq"))
                    {
                        third_arg = getFieldString(&data, 3);
                        if(str_cmp(third_arg, "duration"))
                        {
                            fix_beep.frequency = getFieldInteger(&data, 4);
                            fix_beep.duration = getFieldInteger(&data, 5);
                            writeEeprom(39, fix_beep.frequency);        //block 2, offset 7
                            writeEeprom(40, fix_beep.duration);         //block 2, offset 8
                        }
                    }
                }
            }

            if(isCommand(&data, "distance", 1))
            {
                first_arg = getFieldString(&data, 1);
                if(str_cmp(first_arg, "OFF"))
                {
                    distance_on = 0;
                    writeEeprom(35, 0);                                 //block 2, offset 3
                }
                else
                {
                    distance_on = 1;
                    writeEeprom(35, 1);                                 //block 2, offset 3
                }
            }

            if(isCommand(&data, "average", 2))
            {
                N = getFieldInteger(&data, 1);
                writeEeprom(36, N);                                     //block 2, offset 4
            }

            if(isCommand(&data, "variance", 2))
            {
                V = getFieldInteger(&data, 1);
                writeEeprom(37, V);                                     //block 2, offset 5
            }

            if(isCommand(&data, "correction", 4))
            {
                sensor_num = getFieldInteger(&data, 1);
                k0_input = getFieldInteger(&data, 2);
                k1_input = getFieldInteger(&data, 3);
                if(!(sensor_num >= 0 & sensor_num <= 2))
                {
                    putsUart0("Invalid sensor (must be 0, 1, or 2)\n");
                }

                switch(sensor_num)
                {
                    case 0:
                        sensor0_k0 = k0_input;
                        sensor0_k1 = k1_input;
                        writeEeprom(41, sensor0_k0);                    //block 2, offset 9
                        writeEeprom(42, sensor0_k1);                    //block 2, offset 10
                        break;
                    case 1:
                        sensor1_k0 = k0_input;
                        sensor1_k1 = k1_input;
                        writeEeprom(43, sensor1_k0);                    //block 2, offset 11
                        writeEeprom(44, sensor1_k1);                    //block 2, offset 12
                        break;
                    case 2:
                        sensor2_k0 = k0_input;
                        sensor2_k1 = k1_input;
                        writeEeprom(45, sensor2_k0);                    //block 2, offset 13
                        writeEeprom(46, sensor2_k1);                    //block 2, offset 14
                        break;
                }
            }

            if(isCommand(&data, "method", 2))
            {
                first_arg = getFieldString(&data, 1);
                if(str_cmp(first_arg, "2SE2C"))
                {
                    se2c = 1;
                    tri = 0;
                    writeEeprom(47, 1);                                 //block 2, offset 15
                    writeEeprom(48, 0);                                 //block 3, offset 0
                }
                else if(str_cmp(first_arg, "2TRI"))
                {
                    tri = 1;
                    se2c = 0;
                    writeEeprom(48, 1);                                 //block 3, offset 0
                    writeEeprom(47, 0);                                 //block 2, offset 15
                }
            }
        }
    }
}
