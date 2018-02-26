/*
***************************ECE 4760: FINAL PROJECT*************************
*  Title: Bluetooth Collision Avoidance Car
*  Authors: Dev Sanghvi (dys27), Abhimanyu Khandelwal (ak2455) & Anwitha Paruchuri (ap2286)
* Target PIC:  PIC32MX250F128B
 * Not to be distributed!
 */
///////////////////////////////////////////////////////////////////
// Clocks and Protothreads Configure!
#include "config_1_2_2.h"
// threading library
#include "pt_cornell_1_2_2.h"

//PORT A Internal Pull ups/Pull downs
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;

char buffer[60];  // string buffer
// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_serial;
// The following threads are necessary for UART control

//SPI DAC SETUP
volatile SpiChannel spiChn = SPI_CHANNEL2;
volatile int spiClkDiv = 2; // 20 MHz DAC clock

// Variables
static char character;
static int duty_cycle = 4200; //Initial value for the servo motor in forward direction
static int time1 = 0, time2 = 0, time3=0, time4=0; //To store timer counts
static int flag1 = 0, flag2=0, flag3=0, flag4=0; //Keep track of execution loops

volatile int south = 0, west = 0, north = 0, east = 0; //To capture Infrared sensor values

/****** Input Capture Event Interrupt *******/
void __ISR(_INPUT_CAPTURE_1_VECTOR, ipl2) IC1Handler(void)
{
    south = mIC1ReadCapture(); //Read Capture event for Back Sensor
    if((south < 200) && (mPORTAReadBits(BIT_0)==1))
            {
                mPORTASetBits(BIT_0);
                mPORTASetBits(BIT_3);
                duty_cycle = 4200;
            }
    mIC1ClearIntFlag(); //clear Interrupt Flag
}
void __ISR(_INPUT_CAPTURE_4_VECTOR, ipl2) IC4Handler(void)
{
    west = mIC4ReadCapture(); //Read Capture event for left sensor
    if (west <200 && (mPORTAReadBits(BIT_0)==0) && duty_cycle == 3000) //Obstacle Detection for front left
    {
        mPORTASetBits(BIT_0);
        mPORTASetBits(BIT_3);
        duty_cycle = 4200;
    }
    if (west <200 && (mPORTAReadBits(BIT_0)==1) && duty_cycle == 5500) //Obstacle detection for back right
    {
        mPORTASetBits(BIT_0);
        mPORTASetBits(BIT_3);
        duty_cycle = 4200;
    }
    mIC4ClearIntFlag(); //clear Interrupt flag
}
void __ISR(_INPUT_CAPTURE_5_VECTOR, ipl2) IC5Handler(void)
{
    north = mIC5ReadCapture(); //Read Capture event for front sensor
    if (north <200 && (mPORTAReadBits(BIT_0)==0))
    {
        mPORTAClearBits(BIT_0);
        mPORTAClearBits(BIT_3);
        duty_cycle = 4200;
    }
    mIC5ClearIntFlag(); //clear Interrupt flag
}

void __ISR(_INPUT_CAPTURE_3_VECTOR, ipl2) IC3Handler(void)
{
    east = mIC3ReadCapture(); //Read Capture event for right sensor
    if (east <200 && (mPORTAReadBits(BIT_0)==0) && duty_cycle == 5500) //Obstacle detection for front right
    {

        mPORTASetBits(BIT_0);
        mPORTASetBits(BIT_3);
        duty_cycle = 4200;
    }
       if (east <200 && (mPORTAReadBits(BIT_0)==1)&& duty_cycle == 3000) //Obstacle detection for back left
    {
        mPORTASetBits(BIT_0);
        mPORTASetBits(BIT_3);
        duty_cycle = 4200;
    }
    mIC3ClearIntFlag(); //Clear Interrupt flag
}

//=== Serial terminal thread =========================================
//Thread for interfacing with the Bluetooth to take user input
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
      mPORTASetPinsDigitalOut(BIT_0 | BIT_3); //Set port as output
      mPORTAClearBits(BIT_0 | BIT_3);
      while(1) {
          PT_YIELD_TIME_msec(100);
        character = UARTGetDataByte(UART2); //Store Bluetooth command in a variable
        UARTSendDataByte(UART2, character); //Echo back the same command to serial terminal

        if(character == 'b') //Backward direction
        {
            mPORTASetBits(BIT_0);
            mPORTAClearBits(BIT_3);
            duty_cycle = 4000;
        }
        if(character == 'f') //Forward direction
        {
            mPORTASetBits(BIT_3);
            mPORTAClearBits(BIT_0);
            duty_cycle = 4200;
        }
        if(character == 's') //Stop
        {
            mPORTASetBits(BIT_0);
            mPORTASetBits(BIT_3);
            duty_cycle = 4200;
        }
        if(character == 'i') //Turn left and Forward
        {
            flag1=0;
            time1 = PT_GET_TIME();
            duty_cycle = 3000;
            mPORTAClearBits(BIT_0);
            mPORTASetBits(BIT_3);
        }
        if((PT_GET_TIME()-time1)>1500 && flag1==0)
            {
                UARTSendDataByte(UART2, character);
                duty_cycle=4200;
                flag1=1;
            }
        if(character == 'o') //Turn right and forward
        {
            time2 = PT_GET_TIME();
            flag2=0;
            mPORTAClearBits(BIT_0);
            mPORTASetBits(BIT_3);
            duty_cycle = 5500;
        }
            if(PT_GET_TIME()-time2>=1500 && flag2==0)
            {
                duty_cycle=4200;
                flag2=1;
            }
        if(character == 'k') //Turn left and backward
        {
            time3 = PT_GET_TIME();
            flag3=0;
            mPORTASetBits(BIT_0);
            mPORTAClearBits(BIT_3);
            duty_cycle = 3000;

        }
        if(PT_GET_TIME()-time3>=1500 && flag3==0)
            {
                duty_cycle=4200;
                flag3=1;
            }
        if(character == 'l') //Turn right and backward
        {
            time4 = PT_GET_TIME();
            flag4=0;
            mPORTASetBits(BIT_0);
            mPORTAClearBits(BIT_3);
            duty_cycle = 5500;
        }
            if(PT_GET_TIME()-time4>=1500 && flag4==0)
            {
                duty_cycle=4200;
                flag4=1;
            }
        if(character == 'q') // circular left
        {
           mPORTAClearBits(BIT_0);
            mPORTASetBits(BIT_3);
            duty_cycle = 3000;
        }
        if(character == 'w') // circular right
        {
           mPORTAClearBits(BIT_0);
            mPORTASetBits(BIT_3);
            duty_cycle = 5500;
        }
        SetDCOC3PWM(duty_cycle);
      } // End While(1)
  PT_END(pt); // End Thread
}

// === Main  ======================================================

void main(void) {
      // === config threads ==========
     ANSELA = 0; ANSELB = 0;

     PT_setup();

     //=== Timer Setups =========================================
     // Set up Timer 2 to control  DC Motor speed
     OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_16, 50000); // 40000000/(50000*16)=50Hz Frequency

     // set up timer 3 for echo captures of all sensors through IC1,IC3,IC4,IC5
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_256, 20000); //Controls PWM Signals

     // back sensor Echo capture
    OpenCapture1(IC_EVERY_FALL_EDGE | IC_INT_1CAPTURE |IC_TIMER3_SRC | IC_ON );
    // turn on the interrupt so that every capture can be recorded
    ConfigIntCapture1(IC_INT_ON | IC_INT_PRIOR_2 | IC_INT_SUB_PRIOR_3);
    INTClearFlag(INT_IC1);
    // connect RPB13 to IC1 capture unit
    PPSInput(3, IC1, RPB13);
    mPORTBSetPinsDigitalIn(BIT_13); // Set port as input (BIT_13)

  // set up compare1 for back sensor
    OpenOC1(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 10, 10); //
    // OC1 is PPS group 1, map to RPB3
    PPSOutput(1, RPB3, OC1);

    // west sensor Echo capture
    OpenCapture4(IC_EVERY_FALL_EDGE | IC_INT_1CAPTURE |IC_TIMER3_SRC | IC_ON );
    // turn on the interrupt so that every capture can be recorded
    ConfigIntCapture4(IC_INT_ON | IC_INT_PRIOR_2 | IC_INT_SUB_PRIOR_2);
    INTClearFlag(INT_IC4);
    // connect RPB7 to IC4 capture unit
    PPSInput(1, IC4, RPB7);
    mPORTBSetPinsDigitalIn(BIT_7); // Set port as input (BIT_7)

    // set up compare2 for west sensor
    OpenOC2(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 10, 10); //
    //OC2 is PPS group 2, map to RPB8
    PPSOutput(2, RPB8, OC2);

    // front sensor Echo capture
    OpenCapture5(IC_EVERY_FALL_EDGE | IC_INT_1CAPTURE |IC_TIMER3_SRC | IC_ON );
    // turn on the interrupt so that every capture can be recorded
    ConfigIntCapture5(IC_INT_ON | IC_INT_PRIOR_2 | IC_INT_SUB_PRIOR_1);
    INTClearFlag(INT_IC5);
    // connect RPB2 to IC5 capture unit
    PPSInput(3, IC5, RPB2);
    mPORTBSetPinsDigitalIn(BIT_2); // Set port as input (BIT_2)

    // set up compare4 for front sensor
    OpenOC4(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 10, 10); //
    //OC4 is PPS group 3, map to RPBA2
    PPSOutput(3, RPA2, OC4);

    // east sensor Echo capture
    OpenCapture3(IC_EVERY_FALL_EDGE | IC_INT_1CAPTURE |IC_TIMER3_SRC | IC_ON );
    // turn on the interrupt so that every capture can be recorded
    ConfigIntCapture3(IC_INT_ON | IC_INT_PRIOR_2 | IC_INT_SUB_PRIOR_3);
    INTClearFlag(INT_IC3);
    // connect RPB11 to IC3 capture unit
    PPSInput(2, IC3, RPB11);
    mPORTBSetPinsDigitalIn(BIT_11); // Set port as input (BIT_11)

    // set up compare5 for east sensor
    OpenOC5(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 10, 10); //
     //OC5 is PPS group 3, map to RPA4
    PPSOutput(3, RPA4, OC5);

    //=== Setup of servo motor =========================================

    // set up compare3 for PWM mode
    OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, duty_cycle, duty_cycle); //
    // OC3 is PPS group 4, map to RPB9
    PPSOutput(4, RPB9, OC3);

    //=== Setup of SPI =========================================
    // SDO2 is in PPS output group 2 connected to RPB5
    PPSOutput(2, RPB5, SDO2);
    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);
    // divide Fpb by 2, configure the I/O ports.
    // 16 bit transfer CKP=1 CKE=1
    // Enable SPI at 20MHz clock
    SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN |
    SPI_OPEN_CKE_REV, spiClkDiv);

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
  //INIT the threads
 PT_INIT(&pt_serial);
  // Schedule the threads using round-robin scheduling
  while (1){
      PT_SCHEDULE(protothread_serial(&pt_serial));
      }
  } // main

// === end  ======================================================

