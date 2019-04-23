/******************************************************************************
MSP430G2553 Project Creator
SE 423  - Dan Block
        Spring(2019)
        Written(by) : Steve(Keres)
College of Engineering Control Systems Lab
University of Illinois at Urbana-Champaign
*******************************************************************************/

#include "msp430g2553.h"
#include "UART.h"

void print_every(int rate);
void pwm_motor(int DutyCycle1,int DutyCycle2);

char newprint = 0;
long NumOn = 0;
long NumOff = 0;
int statevar = 1;
int timecheck = 0;
int DutyCycle1 = 50;
int DutyCycle2 = 50;
int turn = 0;

unsigned int ADC[4];  // Array to hold ADC values
int A0value = 0;
int A3value = 0;
char newADC = 0;

unsigned char frontObstacle = 0;
unsigned char backObstacle = 0;

char recchar = 0;
void main(void) {

    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

    if (CALBC1_16MHZ ==0xFF || CALDCO_16MHZ == 0xFF) while(1);

    DCOCTL = CALDCO_16MHZ;    // Set uC to run at approximately 16 Mhz
    BCSCTL1 = CALBC1_16MHZ;
	
    // Initialize Port 1
    P1SEL &= ~0x09;  // See page 42 and 43 of the G2553's datasheet, It shows that when both P1SEL and P1SEL2 bits are zero
    P1SEL2 &= ~0x09; // the corresponding pin is set as a I/O pin.  Datasheet: http://coecsl.ece.illinois.edu/ge423/datasheets/MSP430Ref_Guides/msp430g2553datasheet.pdf
    P1REN = 0x0;  // No resistors enabled for Port 1
    P1DIR &= ~0x9; // Set P1.0 P1.3 to intput to drive LED on LaunchPad board.  Make sure shunt jumper is in place at LaunchPad's Red LED

    P1SEL &= ~0x10; set P1.4 I/O
    P1SEL2 &= ~0x10;
    P1REN = 0x0;  // No resistors enabled for Port 1
    P1DIR |= 0x10; // Set P1.4 output
    P1OUT |= 0x10;
    P1OUT &= ~0x10; // clear P1.4 output


    // Timer A Config
    TACCTL0 = CCIE;             // Enable Periodic interrupt
    TACCR0 = 16000;                // period = 1ms
    TACTL = TASSEL_2 + MC_1; // source SMCLK, up mode

    ADC10CTL1 = INCH_3 + ADC10SSEL_3 + CONSEQ_1; //INCH_3: Enable A3 first, Use SMCLK, Sequence of Channels
    ADC10CTL0 = ADC10ON + MSC + ADC10IE;  // Turn on ADC,  Put in Multiple Sample and Conversion mode,  Enable Interrupt

    ADC10AE0 |= 0x09;                   // Enable A0 and A3 which are P1.0,P1.3

    ADC10DTC1 = 4;                 // Four conversions.
    ADC10SA = (short)&ADC[0];           // ADC10 data transfer starting address.



    // Initialize Port 2
    P2DIR |= BIT2 + BIT4;                          // P2.1 P2.4 output
    P2SEL |= BIT2 + BIT4;                          // P2.1 P2.4 TA1.1 TA1.2 option
    P2SEL2 &= ~(BIT2 + BIT4);                        // P2.1 P2.4 TA1.1 TA1.2 option

    // Timer A Config
    TACCTL0 = CCIE;             // Enable Periodic interrupt
    TACCR0 = 16000;                // period = 1ms
    TACTL = TASSEL_2 + MC_1; // source SMCLK, up mode

    // Timer A1.1 Config
    TA1CCR0 =  800    ;                            // PWM Period  ms
    TA1CCTL1 = OUTMOD_7; // TA1CCR1 reset/set
    TA1CCTL2 = OUTMOD_7; // TA1CCR2 reset/set
    TA1CCR1 = 0; // TA1CCR1 PWM duty cycle
    TA1CCR2 = 0;// TA1CCR2 PWM duty cycle
    TA1CTL = TASSEL_2 + MC_1;                  // SMCLK, up mode 
    DutyCycle1 = 50; 			// Initialize DutyCycle1 and DutyCycle2
    DutyCycle2 = 50;

    Init_UART(9600,1);  // Initialize UART for 9600 baud serial communication

    _BIS_SR(GIE);       // Enable global interrupt


    while(1) {  // Low priority Slow computation items go inside this while loop.  Very few (if anyt) items in the HWs will go inside this while loop

// for use if you want to use a method of receiving a string of chars over the UART see USCI0RX_ISR below
//      if(newmsg) {
//          newmsg = 0;
//      }

        // The newprint variable is set to 1 inside the function "print_every(rate)" at the given rate
        if ( (newprint == 1) && (senddone == 1) )  { // senddone is set to 1 after UART transmission is complete

            // only one UART_printf can be called every 15ms
            //UART_printf("D1%d D2%d A0%d A3%d\n\r",DutyCycle1,DutyCycle2,A0value,A3value);

            newprint = 0;
        }

    }
}


// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
    timecheck++; // Keep track of time for main while loop.
    if ((timecheck%100) == 0) {
        ADC10CTL0 |= ENC + ADC10SC;         // Enable Sampling and start conversion every 100ms.
    }

    print_every(500);  // units determined by the rate Timer_A ISR is called, print every "rate" calls to this function
}



// ADC 10 ISR - Called when a sequence of conversions (A7-A0) have completed
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
    A0value = ADC[3];  // Notice the reverse in index back
    A3value = ADC[0];  // ADC[0] has A3 value and ADC[3] has A0's value front
	/*check the back obstacle, if back obstacle is detected, saturate DutyCycle1 and DutyCycle2 so the robot can not go back, also blink LED */
    if(A0value>900){ 
        backObstacle = 1;
        if(DutyCycle1 > 50){
            DutyCycle1 = 50;
        }
        if(DutyCycle2 > 50){
            DutyCycle2 =50;
        }
        P1OUT ^= 0x10; // Set P1.4 output
    }
    else{
        backObstacle = 0;
    }
	/*check the front obstacle, if front obstacle is detected, saturate DutyCycle1 and DutyCycle2 so the robot can not go front, also blink LED */
    if(A3value>900){
        frontObstacle = 1;
        if(DutyCycle1 < 50){
            DutyCycle1 = 50;
        }
        if(DutyCycle2 < 50){
            DutyCycle2 =50;
        }
        P1OUT ^= 0x10; // Set P1.4 output
    }
    else{
        frontObstacle = 0;
    }
	// sent DutyCycle1,DutyCycle2 to PWM
    pwm_motor(DutyCycle1,DutyCycle2);



    ADC10CTL0 &= ~ADC10IFG;  // clear interrupt flag

    ADC10SA = (short)&ADC[0]; // ADC10 data transfer starting address.

    newADC = 1;
}



// USCI Transmit ISR - Called when TXBUF is empty (ready to accept another character)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void) {

    if(IFG2&UCA0TXIFG) {        // USCI_A0 requested TX interrupt
        if(printf_flag) {
            if (currentindex == txcount) {
                senddone = 1;
                printf_flag = 0;
                IFG2 &= ~UCA0TXIFG;
            } else {
                UCA0TXBUF = printbuff[currentindex];
                currentindex++;
            }
        } else if(UART_flag) {
            if(!donesending) {
                UCA0TXBUF = txbuff[txindex];
                if(txbuff[txindex] == 255) {
                    donesending = 1;
                    txindex = 0;
                }
                else txindex++;
            }
        } else {  // interrupt after sendchar call so just set senddone flag since only one char is sent
            senddone = 1;
        }

        IFG2 &= ~UCA0TXIFG;
    }

    if(IFG2&UCB0TXIFG) {    // USCI_B0 requested TX interrupt (UCB0TXBUF is empty)

        IFG2 &= ~UCB0TXIFG;   // clear IFG
    }
}


// USCI Receive ISR - Called when shift register has been transferred to RXBUF
// Indicates completion of TX/RX operation
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {

    if(IFG2&UCB0RXIFG) {  // USCI_B0 requested RX interrupt (UCB0RXBUF is full)

        IFG2 &= ~UCB0RXIFG;   // clear IFG
    }

    if(IFG2&UCA0RXIFG) {  // USCI_A0 requested RX interrupt (UCA0RXBUF is full)
        statevar = UCA0RXBUF; // read HEX value from UART BUFF which comes from the bluetooth
        sendchar(statevar); // send value back to terminal window
            switch (statevar) {
                case 1:  //case 1 statevar == 1 go front increase spd and saturate DutyCycle1 and DutyCycle2, sent to PWM func
                    DutyCycle1--;
                    DutyCycle2--;
                    if(DutyCycle1 <0) DutyCycle1 = 0;
                    if(DutyCycle2 <0) DutyCycle2 = 0;
                    pwm_motor(DutyCycle1,DutyCycle2);
                    break;
                case 2:  // case 2 statevar == 2 back decrease spd and saturate DutyCycle1 and DutyCycle2, sent to PWM func
                    DutyCycle1++;
                    DutyCycle2++;
                    if(DutyCycle1 > 100) DutyCycle1 = 100;
                    if(DutyCycle2 > 100) DutyCycle2 = 100;
                    pwm_motor(DutyCycle1,DutyCycle2);
                    break;
                case 3:  //case 3 statevar == 3 right turn increase DutyCycle1 and decrease DutyCycle2
                    DutyCycle1--;
                    DutyCycle2++;
                    pwm_motor(DutyCycle1,DutyCycle2);
                    break;
                case 4:  //case 4 statevar == 4 left turn increase DutyCycle2 and decrease DutyCycle1
                    DutyCycle1++;
                    DutyCycle2--;
                    pwm_motor(DutyCycle1,DutyCycle2);
                    break;
                case 5:  //case 5 statevar == 5 stop
                    DutyCycle1 = 50;
                    DutyCycle2 = 50;
                    pwm_motor(DutyCycle1,DutyCycle2);
                    break;
                default: // default case
                    DutyCycle1 = 50;
                    DutyCycle2 = 50;
                    pwm_motor(DutyCycle1,DutyCycle2);
                    break;
            }
        IFG2 &= ~UCA0RXIFG;
    }

}

// This function takes care of all the timing for printing to UART
// Rate determined by how often the function is called in Timer ISR
int print_timecheck = 0;
void print_every(int rate) {
    if (rate < 15) {
        rate = 15;
    }
    if (rate > 10000) {
        rate = 10000;
    }
    print_timecheck++;
    if (print_timecheck == rate) {
        print_timecheck = 0;
        newprint = 1;
    }

}
/*PWM function, read DutyCycle1 and DutyCycle2 and transfer to TA1CCR1 and TA1CCR2*/
void pwm_motor(int DutyCycle1,int DutyCycle2){
    DutyCycle2 = 100 - DutyCycle2;
    TA1CCR1 = (DutyCycle1 * (long)TA1CCR0)/100; //TA1CCR1 PWM duty cycle conversion
    TA1CCR2 = (DutyCycle2 * (long)TA1CCR0)/100; //TA1CCR2 PWM duty cycle conversion
}
