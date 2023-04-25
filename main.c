#include <msp430.h> 

#define Red_LED BIT1;
#define Green_LED BIT0;
#define Blue_LED BIT0;

#define Green_SW BIT7;
#define Red_SW BIT6;

#define Duty_Cycle 32768;
#define DeBounce_Delay 10000;

#define Relay BIT2;
#define On 1
#define Off 0

void Setup_GPIO(void);
void Setup_Timers(void);
void Setup_UART(void);
void Send_UART_Message(int mode);
unsigned int i;

int Mode = 2;
int Transmit_Mode;
//UART Variables:

char Idle_Message[25] = {"\e[?25l\033[2J\rSystem Idle."};
char Start_Message[16] = {"\rSystem Started."};
char Stop_Message[16] = {"\rSystem Stopped."};
//char *UART_Message_ptr = UART_Message_Global;
unsigned int UART_Position_Counter;
int UART_Message_Length;

int First_Setup = 0;
void main(void)
{
 	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	Setup_GPIO();
	Setup_Timers();
  //  Setup_UART();
    __enable_interrupt();

    while(1){
        //Infinite loop waiting for interrupt
        if(First_Setup == 0){
        //Send_UART_Message(1);
        for(i = 0; i < 10000; i ++);
        First_Setup = 1;
        }
        else{
            //Do nothing.
        }
    }
}




void Setup_GPIO(void){
    //Setup LEDS
    P1DIR |= BIT0;
    P1DIR |= BIT1;
    P1OUT &= ~Red_LED;
    P1OUT &= ~Green_LED;
    P2DIR |= Blue_LED;
    P2OUT &= ~Blue_LED;
    //Setup Relay pin
    P1DIR |= Relay;
    P1OUT &= ~Relay;

    //Setup input Switches
    P2DIR &= ~Green_SW;
    P2DIR &= ~Red_SW;
    P2IES &= ~Green_SW;
    P2IES &= ~Red_SW;

    P2IFG &= ~Green_SW;
    P2IFG &= ~Red_SW;

    P2IE |= Green_SW;
    P2IE |= Red_SW;

    //Enable IO (take out of low power mode)
    PM5CTL0 &= ~LOCKLPM5;

}

void Setup_Timers(void){
    TB1CTL |= TBCLR;
    TB1CTL |= TBSSEL__ACLK;
    TB1CTL |= MC__UP;
    TB1CCR0 |= Duty_Cycle;               //Set to .5 seconds interrupt to trigger.....

    TB1CCTL0 |= CCIE;
    TB1CCTL0 &= ~CCIFG;
}

void Setup_UART(void){
    // Take eUSCI_B1 out of software reset with UCSWRST = 0
    UCA0CTLW0 &= ~UCSWRST;

    // UART Communication to PC, Setup UART A1 (Tx)

    UCA0CTLW0 |= UCSWRST;

    UCA0CTLW0 |= UCSSEL__SMCLK; // Using SM clock

    //UCA1BRW = 6;              //For 9600
    UCA0BRW = 1;                //For 38400

    //UCA1MCTLW |= 0x2081;        //For 9600
    UCA0MCTLW |= 0x00A1;

    P1SEL1 &= ~BIT6; // Use pin 4.7 for UART TX to Bluetooth
    P1SEL0 |= BIT6;


    P1SEL1 &= ~BIT7; // Use pin4.6 for UART RX on Bluetooth
    P1SEL0 |= BIT7;


    UCA0CTLW0 &= ~UCSWRST; // Take UART A1 out of SW Reset
}

void Send_UART_Message(int mode){
    UART_Position_Counter = 0;
    UCA0IE |= UCTXCPTIE;                                             //Enable the TX IRQ
    UCA0IE |= UCRXIE;                                                //Enable the RX IRQ
    UCA0IFG &= ~UCTXCPTIFG;                                          //Clear the TX IFG
    Transmit_Mode = mode;
    switch(Transmit_Mode){
    case 1:
        UCA0TXBUF = Idle_Message[UART_Position_Counter];          //Put first value into the tx buffer
        UART_Message_Length = 25;
        break;
    case 2:
        UCA0TXBUF = Start_Message[UART_Position_Counter];          //Put first value into the tx buffer
        UART_Message_Length = 16;

        break;
    case 3:
        UCA0TXBUF = Stop_Message[UART_Position_Counter];          //Put first value into the tx buffer
        UART_Message_Length = 16;

        break;
    default:
        break;
    }
}
//Interrupts here
//Need GPIO Interrupts for SW1 and SW2
//Need Timer Interrupt for the Pump control (Maybe use a sensor of some kind??)
//Need I2C Interrupt for the RTC
//Need UART Interrupt for the bluetooth.


//Port 2 interrupt (Green and Red Switch)

#pragma vector = PORT2_VECTOR
__interrupt void ISR_PORT2_Switches(void){
    switch(P2IV){
    case 16:
        P2IE &= ~Green_SW;
        for(i = 0; i < 10000; i++);
        for(i = 0; i < 10000; i++);
        P2IFG &= ~Green_SW;
        P2OUT &= ~Blue_LED;
        P1OUT |= Green_LED;
        P1OUT &= ~Red_LED;
        P1OUT |= Relay;
        TB1CTL &= MC__STOP; //Turn off idle LED
        //Send_UART_Message(2);
        break;
    case 14:
         P2IE &= ~Red_SW;
         for(i = 0; i < 10000; i++);
         for(i = 0; i < 10000; i++);
         P1OUT |= Red_LED;
         P1OUT &= ~Green_LED;
         P2OUT &= ~Blue_LED;

         P1OUT |= Relay;
         P2IFG &= ~Red_SW;
         TB1CTL &= MC__STOP; //Turn off idle LED
        //Send_UART_Message(3);

         break;
    default:
        break;
    }
    P2IE |= Green_SW;
    P2IE |= Red_SW;
}

//Timer interrupt
//When this triggers, toggle the pump??
//Right now testing with LEDs.
#pragma vector = TIMER1_B0_VECTOR
__interrupt void Pump_Timer(void)
{
    TB0CCTL0 &= ~CCIFG;
    P2OUT ^= Blue_LED;

}


//UART Interrupt
#pragma vector = EUSCI_A0_VECTOR
__interrupt void ISR_EUSCI_A0(void)
{
        if (UART_Position_Counter == UART_Message_Length)
        {
            UCA0IE &= ~UCTXCPTIE;
        }
        else
        {
            UART_Position_Counter++;
            switch(Transmit_Mode){
            case 1:
                UCA0TXBUF = Idle_Message[UART_Position_Counter];          //Put first value into the tx buffer
                break;
            case 2:
                UCA0TXBUF = Start_Message[UART_Position_Counter];          //Put first value into the tx buffer

                break;
            case 3:
                UCA0TXBUF = Stop_Message[UART_Position_Counter];          //Put first value into the tx buffer

                break;
            default:
                break;
            }        }
    UCA0IFG &= ~UCTXCPTIFG;
}

