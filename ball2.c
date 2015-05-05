#include "msp430g2553.h"

/* the Ball2 (Box2) - a smart distance measuring device
 * Group members - Yiqiu Wang, Tingkai Liu, Yuqiang Heng
 *
 * This device contains a vibration sensor and a push button as control and SPI LED as output
 * 
 * Button and vibration sensor are both connected to port2, and are both debounced in interrupts
 * Buttons have three patterns, double click - reset, long push - discard, short push - record average
 * Vibration sensors trigger interrupts that records the time between consecutive impacts
 *
 * The device is to be tossed, hit the ceiling and fall to the floor, and it would generate an output
 * upon short push of the button, the results is always averaged before reset
 */

volatile int port2Flag; // This is used to track port2 flags in interrupts (for ease of debugging)
volatile int timer1;  // This is timer1, whose value is incremented in WDT interrupt periodically
volatile int timer2;  // This is timer2, whose value is incremented in WDT interrupt periodically
volatile int time1;   // This record the time between first impact and second impact
volatile int time2;   // This record the time between second and (possibly) third impact
volatile int longPressTimer;  //This is used to track how long user pressed button
volatile int fastPressTimer;  //This is used to track time between consecutive presses
volatile int resetFlag = 0;   //This flag is set when user choose  to reset all data
volatile int startFlag = 1;   //This flag is set the sensor expects a first or third impact
volatile int avg;       //This records average distance measured so far
volatile int cnt = 0;     //This records total rounds of measurements so far
volatile int dist;        //This records distance measured in the current round
volatile int timeCalibration = 110; //Ratio between time/realworldtime in sec
volatile int impactCnt = 0; //Records number of impacts in one round

void changeColor(r,g,b,global) {        //  This function changes color of SPI led according to parameter r,g,b
                                
    // Adjusts illegal inputs
    if (r < 0) r = 0;
    if (g < 0) g = 0;
    if (b < 0) b = 0;
    if (b > 255) b = 255;
    if (g > 255) g = 255;
    if (r > 255) r = 255;

    global += 224;          //  ORing global with 11100000

    P1OUT  &=  (~BIT5);           //  select device
    //TRANSMISSION START
    while  (!(IFG2 &   UCA0TXIFG));   //  detect whether USCI_A0 buffer is ready for transmission
    UCA0TXBUF  =   0x00;          //  writes 00000000
    while  (!(IFG2 &   UCA0TXIFG));
    UCA0TXBUF  =   0x00;          //  writes 00000000
    while  (!(IFG2 &   UCA0TXIFG));
    UCA0TXBUF  =   0x00;          //  writes 00000000
    while  (!(IFG2 &   UCA0TXIFG));
    UCA0TXBUF  =   0x00;          //  writes 00000000
    while  (!(IFG2 &   UCA0TXIFG));
    UCA0TXBUF  =   global;        //  global intensity
    while  (!(IFG2 &   UCA0TXIFG));
    UCA0TXBUF  =   b;           //  blue led
    while  (!(IFG2 &   UCA0TXIFG));
    UCA0TXBUF  =   g;           //  green led
    while  (!(IFG2 &   UCA0TXIFG));
    UCA0TXBUF  =   r;           //  red led
    while  (!(IFG2 &   UCA0TXIFG));
    UCA0TXBUF  =   0xFF;          //  writes 11111111
    while  (!(IFG2 &   UCA0TXIFG));
    UCA0TXBUF  =   0xFF;          //  writes 11111111
    while  (!(IFG2 &   UCA0TXIFG));
    UCA0TXBUF  =   0xFF;        //  writes 11111111
    while  (!(IFG2 &   UCA0TXIFG));
    UCA0TXBUF  =   0xFF;          //  writes 11111111
    P1OUT  |=  (BIT5);          //  unselect device
}

void display(int value){
  /*
   * This function displays an integer (value) by blinking the LED
   * The value is displayed as ( redblinks*1000 + greenblinks*100 + blueblinks*10 )
   */
  int i = 0;

  for (i = 0; i < (value/100); i++){  // blinks red led for value/100 times
    changeColor(15,0,0,100);__delay_cycles(100000);
    changeColor(0,0,0,0);__delay_cycles(300000);
  }

  for (i = 0; i < ((value%100)/10); i++){ // blinks green led for (value%100)/10 times
    changeColor(0,15,0,100);__delay_cycles(100000);
    changeColor(0,0,0,0);__delay_cycles(300000);
  }

  for (i = 0; i < (value%10); i++){ // blinks blue led for value%10 times
    changeColor(0,0,15,100);__delay_cycles(100000);
    changeColor(0,0,0,0);__delay_cycles(300000);
  }
}

void calDist(time){ // This function calculates distance traveled in free fall given time, given g = 981 cm s^-2
  dist = 0.5*981*(time/timeCalibration) * (time/timeCalibration);// distance = 0.5 * g * time^2
}

void calAverage(){  // This function calculates average of of distance measured
  startFlag = 1;
  cnt++;
  /*
   * Time 1 - time interval between first-second impact
   * Time 2 - time interval between second-third impact
   * There can be multiple impacts depending on whether the vibration sensor is triggered upon thowing,
   * which is a 50%-50% event, and whether it bounces when fall to the floor
   * We assume the switch will definitely be triggered upon impact on the ceiling and floor
   */

  if (impactCnt == 2) calDist(time1); 
  // If there are two impacts, we can be sure that they are on the ceiling and floor respectively
  // If the device does not hit either, this is considered a bad toss and the reading can be discarded
  // by long press of button

  else {  // There can be at most three impacts because we neglect any impacts beyong 3
    if (time2 > time1) calDist(time2); 
    else calDist(time1);
  }
  // If there are three impacts, we would assume one extra impact either comes from throwing or because
  // the device bounced on the floor, we would take the larger one between time1 and time2. Reason:
  // Due to the position the device is tossed and an initial acceleration, the time for the device to 
  // travel to the ceiling is definitely shorter than the time it takes to free fall to the floor; the 
  // time taken for the device to bounce is definitely shorter than the time it takes to free fall

  avg = (avg+dist)/cnt; // Average the current distance measured into avg
  impactCnt = 0;        // Reset impact counter
  display(avg);         // Display the average
}

void discardCurrentReading(){ // This function discards the current throw result

  // Animation : two red blinks
  changeColor(13,0,0,100);__delay_cycles(100000);
  changeColor(0,0,0,0);__delay_cycles(100000);
  changeColor(13,0,0,100);__delay_cycles(100000);
  changeColor(0,0,0,0);

  startFlag = 1;  // Get ready to start the next measurement
  time1 = 0;      // Reset time1
  time2 = 0;      // Reset time2
  impactCnt = 0;  // Reset impactCnt
}

void reset(){ 
  // This function resets everything, it clears the average

  // Animation: Consecutive R, G, B blink
  changeColor(13,0,0,100);__delay_cycles(100000);
  changeColor(0,0,0,0);__delay_cycles(100000);
  changeColor(0,13,0,100);__delay_cycles(100000);
  changeColor(0,0,0,0);__delay_cycles(100000);
  changeColor(0,0,13,100);__delay_cycles(100000);
  changeColor(0,0,0,0);

  startFlag = 1;  // Get ready to start the next measurement
  time1 = 0;      // Reset time1
  time2 = 0;      // Reset time2
  cnt = 0;        // Reset total rounds of averaging
  avg = 0;        // Reset average
  impactCnt = 0;  // Reset counter for number of impacts
}



void main(void){
   if (CALBC1_1MHZ == 0xFF || CALDCO_1MHZ == 0xff) while(1); // If calibration data erased, trap

  BCSCTL1 = CALBC1_1MHZ; // Set the DCO to 1 MHz
  DCOCTL = CALDCO_1MHZ; // And load calibration data

  // debouncing timer setting
  TA0CTL = TASSEL_2 | MC_1;
  TA0CCR0 = 40000;    //one cycle count = 1/1M * 1000 = 5ms
  TA0CCTL0 &= ~CCIE;  // disable timer A interrupt until button is pressed

    // Part II. SPI settings

  P1OUT  |=  BIT5;
  P1DIR  |=  BIT5;
  P1SEL  =   BIT1|BIT2|BIT4;  // P1.1 – UCA0SOMI  P1.2 – UCA0SIMO   P1.4 – UCA0CLK
  P1SEL2 =   BIT1|BIT2|BIT4;
  UCA0CTL1   =   UCSWRST;
  UCA0CTL0   |=  UCCKPH  +   UCMSB   +   UCMST   +   UCSYNC;  // 8 bit, choose SPI master
  UCA0CTL1   |=  UCSSEL_1;                    // set clock to SMCLK
  UCA0BR0    |=  0x02;
  UCA0BR1    =   0;
  UCA0MCTL   =   0;                     //  turn off modulation
  UCA0CTL1   =  ~UCSWRST;

  // Watch dog time setting
  WDTCTL = WDT_MDLY_8; // Select interval timer mode, 32ms;
  IE1 |= WDTIE; // Enable WDT interrupt
  //WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

  P2OUT = (BIT0 + BIT2); //p2.0 2.2 set, else resets
  P2REN |= (BIT0 + BIT2); //p2.0 2.2 pulled up
  P2IES |= (BIT0 + BIT2); //edge select, high to low

  P2IE = (BIT0 + BIT2); // enable 2.0 2.2 interrupts
  P2IFG &= ~(BIT0 + BIT2); // clear flags

  // start animation
  changeColor(13,0,0,100);__delay_cycles(100000);
  changeColor(0,0,0,0);__delay_cycles(100000);
  changeColor(0,13,0,100);__delay_cycles(100000);
  changeColor(0,0,0,0);__delay_cycles(100000);
  changeColor(0,0,13,100);__delay_cycles(100000);
  changeColor(0,0,0,0);

  _BIS_SR(GIE + CPUOFF); //GIE for buttons
  //////

}

#pragma vector = PORT2_VECTOR
__interrupt void port2_isr(void){
  port2Flag = P2IFG;
  P2IE &= ~(BIT0 + BIT2);
  TA0CCR0 = 0;   // resetting timer cycle
  TA0CCR0 = 50000;   // start to count 5 ms until timer interrupt
  TA0CCTL0 = CCIE;  // enable timer interrupt
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void timerA0_isr(void){
  TA0CCTL0 &= ~CCIE;  //disable timer interrupt


  /*  --------------- Vibration Switch -----------------------
     * Time 1 - time interval between first and second impact
     * Time 2 - time interval between second and third impact
     * There can be multiple impacts depending on whether the vibration sensor is triggered upon thowing,
     * which is a 50%-50% event, and whether the device bounces upon falling to the floor
     * 
     * We assume the switch will definitely be triggered upon impact on the ceiling and floor. And the impact
     * on ceiling and floor will definitely occur within three impacts
     */

  if((port2Flag & BIT0) && (P2IES & BIT0)){

    if (startFlag == 1){
      if (impactCnt <= 3){  // The total number of impacts would be < 3
        time2 = timer2; // Record time2, between second and third impact
        timer1 = 0;   // Initializa timer1 to record first and second impact
        startFlag = 0;  // Clear startFlag, marking the start of timer1
        impactCnt ++;   // Increment impactCnt
      }
    } else {  //hit signaling
      if (impactCnt <= 3){
        time1 = timer1; // Record time1, between first and second impact
        timer2 = 0;   // Initialize timer2 to record second and third impact
        startFlag = 1;  // Clear startFlag, get ready for the next timer1
        impactCnt ++; // Increment impactCnt
      }
    }
    __delay_cycles(100000);//added debounce time
    // debounce time, used delay_cycles because this interval must be different from button debounce using timer
  }

 /*  --------------- Push Button -----------------------
     * Time 1 - time interval between first and second impact
     * Time 2 - time interval between second and third impact
     */

  if((port2Flag & BIT2) && (P2IES & BIT2)){
    P2IES &= ~BIT2;
    if (fastPressTimer<40) {  // if two consecutive press has time interval smaller than 0.4s, this is considered a double click
      reset();
      resetFlag = 1;  // set the resetFlag, hence "discardCurrentReading()" will not be error triggered
    }
    longPressTimer = 0;
  } else if((port2Flag & BIT2) && !(P2IES & BIT2)){ //release button actions
    P2IES |= BIT2;
    if ((longPressTimer>150) && (resetFlag != 1)) { // if pressed for longer than 1.5s, discard current reading
      discardCurrentReading();
      resetFlag = 0;
    } else if (longPressTimer>40){  // if pressed longer than 0.4s, take reading and average
      calAverage();
    }
    resetFlag = 0;      // reset resetFlag
    fastPressTimer = 0; // reset fastPressTimer
  }

  P2IFG &= ~(BIT0 + BIT2); //clear flag
  P2IE = (BIT0 + BIT2);   //enable button interrupt

}

#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void) {
  timer1++;         // increment timer1
  timer2++;         // increment timer2
  longPressTimer++; // increment longPressTimer
  fastPressTimer++; // increment fastPressTimer
}
