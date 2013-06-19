/*******************************************************************/
/* Based on WS2801 control code by:                                */
/* Greg Whitmore                                                   */
/* greg@gwdeveloper.net                                            */
/* www.gwdeveloper.net                                             */
/*******************************************************************/
/* Modified to control LPD8806-based strips by:                    */
/* Paul Nicholls                                                   */
/* https://github.com/MaxThrax/MSP430-LPD8806-RGB-LED-Strip        */
/*******************************************************************/
/* released under the "Use at your own risk" license               */
/* use it how you want, where you want and have fun                */
/* debugging the code.                                             */
/* MSP430 spi bit bang WS2801 RGB LED strip                        */
/*******************************************************************/
/* WS2801 code was translated from Adafruit WS2801 Arduino library */
/* https://github.com/adafruit/WS2801-Library                      */
/* LPD8806 conversion based on Adafruit LPD806 Arduino library:    */
/* https://github.com/adafruit/LPD8806                             */
/*******************************************************************/

#include <msp430.h>
//#include <legacymsp430.h>

// LED Left side
#define DATA_L    BIT5
#define CLOCK_L   BIT4
#define NUMLEDS_L 20

// LED Right side
#define DATA_R    BIT6
#define CLOCK_R   BIT7
#define NUMLEDS_R 20

// Synchronized
#define DATA (DATA_R|DATA_L)
#define CLOCK (CLOCK_R|CLOCK_L)
#define NUMLEDS 20

//Controller Inputs
#define M_PHASEA BIT0
#define M_PHASEB BIT1
#define M_BOTH (M_PHASEA | M_PHASEB)
#define S_PHASEA BIT5
#define S_PHASEB BIT4
#define S_BOTH (S_PHASEA | S_PHASEB)

#define ON_SWITCH BIT3

#define FWD 1
#define BKW -1

// wdt delay constants
#define MCLK_FREQUENCY      1000000
#define WDT_DIVIDER         512

const unsigned long WDT_FREQUENCY = MCLK_FREQUENCY / WDT_DIVIDER;
volatile unsigned long wdtCounter = 0;

// data arrays
unsigned long pixels_left[NUMLEDS_L];
unsigned long pixels_right[NUMLEDS_R];
int speed = 0;
#define SPEED_SLOW 100
#define SPEED_INC 1

// encoder data
int m_encoder_state = 0;
int m_direction_ctr = 0;
int s_encoder_state = 0;
int s_direction_ctr = 0;

//incrementers
int p;
long i;

//other state variables
enum {
  CHASE = 0,
  RANDOM,
  RAINBOW,
  BLINK,
  MODE_COUNT
} led_mode;

unsigned int analog_val; //for pot input on ADC pin

// prototypes
void init(void);
void display(void);
unsigned long color(unsigned char r, unsigned char g, unsigned char b);
void setPixelS(unsigned int n, unsigned long c);
void setPixel(unsigned int n, unsigned char r, unsigned char g, unsigned char b);
unsigned long wheel(unsigned char wheelpos);

// pattern functions
void demos(void);
void lbgg_chase(void);
void randomchase(void);
void copcar(void);
void goJoe(unsigned long time); // larger time value is slower chase
void randomdance(void);
void solidblink(unsigned long c);
void colorwipe(unsigned long c);
void rainbowcycle(void);
void showrainbow(void);

void check_mode_encoder(void);
void check_speed_encoder(void);
void check_switches(void);
void update_leds(void);

// random function and delay millis borrowed from NatureTM.com
// random generator slightly modified to create 32bit value
unsigned long adcGenRand24(void);
void delayMillis(unsigned long milliseconds);

// main colors
unsigned long clear = 0x808080 | 0x000000;
unsigned long red = 0x808080 | 0x00FF00;
unsigned long green = 0x808080 | 0xFF0000;
unsigned long blue = 0x808080 | 0x0000FF;
unsigned long white = 0x808080 | 0xFFFFFF;

unsigned long randomcolor;

void main(void) {
  //WDTCTL = WDTPW + WDTHOLD;
  
  // use 1MHz calibrated values
  DCOCTL = CALDCO_1MHZ;
  BCSCTL1 = CALBC1_1MHZ;
  // wdt set as interval
  //WDTCTL = WDTPW + WDTTMSEL + WDTIS1;
  // wdt interrupt
  //  IE1 |= WDTIE;
  // enable global interrupts using intrinsic
  //__enable_interrupt();
  
  // initialize pins for SPI
  init();
  led_mode = CHASE;
  
  colorwipe(clear); // clear led strip
  //delayMillis(1000);
  while (1) { 
    //demos();
    //check switches
    check_switches();
    //update LEDs
    update_leds();
  }

}

/* use functions */

//check switches
void check_switches(void) {

  while (P2IN & ON_SWITCH) colorwipe(clear); // on or off based on switch

  check_mode_encoder();
  led_mode += m_direction_ctr;
  m_direction_ctr = 0;
  while (led_mode >= MODE_COUNT) led_mode -= MODE_COUNT;
  while (led_mode < 0) led_mode += MODE_COUNT;

  check_speed_encoder();
  speed += s_direction_ctr;
  s_direction_ctr = 0;
  while (speed > SPEED_SLOW) speed = SPEED_SLOW;
  while (speed < 0) speed = 0;

}

//update LEDs
void update_leds (void) {

  switch (led_mode){
  case CHASE:
    //lbgg_chase();
    copcar();
    break;
  case RANDOM:
    randomdance();
    break;
  case RAINBOW:
    rainbowcycle();
    break;
  case BLINK:
    solidblink(green);
  default: break;
  }

}

// random chase
void randomchase(void) {
  int m;
  for ( m = 0; m <= NUMLEDS; m++ ) {
    setPixelS(m, adcGenRand24());
    setPixelS(m-1, clear);
    display();
    delayMillis(100);
  }
  for ( m = NUMLEDS; m >= 0; m-- ) {
    setPixelS(m, adcGenRand24());
    setPixelS(m+1, clear);
    display();
    delayMillis(100);
  }
}

//LBGG color chase
void lbgg_chase(void) {
  int m;

  int time = analog_val; //TODO: do some math here

  //colorwipe(clear); do this in switch logic
  
  for ( m = 0; m < NUMLEDS; m++ ) {
    setPixelS(m, green);
    setPixelS(m - 2, white);
    setPixelS(m - 4, green);
    setPixelS(m - 6, clear);
    display();
    delayMillis(time);
  }
  for ( m = NUMLEDS; m >= 0; m-- ) {
    setPixelS(m, clear);
    setPixelS(m - 2, green);
    setPixelS(m - 4, white);
    setPixelS(m - 6, green);
    display();
    delayMillis(time);
  }
}

// police light bar chaser
void copcar(void) {
  int m;
  int middle = NUMLEDS / 2 ;

  colorwipe(clear);

  for ( m = 0; m < NUMLEDS; m++ ) {
    if ( m <= middle ) {
      setPixelS(m, red);
      setPixelS(m - 2, clear);

      setPixelS(NUMLEDS - m, blue);
      setPixelS(NUMLEDS - m + 2, clear);
    }
    display();

    if  ( m >= middle ) {
      setPixelS(m, blue);
      setPixelS(m - 2, clear);
  
      setPixelS(NUMLEDS - m, red);
      setPixelS(NUMLEDS - m + 2, clear);
    }
    display();
  }
}

// red white and blue chasers
void goJoe(unsigned long time) {
  int m;

  colorwipe(clear); // clear display from existing patterns
    
  for ( m = 0; m < NUMLEDS; m++ ) {
    setPixelS(m, blue);
    setPixelS(m - 2, red);
    setPixelS(m - 4, white);
    setPixelS(m - 6, clear);
    display();
    delayMillis(time);
  }
  for ( m = NUMLEDS; m >= 0; m-- ) {
    setPixelS(m, clear);
    setPixelS(m - 2, white);
    setPixelS(m - 4, red);
    setPixelS(m - 6, blue);
    display();
    delayMillis(time);
  }
}

// send random colors down each pixel
void randomdance(void) {
  int m;
    
    for ( m = 0; m < NUMLEDS; m++ ) {
      setPixelS(m, adcGenRand24());
      display();
    }
}

void solidblink(unsigned long c) {
  colorwipe(c);
  delayMillis(500);
  colorwipe(clear);
  delayMillis(500);
}

// animate fading rainbow cycle
void rainbowcycle(void) {
  int k, j;
  
  for ( j=0; j<256; j++ ) {
    for ( k=0; k < NUMLEDS; k++ ) {
      setPixelS(k, wheel( ( k+j) % 255 ) );
    }
    display();
    delayMillis(100);
  }
}

// display static rainbow
void showrainbow(void) {
  int k;
  
  for ( k=0; k < NUMLEDS; k++ ) {
    setPixelS(k, wheel( ((k * 256 / NUMLEDS )) % 255) );
  }
  display();
  delayMillis(100);
}
      
// wipe strip to selected color
void colorwipe(unsigned long c) {
  int v;
  
  for ( v=0; v < NUMLEDS; v++)
    setPixelS(v, c);
  display();
    //delayMillis(100);
}

// run all functions as demo
void demos(void) {
  int x;
    
// run demos for display
    
  for (x = 0; x < 3; x++) {
    lbgg_chase();
  }
    
  /* for (x = 0; x < 3; x++) {
    goJoe(50);
    }*/

  for (x = 0; x < 5; x++) {
    randomdance();
  }

  colorwipe(clear);
    
  for (x = 0; x < 3; x++) {
    solidblink(green);
  }

  for (x = 0; x < 2; x++) {
    rainbowcycle();
  }
}

/* library functions */

// write a bunch of zeroes, used for latch
void writezeros(unsigned int n) {
  unsigned int i;
  P1OUT &= ~DATA; // Data low
  for(i = 8 * n; i>0; i--) {
    P1OUT |= CLOCK;
    P1OUT &= ~CLOCK;
  }
}

//initialization
void init(void) {
  int i;

  __disable_interrupt(); //because fuck that shit

  WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

  P1DIR |= DATA + CLOCK; // set data and clock pins to output
  P1OUT &= ~DATA; // Data low
  P1OUT &= ~CLOCK;
  for(i=0; i<NUMLEDS; i++) {
    pixels_left[i] = pixels_right[i] = 0x808080;
  }
  writezeros(3 * ((NUMLEDS + 63) / 64)); // latch to wake it up

    //encoder
    P2DIR = 0; //all input
    P2REN = M_PHASEA | M_PHASEB | S_PHASEA | S_PHASEB | ON_SWITCH; //pull-up resistors

}

// send data to led strip; create patten with a 'use' function then send with display
void display_left(void) {
  unsigned long data;
    
    // send all the pixels
    for ( p=0; p < NUMLEDS_L ; p++ ) {
      data = pixels_left[p];
      // 24 bits of data per pixel
      for ( i=0x800000; i>0 ; i>>=1 ) {
        if (data & i) {
            P1OUT |= DATA_L;
        } else {
            P1OUT &= ~DATA_L;
        }
        P1OUT |= CLOCK_L;    // latch on clock rise
        P1OUT &= ~CLOCK_L;
      }
    }
    writezeros(3 * ((NUMLEDS_L + 63) / 64)); // latch
    delayMillis(3);
}

void display_right(void) {
  unsigned long data;
    
    // send all the pixels
    for ( p=0; p < NUMLEDS_R ; p++ ) {
      data = pixels_right[p];
      // 24 bits of data per pixel
      for ( i=0x800000; i>0 ; i>>=1 ) {
        if (data & i) {
            P1OUT |= DATA_R;
        } else {
            P1OUT &= ~DATA_R;
        }
        P1OUT |= CLOCK_R;    // latch on clock rise
        P1OUT &= ~CLOCK_R;
      }
    }
    writezeros(3 * ((NUMLEDS_R + 63) / 64)); // latch
    delayMillis(3);
}

void display(void) {
  display_left();
  display_right();
}

// create 24bit color value
unsigned long color(unsigned char r, unsigned char g, unsigned char b) {
  unsigned long c;
  
  c = 0x808080 | ((unsigned long)g << 16) | ((unsigned long)r << 8) | (unsigned long)b;
  return c;
}
// create color value from an unsigned long, so you can use colorHex(0xABCDEF);
unsigned long colorHex(unsigned long hex) {
  return 0x808080 | hex;
}

// set pixel to specified color
void setPixel(unsigned int n, unsigned char r, unsigned char g, unsigned char b) {
  if ( n > NUMLEDS ) return;
  
  pixels_left[n] = pixels_right[n] = color(r, g, b);
}

//set pixel to color by function
void setPixelS(unsigned int n, unsigned long c) {
  if ( n > NUMLEDS ) return;
  
  pixels_left[n] = pixels_right[n] = c;
}

// rotate colorwheel for rainbows
unsigned long wheel(unsigned char wheelpos) {
  if (wheelpos <=85) {
    return color( wheelpos * 3, 255 - wheelpos * 3, 0 );
  }
  else if ( wheelpos < 170 ) {
    return color( 255 - wheelpos * 3, 0, wheelpos * 3 );
  }  else {
    wheelpos -= 170;
    return color( 0, wheelpos * 3, 255 - wheelpos * 3 );
  }
}

// generate random 24bit number using ADC10 channel 5; leave P1.4 & P1.5 floating
unsigned long adcGenRand24(void) {
  char bit;
  unsigned long random;
  
  for(bit = 0; bit < 24; bit++) {
    ADC10CTL1 |= INCH_5;
    ADC10CTL0 |= SREF_1 + ADC10SHT_1 + REFON + ADC10ON;
    ADC10CTL0 |= ENC + ADC10SC;
    while(ADC10CTL1 & ADC10BUSY);
    random <<= 1;
    random |= (ADC10MEM & 0x01);
  }
  return random | 0x808080;
}

// millisecond delay counter using WDT
void delayMillis(unsigned long milliseconds) {

  int ix;
  for (ix = 0; ix < (milliseconds * speed * SPEED_INC); ++ix) {
    //__NOP();
    int dummy = m_direction_ctr * i;
  }
  
  //return;
  //unsigned long wakeTime = wdtCounter + (milliseconds * WDT_FREQUENCY / 1000);
  //while(wdtCounter < wakeTime);
}

// wdt isr
/*interrupt(WDT_VECTOR) watchdog_timer(void) {
  wdtCounter++;
  }*/

void check_mode_encoder(void) {
  int m_new_state = 0;
  //check input state
  if (P2IN & M_PHASEA) m_new_state |= M_PHASEA;
  if (P2IN & M_PHASEB) m_new_state |= M_PHASEB;

  //compare against old state and update
  switch(m_encoder_state)
  {
  case 0:
    if (m_new_state == M_PHASEA) m_direction_ctr = FWD;
    if (m_new_state == M_PHASEB) m_direction_ctr = BKW;
    break;
  case M_PHASEA:
    if (m_new_state == M_BOTH) m_direction_ctr = FWD;
    if (m_new_state == 0) m_direction_ctr = BKW;
    break;
  case M_PHASEB:
    if (m_new_state == 0) m_direction_ctr = FWD;
    if (m_new_state == M_BOTH) m_direction_ctr = BKW;
    break;
  case M_BOTH:
    if (m_new_state == M_PHASEB) m_direction_ctr = FWD;
    if (m_new_state == M_PHASEA) m_direction_ctr = BKW;
    break;
  default:
    break;

  }

  m_encoder_state = m_new_state;
}

void check_speed_encoder(void) {
  int s_new_state = 0;
  //check input state
  if (P2IN & S_PHASEA) s_new_state |= S_PHASEA;
  if (P2IN & S_PHASEB) s_new_state |= S_PHASEB;

  //compare against old state and update
  switch(s_encoder_state)
  {
  case 0:
    if (s_new_state == S_PHASEA) s_direction_ctr = FWD;
    if (s_new_state == S_PHASEB) s_direction_ctr = BKW;
    break;
  case S_PHASEA:
    if (s_new_state == S_BOTH) s_direction_ctr = FWD;
    if (s_new_state == 0) s_direction_ctr = BKW;
    break;
  case S_PHASEB:
    if (s_new_state == 0) s_direction_ctr = FWD;
    if (s_new_state == S_BOTH) s_direction_ctr = BKW;
    break;
  case S_BOTH:
    if (s_new_state == S_PHASEB) s_direction_ctr = FWD;
    if (s_new_state == S_PHASEA) s_direction_ctr = BKW;
    break;
  default:
    break;

  }

  s_encoder_state = s_new_state;
}
