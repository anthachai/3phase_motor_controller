

// DDS Sine Generator 3 phase ATMEGA 168 328P PWM 4KHZ + Danijel Gorupec, 2015
// Support 7 Segments Show Hz  15/07/2017
// Import SevSeg Library : https://playground.arduino.cc/Main/SevenSegmentLibrary

#include "arduino.h" //Store data in flash (program) memory instead of SRAM
#include "avr/pgmspace.h"
#include "avr/io.h"
// include the SevSeg.h for using the 7 segments
//#include "SevSeg.h"

// include the LiquidCrystal_I2C for using the 16x2 LCD
#include <LiquidCrystal_I2C.h>

// declare the 16x2 LCD variable
LiquidCrystal_I2C lcd(0x27, 16, 2);
//variable to compare the frequency digit to clear the screen, when it change down from two to one digit
int curNumDigit, oldNumDigit;

//SevSeg sevseg; //Instantiate a seven segment controller object

   const byte sine256[] PROGMEM  = {
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,221,223,225,227,229,231,233,234,236,238,239,240,
  242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,
  221,219,217,215,212,210,208,205,203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,
  76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,
  33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124

};
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) //define a bit to have the properties of a clear bit operator
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))//define a bit to have the properties of a set bit operator
#define INPUT_DIR ((PINC&0x04)==0)  // 00000010 = A2 //Control Direction


int PWM1=  9;  //PWM1 output, phase 1
int PWM2 = 10; //PWM2 output, phase 2
int PWM3 = 11; //PWM3 output, phase 3


//int offset_1 = 85; //offset 1 is 120 degrees out of phase with previous phase, Refer to PWM to sine.xls
//int offset_2 = 170; //offset 2 is 120 degrees out of phase with offset 1. Refer to PWM to sine.xls
int offset_1; //offset 1 is 120 degrees out of phase with previous phase, Refer to PWM to sine.xls
int offset_2; //offset 2 is 120 degrees out of phase with offset 1. Refer to PWM to sine.xls
//int program_exec_time = A3; //monitor how quickly the interrupt trigger
int ISR_exec_time = A12; //monitor how long the interrupt takes
int INVERTOR_ENABLE = A1; //INVERTOR ENABLE

double ad_cel; //Manat Add Motor Acceleration, Deceleration
double spd_ref;  //Manat Add
double spd_ref_max = 481;  //60Hz Manat Add
double spd_ref_min = 20;  //2.5Hz Manat Add
double speed;   // Manat Add
unsigned char direction; // Manat Add rotation direction (0 forwared, 1 reverse)
unsigned char run;  //Manat Add
int num = 0; // Manat Add for Hz Show 7-Segments

const double refclk=31376.6;      // measured output frequency
//----------------------------------------------------------------------------
const int ledPin =  13;         // the number of the LED pin
int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
long interval = 50000;           // interval at which to blink (milliseconds)
//----------------------------------------------------------------------------

// variables used inside interrupt service declared as voilatile
volatile byte current_count;              // Keep track of where the current count is in sine 256 array
volatile byte ms4_delay;             //variable used to generate a 4ms delay
volatile byte c4ms;              // after every 4ms this variable is incremented, its used to create a delay of 1 second
volatile unsigned long phase_accumulator;   // pahse accumulator
volatile unsigned long tword_m;  // dds tuning word m, refer to DDS_calculator (from Martin Nawrath) for explination.

void setup()
{
  Serial.begin(9600);        // Manat Add

  // initialize the 16x2 LCD
  lcd.begin();
  oldNumDigit = 0;
  // print the starting wellcome screen
  lcd.setCursor(0,0);
  lcd.print("Motor Controller");
  lcd.setCursor(0,1);
  lcd.print(" KAMSAI IKAMSAI ");
  
  pinMode(PWM1, OUTPUT);      //sets the digital pin as output
  pinMode(PWM2, OUTPUT);      //sets the digital pin as output
  pinMode(PWM3, OUTPUT);  //sets the digital pin as output
  
  pinMode(ledPin, OUTPUT);  //Manat Add
  pinMode(ISR_exec_time, OUTPUT);      //sets the digital pin as output
  pinMode(INVERTOR_ENABLE, OUTPUT);      //sets the digital pin as output
  digitalWrite(INVERTOR_ENABLE, LOW);  //Manat Add
 
  
  
  //sbi(PORTB,program_exec_time); //Sets the pin
  //digitalWrite(program_exec_time, HIGH);
  
  Setup_timer0();
  Setup_timer1();
  Setup_timer2();
  //Disable Timer 1 interrupt to avoid any timing delays
  //cbi (TIMSK0,TOIE0);              //disable Timer0 !!! delay() is now not available
  sbi (TIMSK2,TOIE2);              //enable Timer2 Interrupt
  
  
  tword_m=pow(2,32)*speed/refclk;  //calulate DDS new tuning word 
  /*
  //-----------SevenSegment-------------
  byte numDigits = 2;
  byte digitPins[] = {13, 12};
  byte segmentPins[] = {8, 7, 6, 5, 4, 3, 2};
  bool resistorsOnSegments = false; // 'false' means resistors are on digit pins
  byte hardwareConfig = COMMON_CATHODE; // See README.md for options
  bool updateWithDelays = false; // Default. Recommended
  bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
  
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments, updateWithDelays, leadingZeros);
  sevseg.setBrightness(1);
  //------------------------------------
  */
  digitalWrite(ledPin, HIGH);  
  WaitLoop(30000);
  digitalWrite(ledPin, LOW);  
  lcd.clear();
}

void loop()
{
  while(1) 
  { 

// DDS Sine Generator 3 phase ATMEGA 168 328P PWM 4KHZ + Danijel Gorupec, 2015
// Support 7 Segments Show Hz  15/07/2017
// Import SevSeg Library : https://playground.arduino.cc/Main/SevenSegmentLibrary

#include "arduino.h" //Store data in flash (program) memory instead of SRAM
#include "avr/pgmspace.h"
#include "avr/io.h"
#include "string.h"

// include the SevSeg.h for using the 7 segments
//#include "SevSeg.h"

// include the LiquidCrystal_I2C for using the 16x2 LCD
#include <LiquidCrystal_I2C.h>

// declare the 16x2 LCD variable
LiquidCrystal_I2C lcd(0x27, 16, 2);
//variable to compare the frequency digit to clear the screen, when it change down from two to one digit
int curNumDigit, oldNumDigit;

//SevSeg sevseg; //Instantiate a seven segment controller object

   const byte sine256[] PROGMEM  = {
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,221,223,225,227,229,231,233,234,236,238,239,240,
  242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,
  221,219,217,215,212,210,208,205,203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,
  76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,
  33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124

};
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) //define a bit to have the properties of a clear bit operator
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))//define a bit to have the properties of a set bit operator
#define INPUT_DIR ((PINC&0x04)==0)  // 00000010 = A2 //Control Direction


int PWM1=  9;  //PWM1 output, phase 1
int PWM2 = 10; //PWM2 output, phase 2
int PWM3 = 11; //PWM3 output, phase 3


//int offset_1 = 85; //offset 1 is 120 degrees out of phase with previous phase, Refer to PWM to sine.xls
//int offset_2 = 170; //offset 2 is 120 degrees out of phase with offset 1. Refer to PWM to sine.xls
int offset_1; //offset 1 is 120 degrees out of phase with previous phase, Refer to PWM to sine.xls
int offset_2; //offset 2 is 120 degrees out of phase with offset 1. Refer to PWM to sine.xls
//int program_exec_time = A3; //monitor how quickly the interrupt trigger
int ISR_exec_time = 12; //monitor how long the interrupt takes
int INVERTOR_ENABLE = A1; //INVERTOR ENABLE

double ad_cel; //Manat Add Motor Acceleration, Deceleration
double spd_ref;  //Manat Add
double spd_ref_max = 481;  //60Hz Manat Add
double spd_ref_min = 20;  //2.5Hz Manat Add
double m_speed;   // Manat Add
unsigned char m_direction; // Manat Add rotation direction (0 forwared, 1 reverse)
unsigned char m_run;  //Manat Add
int num = 0; // Manat Add for Hz Show 7-Segments

const double refclk=31376.6;      // measured output frequency
//----------------------------------------------------------------------------
const int ledPin =  13;         // the number of the LED pin
int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
long interval = 50000;           // interval at which to blink (milliseconds)
//----------------------------------------------------------------------------

// variables used inside interrupt service declared as voilatile
volatile byte current_count;              // Keep track of where the current count is in sine 256 array
volatile byte ms4_delay;             //variable used to generate a 4ms delay
volatile byte c4ms;              // after every 4ms this variable is incremented, its used to create a delay of 1 second
volatile unsigned long phase_accumulator;   // pahse accumulator
volatile unsigned long tword_m;  // dds tuning word m, refer to DDS_calculator (from Martin Nawrath) for explination.

void setup()
{
  Serial.begin(9600);        // Manat Add

  // initialize the 16x2 LCD
  lcd.begin();
  oldNumDigit = 0;
  // print the starting wellcome screen
  lcd.setCursor(0,0);
  lcd.print("Motor Controller");
  lcd.setCursor(0,1);
  lcd.print(" KAMSAI IKAMSAI ");
  
  pinMode(PWM1, OUTPUT);      //sets the digital pin as output
  pinMode(PWM2, OUTPUT);      //sets the digital pin as output
  pinMode(PWM3, OUTPUT);  //sets the digital pin as output
  
  pinMode(ledPin, OUTPUT);  //Manat Add
  pinMode(ISR_exec_time, OUTPUT);      //sets the digital pin as output
  pinMode(INVERTOR_ENABLE, OUTPUT);      //sets the digital pin as output
  digitalWrite(INVERTOR_ENABLE, LOW);  //Manat Add
 
  
  
  //sbi(PORTB,program_exec_time); //Sets the pin
  //digitalWrite(program_exec_time, HIGH);
  
  Setup_timer0();
  Setup_timer1();
  Setup_timer2();
  //Disable Timer 1 interrupt to avoid any timing delays
  //cbi (TIMSK0,TOIE0);              //disable Timer0 !!! delay() is now not available
  sbi (TIMSK2,TOIE2);              //enable Timer2 Interrupt
  
  
  tword_m=pow(2,32)*m_speed/refclk;  //calulate DDS new tuning word 
  /*
  //-----------SevenSegment-------------
  byte numDigits = 2;
  byte digitPins[] = {13, 12};
  byte segmentPins[] = {8, 7, 6, 5, 4, 3, 2};
  bool resistorsOnSegments = false; // 'false' means resistors are on digit pins
  byte hardwareConfig = COMMON_CATHODE; // See README.md for options
  bool updateWithDelays = false; // Default. Recommended
  bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
  
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments, updateWithDelays, leadingZeros);
  sevseg.setBrightness(1);
  //------------------------------------
  */
  digitalWrite(ledPin, HIGH);  
  WaitLoop(30000);
  digitalWrite(ledPin, LOW);  
  lcd.clear();
}

void loop()
{
  while(1) 
  { 
     ReadAnalogs();
     unsigned long currentMillis = millis();    //  For ledState
     
   //---------Control Power IR2111----------------------------
     if (m_speed > spd_ref_min){
       offset_1 = 85;
       offset_2 = 170;
       m_run = 1;
       digitalWrite(INVERTOR_ENABLE, HIGH);
     }
     else {
       offset_1 = 0;
       offset_2 = 0;
       m_run = 0;
       digitalWrite(INVERTOR_ENABLE, LOW);
     }
   //---------7 Segments Show Hz------------------------------  
     num = (m_speed/8);
     if(m_speed == spd_ref_min) num = 0;
     
     lcd.setCursor(0,0);
     lcd.print("Frequency :");
     curNumDigit = len(num);
     if(curNumDigit < oldNumDigit)
       {
          lcd.setCursor(14,0);
          lcd.print("  ");
       }
     oldNumDigit = curNumDigit;
     lcd.setCursor(14,0);
     lcd.print(num);
     
     // remove the comment if using the 7 segments
     //sevseg.setNumber(num, 2);
     //sevseg.refreshDisplay();    
     
   //sbi(PORTC,program_exec_time); //Sets the pin 
   //digitalWrite(program_exec_time, HIGH);
   
   //---------Monitor program---------------------------------
  if((currentMillis - previousMillis) > (interval/(num+1))) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
   
   //---------------------------------------------------------------
     
      if (c4ms > 0) // c4ms = 4ms, thus 4ms *250 = 1 second delay
       {                 
        c4ms=0;                          //Reset c4ms
        cbi (TIMSK2,TOIE2);              //Disable Timer2 Interrupt
        tword_m=pow(2,32)*m_speed/refclk;  //Calulate DDS new tuning word
        sbi (TIMSK2,TOIE2);              //Enable Timer2 Interrupt 

      }
      
  }
}

void WaitLoop(unsigned int tm)
{
  unsigned int i,j;
  for (j=0;j<tm;j++)
  {
    for (i=0;i<200;i++) //the ATmega is runs at 16MHz
      if (PORTC==0xFF) DDRB|=0x02; //just a dummy instruction
  }
}


void ReadAnalogs(void)
{
      spd_ref=map(analogRead(0),0,1023,0,spd_ref_max);             //Read voltage on analog 1 to see desired output frequency, 0V = 0Hz, 5V = 1.023kHz
      ad_cel=map(analogRead(3),0,1023,1,200);    // Manat Add
      
       if(spd_ref > spd_ref_max) spd_ref = spd_ref_max;  // Manat add maximum 60Hz  
       if(spd_ref < spd_ref_min) spd_ref = 0;  // Manat Add minimum 2.5Hz
  

       if (INPUT_DIR)
        {
          if (m_direction==0) spd_ref=spd_ref_min;
          if (m_speed==spd_ref_min) m_direction=1; //only allow direction change at minimum speed
          
        }
  
       else
       {
          if (m_direction==1) spd_ref=spd_ref_min;
          if (m_speed==spd_ref_min) m_direction=0; //only alow direction change at minimum speed
          
        }  

     //if (spd_ref>m_speed) m_speed=m_speed+0.02;   // Hz step
     //if (spd_ref<m_speed) m_speed=m_speed-0.02;
     if (spd_ref>m_speed) m_speed=m_speed+(1/ad_cel);   // Hz step
     if (spd_ref<m_speed) m_speed=m_speed-(1/ad_cel);
     if (m_speed<spd_ref_min) m_speed=spd_ref_min;
  
}


void Setup_timer0(void)
{
  
  
  TCCR0B = (TCCR0B & 0b11111000) | 0x02;
  // Timer1 PWM Mode set to Phase Correct PWM
  cbi (TCCR0A, COM0A0);
  sbi (TCCR0A, COM0A1);
  cbi (TCCR0A, COM0B0); 
  sbi (TCCR0A, COM0B1);

  // Mode 1 / Phase Correct PWM
  sbi (TCCR0A, WGM00); 
  cbi (TCCR0A, WGM01);
  
}

void Setup_timer1(void)
{
  
 TCCR1B = (TCCR1B & 0b11111000) |0x02;
  // Timer1 Clock Prescaler to : 1
  
  cbi (TCCR1A, COM1A0);
  sbi (TCCR1A, COM1A1);
  cbi (TCCR1A, COM1B0); 
  sbi (TCCR1A, COM1B1);

 
  sbi (TCCR1A, WGM10); 
  cbi (TCCR1A, WGM11);
  cbi (TCCR1B, WGM12);
  cbi (TCCR1B, WGM13);
}


void Setup_timer2() 
{
  
  TCCR2B = (TCCR2B & 0b11111000) | 0x02;// Timer2 Clock Prescaler to : 1
 
  cbi (TCCR2A, COM2A0);  // clear Compare Match
  sbi (TCCR2A, COM2A1);
  cbi (TCCR2A, COM2B0); 
  sbi (TCCR2A, COM2B1);
  
  // Mode 1  / Phase Correct PWM
  sbi (TCCR2A, WGM20);  
  cbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);
}



ISR(TIMER2_OVF_vect)
{
  //cbi(PORTC,program_exec_time); //Clear the pin
  //sbi(PORTC,ISR_exec_time);          // Sets the pin
  //digitalWrite(program_exec_time, LOW);
  digitalWrite(ISR_exec_time, HIGH);
  if (m_direction==0) 
     phase_accumulator=phase_accumulator+tword_m;
  else
     phase_accumulator=phase_accumulator-tword_m; 
     
   //phase_accumulator=phase_accumulator+tword_m; //Adds tuning M word to previoud phase accumulator. refer to DDS_calculator (from Martin Nawrath) for explination. 
   
  if (m_run==0)
    current_count=0; 
  else
    current_count=phase_accumulator >> 24;     // use upper 8 bits of phase_accumulator as frequency information 
  //-------------------------------

 
  //------------------------------
  
  OCR1A = pgm_read_byte_near(sine256 + current_count); // read value fron ROM sine table and send to PWM
  OCR1B = pgm_read_byte_near(sine256 + (uint8_t)(current_count + offset_1)); // read value fron ROM sine table and send to PWM, 120 Degree out of phase of PWM1
  OCR2A = pgm_read_byte_near(sine256 + (uint8_t)(current_count + offset_2));// read value fron ROM sine table and send to PWM, 120 Degree out of phase of PWM2
  
  //increment variable ms4_delay every 4mS/125 =  milliseconds 32uS
  if(ms4_delay++ == 125)
  
  {  
    c4ms++;
    ms4_delay=0; //reset count
   }   

//cbi(PORTC,ISR_exec_time);            //Clear the pin
digitalWrite(ISR_exec_time, LOW);
}

int len(int number)
{
  int count=0;
  while(number)
     {        
            number=number/10;
            count++;
      }
   return count;
}

