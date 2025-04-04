//Author: Colby Gramelspacher
#include <LiquidCrystal.h>
#include <dht.h>
#include <Stepper.h>
#include <RTClib.h>

//SBn means Set Bit n
//MBn means Mask Bit n

#define SB0 0x01
#define SB1 0x02
#define SB2 0x04
#define SB3 0x08
#define SB4 0x10
#define SB5 0x20
#define SB6 0x40
#define SB7 0x80

#define MB0 0xFE
#define MB1 0xFD
#define MB2 0xFB
#define MB3 0xF7
#define MB4 0xEF
#define MB5 0xDF
#define MB6 0xBF
#define MB7 0x7F

#define RDA 0x80
#define TBE 0x20

volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e = (unsigned char*) 0x2D;
volatile unsigned char* pin_e = (unsigned char*) 0x2C;

volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* pin_h = (unsigned char*) 0x100;

volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;

volatile unsigned char* port_d = (unsigned char*) 0x2B;
volatile unsigned char* ddr_d = (unsigned char*) 0x2A;
volatile unsigned char* pin_d = (unsigned char*) 0x29;

volatile unsigned char* port_g = (unsigned char*) 0x34;
volatile unsigned char* ddr_g = (unsigned char*) 0x33;
volatile unsigned char* pin_g = (unsigned char*) 0x32;

volatile unsigned char* port_c = (unsigned char*) 0x28;
volatile unsigned char* ddr_c = (unsigned char*) 0x27;
volatile unsigned char* pin_c = (unsigned char*) 0x26;

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

dht dht1;

RTC_DS3231 rtc;

unsigned long previousMill;
const long max_interval = 1000;

enum States{
  IDLE = 0,
  RUNNING,
  ERROR,
  DISABLE
};

enum LED{
  GREEN = 0,
  RED,
  BLUE,
  YELLOW
};

//State and flags
volatile States currentState;
volatile LED currentLED;
volatile int enabled_flag = 0;
volatile int error_flag = 0;

const int stepsPerRev = 2038;
Stepper myStepper = Stepper(stepsPerRev, 30, 32, 31, 33);

///////////////////////////////////////////ISR///////////////////////////////////////////////
#pragma region 
void Toggle_Enable()
{
  if(currentState == DISABLE) {
    currentState = IDLE;
    enabled_flag = 1;
  }
  else {
    if(currentState == RUNNING) StopFan();
    currentState = DISABLE;
    enabled_flag = 0;
    error_flag = 0;
  }
}
#pragma endregion
///////////////////////////////////SETUP AND LOOP////////////////////////////////////////////////
void setup()
{
  U0init(9600);
  adc_init();
  lcd.begin(16, 2); // set up number of columns and rows
  LED_init();
  rtc.begin();

  //Vent Controls
  *ddr_c &= MB3;
  *ddr_c &= MB2;
  //Reset Button
  *ddr_b &= MB7;
  //Pin 19 for interrupt
  *ddr_d &= MB2;
  //Fan Control Pins
  *ddr_c |= SB1;
  *ddr_c |= SB0;
  *ddr_d |= SB7;
  
  attachInterrupt(digitalPinToInterrupt(19), Toggle_Enable, RISING);
  
  myStepper.setSpeed(10);

  currentState = DISABLE;
  currentLED = YELLOW;
  previousMill = 0;
}

void loop()
{
  StateUpdate();
  BlinkTimer(millis(), currentLED);

  //stop vent from moving when in error state
  if(enabled_flag){
    TurnFanPositive();
    TurnFanNegative();
  }

  //LCD print only when in RUNNING or IDLE
  if(enabled_flag) LCDPrint();
  else lcd.clear();
}

////////////////////////////////UART FUNCTIONS/////////////////////////////////
#pragma region 
void U0init(unsigned long U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit()
{
  if(*myUCSR0A & RDA) return 1;
  else return 0;
}

unsigned char U0getchar()
{
  unsigned char temp;
  temp = *myUDR0;
  return temp;
}

void U0putchar(unsigned char U0pdata)
{
  while(!(*myUCSR0A & TBE)){}
  *myUDR0 = U0pdata;
}

void U0print(unsigned int data){
  unsigned int out = data;
  bool flag = false;
  if(out >= 1000){
    U0putchar(((out/1000) + '0'));
    out %= 1000;
    flag = true;
  }
  if(out >= 100){
    U0putchar(((out/100) + '0'));
    out %= 100;
    flag = true;
  } else if (flag) U0putchar(0+'0');
  if(out >= 10){
    U0putchar(((out/10) + '0'));
    out %= 10;
  } else if (flag) U0putchar(0+'0');
  U0putchar((out + '0'));
}
#pragma endregion
///////////////////////////////////////////////////ADC FUNCTIONS///////////////////////////////////////////////
#pragma region
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}
#pragma endregion
/////////////////////////////////HELPER FUNCTIONS//////////////////////////////////////////////
////LED Functions////////
#pragma region 
//Initilize LED pins
void LED_init(){
  *ddr_h |= SB3;
  *ddr_h |= SB4;
  *ddr_h |= SB5;
  *ddr_h |= SB6;
}
void Green_Toggle(){
  if(*pin_h &= SB3) *port_h &= MB3;
  else *port_h |= SB3;
}
void Red_Toggle(){
  if(*pin_h &= SB6) *port_h &= MB6;
  else *port_h |= SB6;
}
void Yellow_Toggle(){
  if(*pin_h &= SB5) *port_h &= MB5;
  else *port_h |= SB5;
}
void Blue_Toggle(){
  if(*pin_h &= SB4) *port_h &= MB4;
  else *port_h |= SB4;
}
void Blue_Off(){
  *port_h &= MB4;
}
void Red_Off(){
  *port_h &= MB6;
}
void Yellow_Off(){
  *port_h &= MB5;
}
void Green_Off(){
  *port_h &= MB3;
}
void Turn_Off_LEDs(LED currentLED){
  switch(currentLED){
    case GREEN:
      Yellow_Off();
      Red_Off();
      Blue_Off();
      break;
    case RED:
      Green_Off();
      Yellow_Off();
      Blue_Off();
      break;
    case BLUE:
      Green_Off();
      Yellow_Off();
      Red_Off();
      break;
    case YELLOW:
      Green_Off();
      Red_Off();
      Blue_Off();
      break;
  }
}
void BlinkTimer(const long currentMill, LED currentLED){
  if(currentMill - previousMill >= max_interval){
    previousMill = currentMill;
    switch (currentLED) {
      case GREEN:
        Green_Toggle();
        break;
      case YELLOW:
        Yellow_Toggle();
        break;
      case RED:
        Red_Toggle();
        break;
      case BLUE:
        Blue_Toggle();
        break;
      default:
        break;
    }
  }
}
#pragma endregion
//////Bool Functions
#pragma region 
int WaterCheck(){
  unsigned int water_sensor = adc_read(0);
  if(water_sensor <= 50) return 1;
  else return 0;
}
int TempCheck(){
  int chk = dht1.read11(10);
  if (dht1.temperature > 20) return 1;
  else return 0;
}
int CheckResetPress(){
  if(*pin_b & SB7) return 1;
  else return 0;
}
#pragma endregion
//////State Functions////////
#pragma region 
void StateUpdate(){
  switch(currentState){
    case IDLE:
      currentLED = GREEN;
      if(WaterCheck()){
        currentState = ERROR;
        error_flag = 1;
        lcd.clear();
      }
      else if(TempCheck()){
        currentState = RUNNING;
        StartFan();
      }
      break;
    case RUNNING:
      currentLED = BLUE;
      if(WaterCheck()){
        currentState = ERROR;
        error_flag = 1;
        StopFan();
        lcd.clear();
      }
      else if(!TempCheck()){
        currentState = IDLE;
        StopFan();
      }
      break;
    case DISABLE:
      currentLED = YELLOW;
      break;
    case ERROR:
      currentLED = RED;
      if(CheckResetPress()){
        currentState = IDLE;
        error_flag = 0;
      }
      break;
  }
  //Turn off all other LEDs other than currentLED
  Turn_Off_LEDs(currentLED);
}
#pragma endregion
///////Fan Functions///////
#pragma region 
void TurnFanNegative(){
  if(*pin_c & SB3) myStepper.step(10);
}
void TurnFanPositive(){
  if(*pin_c & SB2) myStepper.step(-10);
}
void StartFan(){
  *port_c |= SB1;
  *port_c |= SB0;
  *port_d &= MB7;
  RTCRecord();
}
void StopFan(){
  *port_c &= MB1;
  *port_c &= MB0;
  *port_d &= MB7;
  RTCRecord();
}
#pragma endregion
//////Print Functions
#pragma region 
void LCDPrint(){
  if(error_flag) {
    PrintWaterError();
    return;
  }
  int chk = dht1.read11(10);
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(dht1.temperature);
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(dht1.humidity);
}
void RTCRecord(){
  DateTime now = rtc.now();
  U0print(now.year());
  U0putchar('/');
  U0print(now.month());
  U0putchar('/');
  U0print(now.day());
  U0putchar(' ');
  U0putchar('T');
  U0putchar('i');
  U0putchar('m');
  U0putchar('e');
  U0putchar('-');
  U0print(now.hour());
  U0putchar(':');
  U0print(now.minute());
  U0putchar(':');
  U0print(now.second());
  U0putchar('\n');
}
void PrintWaterError(){
  lcd.setCursor(0, 0);
  lcd.print("Water");
  lcd.setCursor(0,1);
  lcd.print("level low!");
}
#pragma endregion