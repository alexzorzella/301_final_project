// PDF Info:
// -Monitor the water levels in a reservoir and print an alert when the level is too low
// -Monitor and display the current air temp and humidity on an LCD screen.
// -Start and stop a fan motor as needed when the temperature falls out of a specifed
// range (high or low).
// -Allow a user to use a control to adjust the angle of an output vent from the system
// -Allow a user to enable or disable the system using an on/off button
// -Record the time and date every time the motor is turned on or off. This information
// should be transmitted to a host computer (over USB)

// Diagram Info:
// * Only one LED should be on at a time
// * When the system is disabled, the fan should be off and the yellow LED should be on
// * If the temperature is OK and the water level is OK, the fan should be off and the green LED should be on
// * If the water level is too low, there should be an error message, and the red LED should be on

// Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf

#include <LiquidCrystal.h>

#define RDA 0x80
#define TBE 0x20

// UART Pointers for using the Serial Monitor
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// Water Level Sensor Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// LED Management
int currentLED = 0;

volatile unsigned char* port_k = (unsigned char*) 0x108;
volatile unsigned char* ddr_k  = (unsigned char*) 0x107;
volatile unsigned char* pin_k  = (unsigned char*) 0x106;

// Data
bool systemEngaged = false;

unsigned int temp = 80;
const unsigned int tempThreshold = 90; // Update this value

bool tempOK() {
  return temp <= tempThreshold;
}

unsigned int waterLevel = 100;
const unsigned int waterLevelThreshold = 255; // Update this value

bool waterLevelOK() {
  return waterLevel >= waterLevelThreshold;
}

unsigned int humidity = 70; // I think this will be relative humidity

void setup() {
  U0Init(9600);
  adc_init();
}

void loop() {
  manageLEDs();

  // There should be an interrupt for the button that turns the system on and off
  if(systemEngaged) {
    updateLCDClock();
    readSensorData();
  }
}

/*
 LED Management
*/
void initializeLEDs() {
  // Sets   PK0, PK1, and PK2 to output
  // Ports:  A8,  A9, and A10 to output
  *ddr_k &= (0x1);
  *ddr_k &= (0x1 << 1);
  *ddr_k &= (0x1 << 2);
}

void setCurrentLED(int newLED) {
  currentLED = newLED;
}

void manageLEDs() {
  for(int i = 0; i < 3; i++) {
    if(i == currentLED) {
      // Turn it on
    } else {
      // Turn it off
    }
  }
}

/*
 LCD Management
*/

// LCD pin connections
// These ports are reserved for the LCD
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

unsigned long previousMillis = 0;
const long interval = 60000;

void updateLCDClock() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    void updateLCD();
  }
}

void updateLCD() {
  // Updates the LCD to display the current air temperature and humidity
  lcd.clear();
  lcd.begin(0, 0);
  lcd.print("Temp: 37 F");
  lcd.begin(0, 1);
  lcd.print("Hum: 72 RH");
}

/*
  Water Level Sensor Management
*/
void adc_init() { //write your code after each commented line and follow the instruction
  // setup the A register
 // set bit   7 to 1 to enable the ADC
  *my_ADCSRA |= (1 << 7);
 // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= ~(1 << 6);
 // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= ~(1 << 5);
 // clear bit 0-2 to 0 to set prescaler selection to slow reading
  *my_ADCSRA &= ~0x07;

  // setup the B register
  // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= ~(1 << 3);
  // clear bit 2-0 to 0 to set free running mode
  *my_ADCSRB &= ~0x07;

  // setup the MUX Register
  // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX &= ~(1 << 7);
  // set bit 6 to 1 for AVCC analog reference
  *my_ADMUX |= (1 << 6);
  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= ~(1 << 5);
  // clear bit 4-0 to 0 to reset the channel and gain bits
  *my_ADMUX &= ~0x1F;
}

//work with channel 0
unsigned int adc_read(unsigned char adc_channel_num) {
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= ~0x1F;

  // clear the channel selection bits (MUX 5) hint: it's not in the ADMUX register
  //MUX5 in ADCRSB register
  *my_ADCSRB &= ~(1 << 3); //MUX5 is bit 3 (this took too long to find)

  // set the channel selection bits for channel 0
  *my_ADMUX |= (adc_channel_num & 0x1F); 
  //set channel selection bits to the 5 lowerbits of adc_channel_number

  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= (1 << 6);

  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register and format the data based on right justification (check the lecture slide)
  
  unsigned int val = *my_ADC_DATA;
  return val;
}

// Reads the data from the
// * Water level sensor
// * Thermometer
// * Humidity sensor
void readSensorData() {
  // read the water sensor value by calling adc_read() and check the threshold to display the message accordingly
  waterLevel = adc_read(0); // Water level day may need to be processed, check below for what was done in lab
  temp = adc_read(1); // Both temp and humidity might need to be adjusted as adc_read(...) is from the water level lab
  humidity = adc_read(2);

  //if the value is less than the threshold display the value on the Serial monitor
  // if(SensorVal < waterLevelThreshold){
    // unsigned char digits[3];
    // digits[0] = (SensorVal/100)%10;
    // digits[1] = (SensorVal/10)%10;
    // digits[2] = SensorVal%10;

    // for(int i = 0; i < 3; i++){
    //   U0putchar(digits[i]+48); //add 48 to convert to ASCII, otherwise it will display []
    // }
    // U0putchar('\n');
  // }
  //if the value is over the threshold display "Water Level: High" message on the Serial monitor.
  // else{
    // unsigned char Alert[] = "Water Level: High";
    // for(int i = 0; i < 17; i++){
    //   U0putchar(Alert[i]);
    // }
    // U0putchar('\n');
  // }
  //Use a threshold value that works for you with your sensor. There is no fixed value as sensor's sensitivity can differ.
}

/*
 * UART FUNCTIONS
 */
void U0Init(int U0baud) {
    unsigned long FCPU = 16000000;
    unsigned int tbaud;
    tbaud = (FCPU / 16 / U0baud - 1);
    // Same as (FCPU / (16 * U0baud)) - 1;
    *myUCSR0A = 0x20;
    *myUCSR0B = 0x18;
    *myUCSR0C = 0x06;
    *myUBRR0  = tbaud;
}

unsigned char kbhit() {
   return *myUCSR0A & RDA;
}

unsigned char getChar() {
   return *myUDR0;
}

void putChar(unsigned char U0pdata) {
  while((*myUCSR0A & TBE)==0);
    *myUDR0 = U0pdata;
}