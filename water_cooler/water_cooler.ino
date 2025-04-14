// -Monitor the water levels in a reservoir and print an alert when the level is too low
// -Monitor and display the current air temp and humidity on an LCD screen.
// -Start and stop a fan motor as needed when the temperature falls out of a specifed
// range (high or low).
// -Allow a user to use a control to adjust the angle of an output vent from the system
// -Allow a user to enable or disable the system using an on/off button
// -Record the time and date every time the motor is turned on or off. This information
// should be transmitted to a host computer (over USB)

#include <LiquidCrystal.h>

#define RDA 0x80
#define TBE 0x20

// UART Pointers for using the Serial Monitor
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

void setup() {
  U0Init(9600);
}

void loop() {
  updateLCDClock();
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