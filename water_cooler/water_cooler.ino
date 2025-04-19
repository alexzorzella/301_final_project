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
// DHT Library: https://github.com/RobTillaart/DHTlib/blob/master/examples/dht11_test/dht11_test.ino

#include <LiquidCrystal.h>
#include <dht.h>

#define RDA 0x80
#define TBE 0x20

// Port guide:
// Water Sensor: A0 (PF0)
// LCD: 53 (lcd_RS), 51 (lcd_EN), 49 (lcd_D4), 47 (lcd_D5), 45 (lcd_D6), 43 (lcd_D7)
// Power Button: 2 (PE4)
// LEDs: 21 (PD0: Yellow), 20 (PD1: Green), 19 (PD2: Blue), and 18 (PD3: Red)
// Fan: 6 (PH3), 7 (PH4)
// Stepper Motor: 8 (PH5)

// UART Pointers for using the Serial Monitor
volatile unsigned char* myUCSR0A = (unsigned char*)0x00C0;
volatile unsigned char* myUCSR0B = (unsigned char*)0x00C1;
volatile unsigned char* myUCSR0C = (unsigned char*)0x00C2;
volatile unsigned int* myUBRR0 = (unsigned int*)0x00C4;
volatile unsigned char* myUDR0 = (unsigned char*)0x00C6;

// Water Level Sensor Pointers
volatile unsigned char* my_ADMUX = (unsigned char*)0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*)0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*)0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*)0x78;

dht DHT;

#define DHT11_PIN 4

// LED Management
volatile unsigned char* port_d = (unsigned char*)0x2B;
volatile unsigned char* ddr_d = (unsigned char*)0x2A;
volatile unsigned char* pin_d = (unsigned char*)0x29;

// Button Management (Button functionality is achieved by using attachInterrupt(...))
volatile unsigned char* port_e = (unsigned char*)0x108;
volatile unsigned char* ddr_e = (unsigned char*)0x107;
volatile unsigned char* pin_e = (unsigned char*)0x106;

// Fan and Stepper Motor Management
volatile unsigned char* port_h = (unsigned char*)0x102;
volatile unsigned char* ddr_h = (unsigned char*)0x101;
volatile unsigned char* pin_h = (unsigned char*)0x100;

// Data
unsigned int currentState = 0;
unsigned int lastState = 0;
// 0: Disabled
// 1: Idle
// 2: Running
// 3: Error

unsigned int temp = 25;
const unsigned int tempThreshold = 20;  // Update this value

/*
 LCD Management
*/
const int RS = 53, EN = 51, D4 = 49, D5 = 47, D6 = 45, D7 = 43;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

bool tempOK() {
  return temp <= tempThreshold;
}

unsigned int waterLevel = 100;
const unsigned int waterLevelThreshold = 150;  // Update this value

bool waterLevelOK() {
  return waterLevel >= waterLevelThreshold;
}

unsigned int humidity = 25;

void updateLCD() {
  // Updates the LCD to display the current air temperature and humidity
  // If the water level is too low, the display reflects that
  lcd.clear();
  lcd.setCursor(0, 0);

  if(currentState == 0) {
    lcd.print("System Disabled.");
  } else if (waterLevelOK()) {
    lcd.print("Temp: " + String(temp) + " C W" + String(waterLevel));
    lcd.setCursor(0, 1);
    lcd.print("Hum: " + String(humidity) + "% RH");
  } else {
    lcd.print("Water level");
    lcd.setCursor(0, 1);
    lcd.print("is too low. " + String(waterLevel));
  }
}

void updateStateMachine() {
  switch (currentState) {
    case 0:  // Disabled
      // The only way to leave the disabled state is to press the button
      break;
    case 1:  // Idle
      if (!waterLevelOK()) {
        currentState = 3;
      } else if (!tempOK()) {
        currentState = 2;
      }
      break;
    case 2:  // Running
      if (!waterLevelOK()) {
        currentState = 3;
      } else if (tempOK()) {
        currentState = 1;
      }
      break;
    case 3:  // Error
      // The only way to leave the error state is to press the button
      break;
    default:
      break;
  }
}

void updateFunctionality() {
  readSensorData();

  if(lastState != currentState) {
    lastState = currentState;

    if(currentState == 3) {
      updateLCD();
    }
  }

  if(currentState == 1 || currentState == 2) {
    updateLCDClock();
  }
}

/*
 Button Management
*/
void toggleSystemEngaged() {
  // currentState++;

  // if(currentState > 3) {
  //   currentState = 0;
  // }

  if (currentState == 0) {  // Disabled -> Idle
    currentState = 1;
  } else if (currentState == 3) {  // Error -> Idle
    currentState = 1;
  } else {  // Any other state -> Disabled
    currentState = 0;
  }

  updateLCD();

  unsigned char Monitor[] = "0123";
  putChar(Monitor[currentState]);
}

void initializePins() {
  // Sets     PD0, PD1, PD2, and PD3 to output
  // Port(s):  21,  20,  19, and  18 to output
  *ddr_d |= 0x1111;

  // Sets     PH5, PH4, PH3, port 17 is used in place of 20 because 20 outputs bad data
  // Port(s):   8,   7,   6
  *ddr_h |= 0x11101;

  // Sets  PE4 to input
  // Port(s):  2
  *ddr_e &= (0x1000);

  attachInterrupt(digitalPinToInterrupt(2), toggleSystemEngaged, RISING);
}

void manageOutput() {
  if(currentState == 2) {
      *port_h |= 0x10000;
  } else {
      *port_h &= 0x01111;
  }

  for (int i = 0; i < 4; i++) {
    if (i == currentState) {
      // Turn it on
      if(i == 1) {
        *port_h |= 0x1;
      } else {
        *port_d |= (0x1 << i);
      }
    } else {
      // Turn it off
      if(i == 1) {
        *port_h &= 0x11110;
      } else {
        *port_d &= ~(0x1 << i);
      }
    }
  }
}

unsigned long previousMillis = 0;
const long interval = 60000;

void updateLCDClock() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    updateLCD();
  }
}

/*
  Water Level Sensor Management
*/
void adc_init() {  //write your code after each commented line and follow the instruction
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
  *my_ADCSRB &= ~(1 << 3);  //MUX5 is bit 3 (this took too long to find)

  // set the channel selection bits for channel 0
  *my_ADMUX |= (adc_channel_num & 0x1F);
  //set channel selection bits to the 5 lowerbits of adc_channel_number

  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= (1 << 6);

  // wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0)
    ;
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
  
  waterLevel = adc_read(0);

  int chk = DHT.read11(DHT11_PIN);

  switch (chk) {
    case DHTLIB_OK:
      break;
    case DHTLIB_ERROR_CHECKSUM:
      break;
    case DHTLIB_ERROR_TIMEOUT:
      break;
    case DHTLIB_ERROR_CONNECT:
      break;
    case DHTLIB_ERROR_ACK_L:
      break;
    case DHTLIB_ERROR_ACK_H:
      break;
    default:
      break;
  }

  temp = DHT.temperature;
  humidity = DHT.humidity;
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
  *myUBRR0 = tbaud;
}

unsigned char kbhit() {
  return *myUCSR0A & RDA;
}

unsigned char getChar() {
  return *myUDR0;
}

void putChar(unsigned char U0pdata) {
  while ((*myUCSR0A & TBE) == 0)
    ;
  *myUDR0 = U0pdata;
}

void setup() {
  U0Init(9600);
  adc_init();
  initializePins();

  lcd.clear();
  lcd.begin(16, 2);

  readSensorData();
  updateLCD();

  // lcd.setCursor(0, 1);
  // lcd.print("Amy??");
}

void loop() {
  manageOutput();

  updateStateMachine();
  updateFunctionality();
}