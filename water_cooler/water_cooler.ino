// CPE 301 Final Project: Swamp Cooler
// Alex Zorzella, John G. Henkel, Brian Hunt, and Jared Baze

// Images and Video Google Drive Link: https://drive.google.com/drive/folders/1cFaX7M_nOUcK27EAklEaLIqBZCs4sfJD?usp=sharing
// Technical Document: https://docs.google.com/document/d/1MnegvQWdYwcGvtnNMkRyvDiR0MmGSYa0iBC4kLPaE_0/edit?usp=sharing

// Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf
// DHT Library: https://github.com/RobTillaart/DHTlib/blob/master/examples/dht11_test/dht11_test.ino

#include <LiquidCrystal.h>
#include <dht.h>
#include <uRTCLib.h>
#include <AccelStepper.h>

#define RDA 0x80
#define TBE 0x20

#define BIT(x) (1 << (x))

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

// Temperature and Humidity Sensor Management
dht DHT;

#define DHT11_PIN 4

// LED Management
volatile unsigned char* port_b = (unsigned char*)0x25;
volatile unsigned char* ddr_b = (unsigned char*)0x24;
volatile unsigned char* pin_b = (unsigned char*)0x23;

// On/Off/Reset Button Management
volatile unsigned char* port_e = (unsigned char*)0x108;
volatile unsigned char* ddr_e = (unsigned char*)0x107;
volatile unsigned char* pin_e = (unsigned char*)0x106;

// Fan Motor Management
volatile unsigned char* port_h = (unsigned char*)0x102;
volatile unsigned char* ddr_h = (unsigned char*)0x101;
volatile unsigned char* pin_h = (unsigned char*)0x100;

// Stepper Motor Potentiometer Input Management
volatile unsigned char* port_f = (unsigned char*)0x31;
volatile unsigned char* ddr_f = (unsigned char*)0x30;
volatile unsigned char* pin_f = (unsigned char*)0x2F;

// RTC Input Management
volatile unsigned char* port_d = (unsigned char*)0x2B;
volatile unsigned char* ddr_d = (unsigned char*)0x2A;
volatile unsigned char* pin_d = (unsigned char*)0x29;

// Data
unsigned int currentState = 0;
unsigned int lastState = 0;
// 0: Disabled
// 1: Idle
// 2: Running
// 3: Error

const char* stateNames[4] = { "Disa", "Idle", "Runn", "Erro" };

// Real Time Clock Declaration
uRTCLib rtc(0x68);

// Stepper Motor Management
unsigned int stepperKnob = 0;
unsigned int stepperMode = 0;

const unsigned int stepSpeed = 30;
const int stepsPerRevolution = 100;

AccelStepper stepper(AccelStepper::FULL4WIRE, 29, 25, 27, 23);

// LCD Management
const int RS = 53, EN = 51, D4 = 49, D5 = 47, D6 = 45, D7 = 43;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

unsigned int temp = 25;
const unsigned int tempThreshold = 20;  // Update this value

// Returns whether the current temperature is less than or equal to
// (colder or the same as) the threshold temperature
bool tempOK() {
  return temp <= tempThreshold;
}

unsigned int waterLevel = 100;
const unsigned int waterLevelThreshold = 150;

// Returns whether the water level is above a certain threshold
bool waterLevelOK() {
  return waterLevel >= waterLevelThreshold;
}

unsigned int humidity = 25;

// Updates the LCD to display the current air temperature and humidity
// If the water level is too low, the display instead displays
// "Water level is too low."
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);

  if(currentState == 0) {
    lcd.print("System Disabled.");
  } else if (waterLevelOK()) {
    lcd.print("Temp: " + String(temp) + " C");
    lcd.setCursor(0, 1);
    lcd.print("Hum: " + String(humidity) + "% RH");
  } else {
    lcd.print("Water level");
    lcd.setCursor(0, 1);
    lcd.print("is too low.");
  }
}

// The state of the circuit is determined by this function
// as the current state conditionally changes depending
// on the current inputs from the sensors
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

// Prints a state update message with the state of the circuit,
// the date, and the time
void printDateTime() {
  rtc.refresh();

   String finalMessage = "\nState Updated To " + String(stateNames[currentState]) + "\n" +
                  String(rtc.year()) + "/" + String(rtc.month()) + "/" + String(rtc.day()) + "\n" +
                  String(rtc.hour()) + ":" + String(rtc.minute()) + ":" + String(rtc.second()) + "\n";

  for(int i = 0; i < 39; i++) {
    putChar(finalMessage[i]);
  }
}

// Some parts of the circuit only update once per state change
// to reduce unnessecary calls and LCD refreshes. Whenever its
// state changes, it reads the potentiometer's input and
// updates the angle of the stepper motor if necessary. If the
// circuit has entered the error mode, the LCD updates.
// In addition, it calls the function to update the LCD every
// minute if it's in an appropriate state. 
void updateFunctionality() {
  readSensorData();

  if(lastState != currentState) {
    lastState = currentState;

    if(currentState == 3) {
      updateLCD();
    }

    printDateTime();

    stepperKnob = adc_read(7);

    if(stepperKnob > 500 && stepperMode != 2) {
      stepperMode = 2;
      stepper.moveTo(-150);
    } else if(stepperKnob < 100 && stepperMode != 1) {
      stepperMode = 1;
      stepper.moveTo(150);
    } else if(stepperMode != 0) {
      stepperMode = 0;
    }
  }

  stepper.run();

  if(currentState == 1 || currentState == 2) {
    updateLCDClock();
  }
}

// Button Management
void toggleSystemEngaged() {
  if (currentState == 0) {  // Disabled -> Idle
    currentState = 1;
  } else if (currentState == 3) {  // Error -> Idle
    currentState = 1;
  } else {  // Any other state -> Disabled
    currentState = 0;
  }

  updateLCD();

  unsigned char Monitor[] = "0123";
}

// Manages the state of the LEDs and fan
void manageOutput() {
  // Each LED is turned on or off
  // Only one will ever be on at a time
  for (int i = 0; i < 4; i++) {
    if (i == currentState) {
      // Turn it on
      *port_b |= BIT(i + 4);
    } else {
      // Turn it off{
      *port_b &= ~BIT(i + 4);
    }
  }

  if(currentState == 2) {
    *port_h |= BIT(5);
  } else {
    *port_h &= ~BIT(5);
  }
}

unsigned long previousMillis = 0;
const long interval = 60000;

// Every minute, the LCD is updated with
// the current temperature and humidity
void updateLCDClock() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    updateLCD();
  }
}

//  Water Level Sensor Management
void adc_init() {
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
  *my_ADCSRB &= ~(1 << 3);  //MUX5 is bit 3

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

// Reads in the data from the sensors
void readSensorData() {
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

// UART Functions
void U0Init(int U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);

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

// Initializes the appropriate pins to input and output
void initializePins() {
  // Sets     PB5, PB6, PB7, and PB8 to output
  // Port(s):  10,  11,  12, and  13 to output
  *ddr_b |= BIT(4);
  *ddr_b |= BIT(5);
  *ddr_b |= BIT(6);
  *ddr_b |= BIT(7);

  // Sets     PD0, PD1 to output
  // Port(s):  21, 20 to output
  *ddr_d |= BIT(0);
  *ddr_d |= BIT(1);

  // Sets     PH5 to output
  // Port(s):   8 to output
  *ddr_h |= BIT(5);

  // Sets     PE4 to input
  // Port(s):   2 to input
  *ddr_e &= ~BIT(4);

  // An interrupt is attached to PE4 (Port 2)
  attachInterrupt(digitalPinToInterrupt(2), toggleSystemEngaged, RISING);

  // Sets     PF7 to input
  // Port(s): A7
  *ddr_f &= ~BIT(7);
}

void setup() {
  U0Init(9600);
  adc_init();
  initializePins();

  lcd.clear();
  lcd.begin(16, 2);

  readSensorData();
  updateLCD();

  URTCLIB_WIRE.begin();

  stepper.setMaxSpeed(1000.0);
	stepper.setAcceleration(50.0);
	stepper.setSpeed(200);
  stepper.moveTo(0);
}

void loop() {
  manageOutput();

  updateStateMachine();
  updateFunctionality();
}