// Brake Module ECU test firmware
// 2004-2009 Prius brake actuator

 #include <SPI.h>
// #include <PID_v1.h>
 #include "FiniteStateMachine.h"
 #include "mcp_can.h"
 #include "can_frame.h"
 #include "control_protocol_can.h"


// MOSFET pin (digital) definitions ( MOSFETs control the solenoids )
// pins are not perfectly sequential because the clock frequency of certain pins is different.

// Duty cycles of pins 3 and 5 controlled by timer 3 (TCCR3B)
const byte PIN_SLAFL = 5;      // front left actuation
const byte PIN_SLAFR = 7;      // front right actuation

// Duty cycles of pins 6, 7, and 8 controlled by timer 4 (TCCR4B)
const byte PIN_SLRFL = 6;      // front left return
const byte PIN_SLRFR = 8;      // front right return
const byte PIN_SMC   = 2;      // master cylinder solenoids (two of them)

const byte PIN_PCK1 = 13;      // wheel pressure check pin 1 (PCK1) to assert | Digital IN 13 | PCK 1
const byte PIN_PCK2 = 12;      // wheel pressure check pin 2 (PCK2) to assert | Digital IN 12 | PCK 2

const byte PIN_BRAKE_SWITCH = 48;     // Brake switch PIN          | Digital PIN 48
const byte PIN_PUMP         = 49;     // accumulator pump motor    | Digital PIN 49

// sensor pin (analog) definitions
const byte PIN_MTT = 8;       // accumulator pump motor check (MTT)    | Analog IN 8  | MTT
const byte PIN_PACC = 9;       // pressure accumulator sensor          | Analog IN 9  | PACC
const byte PIN_PMC1 = 10;      // pressure master cylinder sensor 1    | Analog IN 10 | PMC 1
const byte PIN_PMC2 = 11;      // pressure master cylinder sensor 2    | Analog IN 11 | PMC 2
const byte PIN_PFR  = 13;      // pressure front right sensor          | Analog IN 13 | PFR
const byte PIN_PFL  = 14;      // pressure front left sensor           | Analog IN 14 | PFL


// the following are guesses, these need to be debugged/researched
const float ZERO_PRESSURE = 0.48;        // The voltage the sensors read when no pressure is present
const float MIN_PACC = 2.3;              // minumum accumulator pressure to maintain
const float MAX_PACC = 2.4;              // max accumulator pressure to maintain
const float PEDAL_THRESH = 0.6;          // Pressure for pedal interference

 // *****************************************************
 // static global data
 // *****************************************************
 
 // chip select pin for CAN Shield
 #define CAN_CS 53
 #define CAN_CONTROL_BAUD (CAN_500KBPS)
 #define CAN_INIT_RETRY_DELAY (50)
  // construct the CAN shield object
 MCP_CAN CAN(CAN_CS);                                    // Set CS pin for the CAN shield
 static can_frame_s rx_frame_ps_ctrl_brake_command;
 static can_frame_s tx_frame_ps_ctrl_brake_report;
 static uint32_t last_update_ms;
 #define PS_CTRL_RX_WARN_TIMEOUT (150)
 #define GET_TIMESTAMP_MS() ((uint32_t) millis())
 #define GET_TIMESTAMP_US() ((uint32_t) micros())
 bool controlEnabled = false;

 // convert the ADC reading (which goes from 0 - 1023) to a voltage (0 - 5V):
 float convertToVoltage(int input) {
     return input * (5.0 / 1023.0);
 }

 // wheel structure
 struct Brakes {
     float _pressureLeft = 0.0;            // last known right-side pressure
     float _pressureRight = 0.0;           // last known left-side pressure
     byte _sensorPinLeft = 99;             // pin associated with left-side  pressure sensor
     byte _sensorPinRight = 99;            // pin associated with right-side pressure sensors
     byte _solenoidPinLeftA = 99;          // pin associated with MOSFET, associated with actuation solenoid
     byte _solenoidPinRightA = 99;         // pin associated with MOSFET, associated with return solenoid
     byte _solenoidPinLeftR = 99;          // pin associated with MOSFET, associated with actuation solenoid
     byte _solenoidPinRightR = 99;         // pin associated with MOSFET, associated with return solenoid
     bool _increasingPressure = false;     // used to track if pressure should be increasing
     bool _decreasingPressure = false;     // used to track if pressure should be decreasing 
     unsigned long _previousMillis = 0;    // will store last time solenoid was updated 
 
 
     Brakes( byte sensorPinLeft, byte sensorPinRight, byte solenoidPinLeftA, byte solenoidPinRightA, byte solenoidPinLeftR, byte solenoidPinRightR );
 
 
     void depowerSolenoids() 
     {
       analogWrite(_solenoidPinLeftA, 0);
       analogWrite(_solenoidPinRightA, 0);
       analogWrite(_solenoidPinLeftR, 0);
       analogWrite(_solenoidPinRightR, 0);
 
     }
 
 
     // fill pressure
     void powerSLA(int scaler) 
     {
         analogWrite( _solenoidPinLeftA, scaler );
         analogWrite( _solenoidPinRightA, scaler );
     }
 
     
     void depowerSLA() 
     {
         analogWrite( _solenoidPinLeftA, 0 );
         analogWrite( _solenoidPinRightA, 0 );
     }
 
 
     // spill pressure
     void powerSLR(int scaler) 
     {
         analogWrite( _solenoidPinLeftR, scaler );
         analogWrite( _solenoidPinRightR, scaler );
     }
 
     
     void depowerSLR() 
     {
         digitalWrite( _solenoidPinLeftR, LOW );
         digitalWrite( _solenoidPinRightR, LOW );
     }
 
 
     // take a pressure reading 
     void updatePressure() 
     {
       _pressureLeft = convertToVoltage( analogRead(_sensorPinLeft) );
       _pressureRight = convertToVoltage( analogRead(_sensorPinRight) );
     }
 };

static void init_can( void )
 {
     // Wait until we have initialized
     while( CAN.begin(CAN_CONTROL_BAUD) != CAN_OK )
     {
         // wait a little
         Serial.println( "init_can: Trying" );
         delay( CAN_INIT_RETRY_DELAY );
     }
      // Debug log
     Serial.println( "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!init_can: pass" );
 } 

  
int deltaT=10,
    currMicros,
    lastMicros = 0;

unsigned long previousMillis=0;

float pressureToVoltage(int MPa) {
    return ( MPa + 217.1319446 ) / 505.5662053;
    // convert MPa pressure to equivalent voltage
    return MPa;
}

int voltageToPressure( float voltage) {
    // convert voltage reading from sensors to pressure in MPa
    return ( voltage * 505.5662053 ) - 217.1319446;
}

 
uint8_t incomingSerialByte;

// master Solenoid structure
struct SMC {
    float _pressure1 = 0.0; // Initialize pressures to 0.0 to avoid false values
    float _pressure2 = 0.0;
    byte _sensor1Pin = 99;
    byte _sensor2Pin = 99;
    byte _controlPin = 99;

    SMC( byte sensor1Pin, byte sensor2Pin, byte controlPin );

    void solenoidsClose()
    {
        analogWrite( _controlPin, 255 );
    }

    void solenoidsOpen()
    {
        analogWrite( _controlPin, 0 );
    }
};


SMC::SMC( byte sensor1Pin, byte sensor2Pin, byte controlPin )
{
  _sensor1Pin = sensor1Pin;
  _sensor2Pin = sensor2Pin;
  _controlPin = controlPin;

  pinMode( _controlPin, OUTPUT );  // We're writing to pin, set as an output

  solenoidsOpen();
}


// brake constructor
Brakes::Brakes( byte sensorPLeft, byte sensorPRight, byte solenoidPinLeftA, byte solenoidPinRightA, byte solenoidPinLeftR, byte solenoidPinRightR ) {
  _sensorPinLeft = sensorPLeft;
  _sensorPinRight = sensorPRight;
  _solenoidPinLeftA = solenoidPinLeftA;
  _solenoidPinRightA = solenoidPinRightA;
  _solenoidPinLeftR = solenoidPinLeftR;
  _solenoidPinRightR = solenoidPinRightR;

  // initialize solenoid pins to off
  digitalWrite( _solenoidPinLeftA, LOW );
  digitalWrite( _solenoidPinRightA, LOW );
  digitalWrite( _solenoidPinLeftR, LOW );
  digitalWrite( _solenoidPinRightR, LOW );

  // set pinmode to OUTPUT
  pinMode( _solenoidPinLeftA, OUTPUT );
  pinMode( _solenoidPinRightA, OUTPUT );
  pinMode( _solenoidPinLeftR, OUTPUT );
  pinMode( _solenoidPinRightR, OUTPUT );
}

// Instantiate objects
SMC smc(PIN_PMC1, PIN_PMC2, PIN_SMC);
Brakes brakes = Brakes( PIN_PFL, PIN_PFR, PIN_SLAFL, PIN_SLAFR, PIN_SLRFL, PIN_SLRFR);


// A function to parse incoming serial bytes
void processSerialByte() {
    if (incomingSerialByte == 'u') {  
      SMCOpen();
    }
    if (incomingSerialByte == 'i') {  
      SMCClose();
    }
    if (incomingSerialByte == 'a') {  
      SLAFLOpen();
    }
    if (incomingSerialByte == 's') {  
      SLAFLClose();
    }
    if (incomingSerialByte == 'd') {  
      SLAFROpen();
    }
    if (incomingSerialByte == 'f') {  
      SLAFRClose();
    }
    if (incomingSerialByte == 'g') {  
      SLRFLOpen();
    }
    if (incomingSerialByte == 'h') {  
      SLRFLClose();
    }
    if (incomingSerialByte == 'j') {  
      SLRFROpen();
    }
    if (incomingSerialByte == 'k') {  
      SLRFRClose();
    }



    if (incomingSerialByte == '1') {                  // reset
//        pressure_req = .48;
        Serial.println("reset pressure request");
    }
    if (incomingSerialByte == '1') {                  // reset
        smc.solenoidsOpen();
        Serial.println("opened SMCs");
    }
    if (incomingSerialByte == '1') {                  // reset
        smc.solenoidsClose();
        Serial.println("closed SMCs");
    }
    if (incomingSerialByte == '1') {                  // reset
        brakes.depowerSLR();
        Serial.println("depower SLRs");
    }
    if (incomingSerialByte == '1') {                  // reset
        brakes.powerSLR(255);
        Serial.println("power SLRs");
    }
    if (incomingSerialByte == '1') {                  // reset
        brakes.powerSLA(255);
        Serial.println("power SLAs");
    }
    if (incomingSerialByte == '1') {                  // reset
        brakes.depowerSLA();
       Serial.println("depower SLAs");
    }
    if (incomingSerialByte == '1') {                  // reset
//        Serial.println(accumulator._pressure);
    }
    if (incomingSerialByte == 'p') {                  
        Serial.println("{MESSAGE|DATA|Pump on}");
        digitalWrite(PIN_PUMP, HIGH);
    }
    if (incomingSerialByte == '[') {                  
        Serial.println("{MESSAGE|DATA|Pump off}");
        digitalWrite(PIN_PUMP, LOW);
    }
    if (incomingSerialByte == 'n') {                  
        Serial.println("{MESSAGE|DATA|Brake lights on}");
        digitalWrite(PIN_BRAKE_SWITCH, HIGH);
    }
    if (incomingSerialByte == 'm') {                  
        Serial.println("{MESSAGE|DATA|Brake lights off}");
        digitalWrite(PIN_BRAKE_SWITCH, LOW);
    }
      if (incomingSerialByte == 'z') {                  
        Serial.println("{MESSAGE|DATA|Pump startup check}");
   //     pump_startup_check();
    }  
    if (incomingSerialByte == 'x') {                  
        Serial.println("{MESSAGE|DATA|Pressure startup check}");
   //     pressure_startup_check();
    }
    if (incomingSerialByte == 'b') {                  
        Serial.println("{MESSAGE|DATA|Test Power Drivers}");
        test_power_drivers();
    }
}

void SMCOpen()
    {
      pinMode( PIN_SMC, OUTPUT );                
      analogWrite(PIN_SMC, 127);  //Pin 2
      Serial.println("{MESSAGE|DATA|Opened SMCs  S3 PWM}");
    }

void SMCClose()
    {
      pinMode( PIN_SMC, OUTPUT );                
      analogWrite(PIN_SMC, 20);  //Pin 2
      Serial.println("{MESSAGE|DATA|Closed SMCs S3 PWM}");
    }

void SLAFLOpen()
    {
      pinMode( PIN_SLAFL, OUTPUT );                
      analogWrite(PIN_SLAFL, 127); //Pin 5
      Serial.println("{MESSAGE|DATA|Opened SLAFL  S1 PWM}");
    }

void SLAFLClose()
    {
      pinMode( PIN_SLAFL, OUTPUT );                
      analogWrite(PIN_SLAFL, 20); //Pin 5
      Serial.println("{MESSAGE|DATA|Closed SLAFL S1 PWM}");
    }

void SLAFROpen()
    {
      pinMode( PIN_SLAFR, OUTPUT );                
      analogWrite(PIN_SLAFR, 127); //Pin 7
      Serial.println("{MESSAGE|DATA|Opened SLAFR  S5 PWM}");
    }

void SLAFRClose()
    {
      pinMode( PIN_SLAFR, OUTPUT );                
      analogWrite(PIN_SLAFR, 20); //Pin 7
      Serial.println("{MESSAGE|DATA|Closed SLAFR S5 PWM}");
    }

void SLRFLOpen()
    {
      pinMode( PIN_SLRFL, OUTPUT );                
      analogWrite(PIN_SLRFL, 127);
      Serial.println("{MESSAGE|DATA|Opened SLRFL  S2 PWM}");
    }

void SLRFLClose()
    {
      pinMode( PIN_SLRFL, OUTPUT );                
      analogWrite(PIN_SLRFL, 20);
      Serial.println("{MESSAGE|DATA|Closed SLRFL S2 PWM}");
    }

void SLRFROpen()
    {
      pinMode( PIN_SLRFR, OUTPUT );                
      analogWrite(PIN_SLRFR, 127);
      Serial.println("{MESSAGE|DATA|Opened SLRFR  S6 PWM}");
    }

void SLRFRClose()
    {
      pinMode( PIN_SLRFR, OUTPUT );                
      analogWrite(PIN_SLRFR, 20);
      Serial.println("{MESSAGE|DATA|Closed SLRFR S6 PWM}");
    }


// the setup routine runs once when you press reset:
void setup( void )
{
    // set the Arduino's PWM timers to 3.921 KHz, above the acoustic range
    TCCR3B = (TCCR3B & 0xF8) | 0x02; // pins 2,3,5 | timer 3
    TCCR4B = (TCCR4B & 0xF8) | 0x02; // pins 6,7,8 | timer 4

    pinMode( PIN_BRAKE_SWITCH, OUTPUT );
    digitalWrite( PIN_BRAKE_SWITCH, LOW );
    
    pinMode( PIN_PUMP, OUTPUT );
    digitalWrite( PIN_PUMP, LOW );

    // depower all the things
//    digitalWrite( PIN_PUMP, LOW );
//    smc.solenoidsOpen();

    // close rear slrs. These should open only for emergencies and to release brake pressure
//    brakes.depowerSLR();

    // Clear any pressure in the accumulator
//    brakes.powerSLA(250);
//    delay(20000);

    // initialize for braking
//    brakes.depowerSLA();

    Serial.begin( 115200 );

  //  init_can();
    Serial.println( "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!General init: pass" );

}

// in order to test that each PWM driving circuit functions correctly, we can
// drive LEDS, one by one, in a breathing patter (increasing and decreasing PWM duty cycle).
void test_power_drivers() {

    // starting at 0% duty cycle, step up to 100% duty cycle (255)
    for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
        // sets the value (range from 0 to 255):
//        analogWrite(0, fadeValue);  not used
//        analogWrite(1, fadeValue);  not used
        analogWrite(2, fadeValue);  //SMC
        analogWrite(3, fadeValue);  // not used S4
//        analogWrite(4, fadeValue);  not used
        analogWrite(5, fadeValue);  //SLAFL
        analogWrite(6, fadeValue);  //SLRFL
        analogWrite(7, fadeValue);  //SLAFR
        analogWrite(8, fadeValue);  //SLRFR
        analogWrite(44, fadeValue);  //not used S7  
        analogWrite(45, fadeValue);  //not used S8    
        // wait for 30 milliseconds to see the dimming effect
        delay(100);
    }

    // fade from 100% duty cycle back to 0% duty cycle
    for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
        // sets the value (range from 0 to 255):
//        analogWrite(0, fadeValue);  not used
//        analogWrite(1, fadeValue);  not used
        analogWrite(2, fadeValue);  //SMC
        analogWrite(3, fadeValue);  // not used S4
//        analogWrite(4, fadeValue);  not used
        analogWrite(5, fadeValue);  //SLAFL
        analogWrite(6, fadeValue);  //SLRFL
        analogWrite(7, fadeValue);  //SLAFR
        analogWrite(8, fadeValue);  //SLRFR
        analogWrite(44, fadeValue);  //not used S7  
        analogWrite(45, fadeValue);  //not used S8 
        // wait for 30 milliseconds to see the dimming effect
        delay(100);
    }

}



void loop()
{
    // read and parse incoming serial commands
    if( Serial.available() > 0 )
    {
        incomingSerialByte = Serial.read();
        processSerialByte();
    }
    
    float accumulator_pump_check  = convertToVoltage(analogRead(8));
    float MTT = voltageToPressure(accumulator_pump_check);
    Serial.println( "{TABLE|DESCRIPTION|MTT|Acumulator Pump Check}" );
    Serial.print( "{TABLE|SET|MTT|V= " );                
    Serial.print( accumulator_pump_check ); 
    Serial.print( "  P= " ); 
    Serial.print( MTT ); 
    Serial.println( "}" );

    float pressure_accumulator_sensor = convertToVoltage(analogRead(9));
    float PACC = voltageToPressure(pressure_accumulator_sensor);
    Serial.println( "{TABLE|DESCRIPTION|PACC|Pressure accumulator sensor}" );
    Serial.print( "{TABLE|SET|PACC|V= " );                
    Serial.print( pressure_accumulator_sensor ); 
    Serial.print( "  P= " ); 
    Serial.print( PACC ); 
    Serial.println( "}" );

    float PMC1_voltage = convertToVoltage(analogRead(10));
    float PMC1 = voltageToPressure(PMC1_voltage);
    Serial.println( "{TABLE|DESCRIPTION|PMC1|Pressure master cylinder 1}" );
    Serial.print( "{TABLE|SET|PMC1|V= " );                
    Serial.print( PMC1_voltage ); 
    Serial.print( "  P= " ); 
    Serial.print( PMC1 ); 
    Serial.println( "}" );

    float PMC2_voltage = convertToVoltage(analogRead(11));
    float PMC2 = voltageToPressure(PMC2_voltage);
    Serial.println( "{TABLE|DESCRIPTION|PMC2|Pressure master cylinder 2}" );
    Serial.print( "{TABLE|SET|PMC2|V= " );                
    Serial.print( PMC2_voltage ); 
    Serial.print( "  P= " ); 
    Serial.print( PMC2 ); 
    Serial.println( "}" );

    float pressure_front_right = convertToVoltage(analogRead(13));
    float PFR = voltageToPressure(pressure_front_right);
    Serial.println( "{TABLE|DESCRIPTION|PFR|Front Right wheel pressure}" );
    Serial.print( "{TABLE|SET|PFR|V= " );                
    Serial.print( pressure_front_right ); 
    Serial.print( "  P= " ); 
    Serial.print( PFR ); 
    Serial.println( "}" );

    float pressure_front_left = convertToVoltage(analogRead(14));
    float PFL = voltageToPressure(pressure_front_left);
    Serial.println( "{TABLE|DESCRIPTION|PFL|Front Left wheel pressure}" );
    Serial.print( "{TABLE|SET|PFL|V= " );                
    Serial.print( pressure_front_left ); 
    Serial.print( "  P= " ); 
    Serial.print( PFL ); 
    Serial.println( "}" );

}
