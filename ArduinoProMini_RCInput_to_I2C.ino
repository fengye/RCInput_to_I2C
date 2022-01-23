// Must include before ServoInput.h to enable PCINT
// Installation see: https://github.com/NicoHood/PinChangeInterrupt
#include <PinChangeInterrupt.h> 
#include <ServoInput.h>
#include <Wire.h>
#include <util/atomic.h>
#include <math.h>

// Requires: ServoInput library
// https://github.com/dmadison/ServoInput


// Super simple protocol over I2C, 6 bytes in one transmission.
// See the implementation of onRequestData()
// -------- -------- -------- -------- -------- --------
// |  01   |   16bit_servo   |   02   |  16bit_throttle |
// -------- -------- -------- -------- -------- --------

// ServoInput library needs pin that can be triggered by external interrupt
// Uno, Nano, Mini, other 328-based: Pin 2, 3
ServoInputPin<2> servoSignal;
ServoInputPin<3> throttleSignal;
// Pin 4 is not interrupt-able pin but we use PinChangeInterrupt library to enable it
ServoInputPin<4> ch4Signal;

#define I2C_ADDR 0x28
#define SERIAL_OUTPUT

static const int ONBOARDLED = 13;  // Pro Mini only has one onboard LED that is controllable

static long delayMilli = 0;
static long prevMillis = 0;
static uint8_t leftOrRightLED = 0;
static int ledState[2] = {LOW, LOW};
static float angle = 0.0f;
static float throttle = 0.0f;
static int8_t ch4Reading = 0.0f;
volatile uint16_t lastServoReading = 0xffff;
volatile uint16_t lastThrottleReading = 0xffff;
volatile int8_t lastCh4Reading = 0x0;
static long lastServoAvailableMilli = 0;
static long lastThrottleAvailableMilli = 0;
static long lastCh4AvailableMilli = 0;
static const long LOST_SIGNAL_THRESHOLD = 500; // 0.5 sec without signal means lost, can be changed for tuning

void onRequestData();

#ifdef SERIAL_OUTPUT
  #define SOUT(x) Serial.print(x)
  #define SOUTLN(x) Serial.println(x)  
#else
  #define SOUT(x) (void)x
  #define SOUTLN(x) (void)x
#endif


void setup()
{
  pinMode(ONBOARDLED, OUTPUT);  // Set RX LED as an output
  // TX LED is set as an output behind the scenes

  Wire.begin(I2C_ADDR);
  Wire.onRequest(onRequestData);

#ifdef SERIAL_OUTPUT
  Serial.begin(115200); //This pipes to the serial monitor
  Serial.println("Initialize Serial Monitor");
#endif  
}

void loop()
{
  unsigned long currMillis = millis();
  if (servoSignal.available())
  {
    angle = servoSignal.getAngle();
    lastServoReading = (uint16_t)servoSignal.map(0x0, 0xfffe);
    lastServoAvailableMilli = currMillis;

    const float EXP = 0.9f;
    const float SCALE = 4.0f;
    if (angle > 90.0f)
    {
      leftOrRightLED = 0x2;

      float val = 180.0f - angle; // netrual -> [90, 0] <- right most
      delayMilli = (int)(powf(val * SCALE, EXP)) + 33;
    }
    else
    {
      leftOrRightLED = 0x1;

      float val = angle; // leftmost -> [0, 90] <- netrual
      delayMilli = (int)(powf(val * SCALE, EXP)) + 33;
    }
  }
  else
  {
    

    if (currMillis - lastServoAvailableMilli >= LOST_SIGNAL_THRESHOLD)
    {
      leftOrRightLED = 0x1 | 0x2;
      delayMilli = 2000; // It seems the delay needs to be greater than 1s
      
      lastServoReading = 0xffff;
      SOUTLN("Servo signal lost!");
    }
  }

//  SOUT("Servo reading: ");
//  SOUTLN(lastServoReading);

  if (throttleSignal.available())
  {
    throttle = throttleSignal.getPercent();
    lastThrottleReading = (uint16_t)throttleSignal.map(0x0, 0xfffe);
    lastThrottleAvailableMilli = currMillis;
  }
  else
  {
    if (currMillis - lastThrottleAvailableMilli >= LOST_SIGNAL_THRESHOLD)
    {
      lastThrottleReading = 0xffff;
      SOUTLN("Throttle signal lost!");
    }
  }

//  SOUT("Throttle reading: ");
//  SOUTLN(lastThrottleReading);

  if (ch4Signal.available())
  {
    ch4Reading = (int8_t)ch4Signal.map(0l, 16l);
    lastCh4Reading = ch4Reading;
    lastCh4AvailableMilli = currMillis;
  }
  else
  {
    if (currMillis - lastCh4AvailableMilli >= LOST_SIGNAL_THRESHOLD)
    {
      lastCh4Reading = 0;
      SOUTLN("CH4 signal lost!");
    }
  }

  if (currMillis - prevMillis >= delayMilli)
  {
    if (leftOrRightLED & 0x1 && leftOrRightLED & 0x2)
    {
      // both lit, means error
      SOUTLN("Error reading servo signal.");
    }
    else
    {
      // blink LED
      if (leftOrRightLED & 0x2)
      {
        ledState[1] = !ledState[1];
  
        digitalWrite(ONBOARDLED, ledState[1]);
      }
      else if (leftOrRightLED & 0x1)
      {
        ledState[0] = !ledState[0];
  
        digitalWrite(ONBOARDLED, ledState[0]);
      }
      else
      {
        digitalWrite(ONBOARDLED, HIGH);
      }
      SOUT("Angle: ");
      SOUT(angle);

      SOUT(" Throttle: ");
      SOUT(throttle);

      SOUT(" CH4: ");
      SOUTLN(ch4Reading);
    }



    prevMillis = currMillis;
  }
  
}

#define PACKET_SIZE 3
#define PACKET_NUM 3
volatile unsigned char dataBuf[PACKET_NUM][PACKET_SIZE]; // 1 byte for index, 2 bytes for receiver data

static void onRequestData() {

  uint16_t servoReading, throttleReading;
  uint8_t ch4Reading;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    servoReading = lastServoReading;
    throttleReading = lastThrottleReading;
    ch4Reading = lastCh4Reading;
  }

  dataBuf[0][0] = 0x01;
  unsigned char* ptr = dataBuf[0]+1;
  uint16_t* dataPtr = (uint16_t*)ptr;
  *dataPtr = servoReading;

  dataBuf[1][0] = 0x02;
  ptr = dataBuf[1]+1;
  dataPtr = (uint16_t*)ptr;
  *dataPtr = throttleReading;

  dataBuf[2][0] = 0x10; // 0x10 for multiple data from Ch4 and future data
  ptr = dataBuf[2]+1;
  dataPtr = (uint16_t*)ptr;
  *dataPtr = 0xffff & ch4Reading;
  

  Wire.write((char*)dataBuf, PACKET_SIZE*PACKET_NUM);

//  SOUTLN("Data sent!");
}
