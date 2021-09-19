#include <ServoInput.h>
#include <Wire.h>
#include <util/atomic.h>

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

#define I2C_ADDR 0x28
#define SERIAL_OUTPUT

static const int ONBOARDLED = 13;  // Pro Mini only has one onboard LED that is controllable

static long delayMilli = 0;
static long prevMillis = 0;
static uint8_t leftOrRightLED = 0;
static int ledState[2] = {LOW, LOW};
static float angle = 0.0f;
static float throttle = 0.0f;
volatile uint16_t lastServoReading = 0xffff;
volatile uint16_t lastThrottleReading = 0xffff;
static long lastServoAvailableMilli = 0;
static long lastThrottleAvailableMilli = 0;
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

  Serial.begin(9600); //This pipes to the serial monitor
  Serial.println("Initialize Serial Monitor");
}

void loop()
{
  unsigned long currMillis = millis();
  if (servoSignal.available())
  {
    angle = servoSignal.getAngle();
    lastServoReading = (uint16_t)servoSignal.map(0x0, 0xfffe);
    lastServoAvailableMilli = currMillis;
  
    if (angle > 90.0f)
    {
      leftOrRightLED = 0x2;

      float val = 180.0f - angle; // netrual -> [90, 0] <- right most
      delayMilli = (int)(val * 10) + 33;
    }
    else
    {
      leftOrRightLED = 0x1;

      float val = angle; // leftmost -> [0, 90] <- netrual
      delayMilli = (int)(val * 10) + 33;
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

  if (currMillis - prevMillis >= delayMilli)
  {
    // blink TX
    if (leftOrRightLED & 0x2)
    {
      ledState[1] = !ledState[1];

      digitalWrite(ONBOARDLED, ledState[0]);
    }
    else
    {
      digitalWrite(ONBOARDLED, HIGH);
    }

    // blink RX
    if (leftOrRightLED & 0x1)
    {
      ledState[0] = !ledState[0];

      digitalWrite(ONBOARDLED, ledState[0]);
    }
    else
    {
      digitalWrite(ONBOARDLED, HIGH);
    }

    if (leftOrRightLED & 0x1 && leftOrRightLED & 0x2)
    {
      // both lit, means error
      SOUTLN("Error reading servo signal.");
    }
    else
    {
      SOUT("Angle: ");
      SOUT(angle);

      SOUT(" Throttle: ");
      SOUTLN(throttle);

    }


    prevMillis = currMillis;
  }
  
}

#define PACKET_SIZE 3
volatile unsigned char dataBuf[2][PACKET_SIZE]; // 1 byte for index, 2 bytes for receiver data

static void onRequestData() {

  uint16_t servoReading, throttleReading;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    servoReading = lastServoReading;
    throttleReading = lastThrottleReading;
  }

  dataBuf[0][0] = 0x01;
  unsigned char* ptr = dataBuf[0]+1;
  uint16_t* dataPtr = (uint16_t*)ptr;
  *dataPtr = servoReading;

  dataBuf[1][0] = 0x02;
  ptr = dataBuf[1]+1;
  dataPtr = (uint16_t*)ptr;
  *dataPtr = throttleReading;

  Wire.write((char*)dataBuf, PACKET_SIZE*2);
}
