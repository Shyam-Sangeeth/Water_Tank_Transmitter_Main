// Including Header Files
#include <Arduino.h>
#include <mcp_can.h>
#include <SoftwareSerial.h>
// Defining Constants
#define CAN_SEND_ID 0x123
#define CAN_RECEIVE_ID 0x124
#define CAN_IMP_ID 0x100
#define RX_PIN 7
#define TX_PIN 6
#define TEMPERATURE_SENSOR_PIN A0
#define CAN_CS_PIN 4
#define HALT_PIN 9
#define ERROR_PIN 8
#define DRY_RUN_PIN 5
#define FLOAT_SWITCH_PIN 3
#define CAN_INT_PIN 2
#define TANK_RADIUS 0.53             // Meter
#define DISTANCE_AT_TANK_EMPTY 120.0 // Centimeter
#define DISTANCE_AT_TANK_FULL 7.5    // Centimeter
// Declaring Variables
boolean systemState = true;
struct can_frame
{
  unsigned long id;
  byte length;
  byte data[8];
} canFrame;
int temperature = 0;
int waterLevel = 0;
byte errorCount = 0;
unsigned long last_interrupt_time = 0;
unsigned long interrupt_time = 0;
boolean tankfull = false;
boolean tankempty = false;
boolean tankStateChanged = false;
unsigned long tankDataCount = 0;
float waterDistance = 0;
// Creating Instances
SoftwareSerial ultraSonicSerial(RX_PIN, TX_PIN);
MCP_CAN CAN_BUS(CAN_CS_PIN);
// Declaring Funtions
void canISR(void);
void tankStateISR(void);
float calculateWater(float);
void firstRun(void);
float readTemperature(void);
void calculateDistance(void);

void setup()
{
  // Configuring Pins
  pinMode(ERROR_PIN, OUTPUT);
  pinMode(HALT_PIN, INPUT);
  pinMode(CAN_INT_PIN, INPUT);
  pinMode(FLOAT_SWITCH_PIN, INPUT);
  pinMode(DRY_RUN_PIN, INPUT);
  pinMode(TEMPERATURE_SENSOR_PIN, INPUT);
  // Intitialize CAN
  if (CAN_BUS.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK)
  {
    systemState = false;
  }
  CAN_BUS.setMode(MCP_NORMAL);
  delay(100);
  // Intitialize ultra sonic sensor
  ultraSonicSerial.begin(9600);
  ultraSonicSerial.listen();
  // Attaching interrupts
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(FLOAT_SWITCH_PIN), tankStateISR, CHANGE);
  // Check state at first run
  firstRun();
  delay(1000);
}

void loop()
{
  // Handle halt
  if (digitalRead(HALT_PIN) == HIGH)
  {
    return;
  }
  // Handle error
  if ((errorCount >= 15) && !systemState)
  {
    digitalWrite(ERROR_PIN, LOW);
    return;
  }
  else
  {
    digitalWrite(ERROR_PIN, HIGH);
  }
  // Calculate distance
  calculateDistance();
  // Send data
  if (tankStateChanged)
  {
    tankStateChanged = false;
    canFrame.length = 5;
    canFrame.id = CAN_IMP_ID;
    canFrame.data[0] = 0b11110000;
    canFrame.data[1] = tankfull;
    canFrame.data[2] = tankempty;
    canFrame.data[3] = tankDataCount == 0;
    canFrame.data[4] = 0b11110000;
    tankDataCount++;
  }
  else
  {
    canFrame.id = CAN_SEND_ID;
    canFrame.length = 8;
    canFrame.data[0] = 170;
    temperature = readTemperature() * 100;
    canFrame.data[1] = (short)(temperature / 100);
    canFrame.data[2] = (short)((temperature / 100.0 - temperature / 100) * 100);
    waterLevel = calculateWater(waterDistance) * 100;
    canFrame.data[3] = (short)(waterLevel / 100);
    canFrame.data[4] = (short)((waterLevel / 100.0 - waterLevel / 100) * 100);
    canFrame.data[5] = !digitalRead(DRY_RUN_PIN);
    canFrame.data[6] = digitalRead(FLOAT_SWITCH_PIN);
    canFrame.data[7] = 170;
  }
  boolean canState = CAN_BUS.sendMsgBuf(canFrame.id, 0, canFrame.length, canFrame.data) == CAN_OK;
  // Handle errors
  if (!canState)
  {
    systemState = false;
    errorCount++;
  }
  if (!waterDistance)
  {
    systemState = false;
    errorCount++;
  }
  if (waterDistance && canState)
  {
    systemState = true;
    errorCount = 0;
  }
  delay(10);
}

float readTemperature()
{
  float temperature = (float)analogRead(TEMPERATURE_SENSOR_PIN) * 500 / 1023.0;
  return temperature;
}

void calculateDistance()
{
  if (ultraSonicSerial.available())
  {
    unsigned char data[4] = {};
    for (int i = 0; i < 4; i++)
    {
      data[i] = ultraSonicSerial.read();
    }
    if (data[0] == 0xFF)
    {
      if (((data[0] + data[1] + data[2]) & 0x00FF) == data[3])
      {
        int readedDistance = (data[1] << 8) + data[2];
        if (readedDistance > 30)
        {
          waterDistance = (float)readedDistance / 10.0;
        }
        else
        {
          waterDistance = 0;
        }
      }
      else
      {
        waterDistance = 0;
      }
    }
    else
    {
      waterDistance = 0;
    }
  }
}

void canISR() {}

void tankStateISR()
{
  if (millis() < 2000 || interrupt_time < last_interrupt_time)
    last_interrupt_time = 0;
  interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 2000)
  {
    tankStateChanged = true;
    last_interrupt_time = interrupt_time;
    if (digitalRead(FLOAT_SWITCH_PIN) == HIGH)
    {
      tankfull = true;
      tankempty = false;
    }
    else
    {
      tankempty = true;
      tankfull = false;
    }
  }
}

void firstRun()
{
  tankStateChanged = true;
  if (digitalRead(FLOAT_SWITCH_PIN) == HIGH)
  {
    tankfull = true;
    tankempty = false;
  }
  else
  {
    tankempty = true;
    tankfull = false;
  }
}

float calculateWater(float distance)
{
  float tankHeight = (DISTANCE_AT_TANK_EMPTY - distance) / 100.0;
  float tankArea = PI * TANK_RADIUS * TANK_RADIUS;
  float tankVolume = tankArea * tankHeight;
  float fullVolume = tankArea * ((DISTANCE_AT_TANK_EMPTY - DISTANCE_AT_TANK_FULL) / 100.0);
  float percentage = (tankVolume / fullVolume) * 100;
  return percentage >= 0 && percentage <= 100 ? percentage : 0;
}