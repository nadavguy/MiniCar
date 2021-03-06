#include <Wire.h>
#include "kalman1.cpp"

#define Forward 90
#define Left 150
#define Right 30

int MotorPWMPin = 5;
int ServoPWMPin = 6;

int ArrayCounter = 0;
int reading = 15;

int TestCounter = 0;

unsigned long PreviousTimer = 0;
unsigned long TwentyMilisecondCycle = 0;
unsigned long FixedCourseTime = 0;
unsigned long LastMeasurementInitiated = 0;

float Distance = 0;
float AveragedDistance = 0;
float DistanceArray[5] = {0};
float ArraySum = 0;
float process_noise = 0.2, sensor_noise = 32, estimated_error = 100, initial_value = 0;
float SelectedAngle = 90;

bool IsLockedCourse = false;
bool IsCounterOverProtection = false;
bool IsDistanceMeasurementCompleted = true;
bool DistanceCheckInitiated = false;
bool IsLeftTestFinished = false;
bool IsRightTestFinished = false;
bool IsForwardTestFinished = false;
bool IsTurnFinished = false;
bool ForcePrint = false;

byte GreenSF02 = 112;

double FilteredRange = 0;
double TimeInSeconds = 0;

double DistanceLeft = 0;
double DistanceRight = 0;
double DistanceForward = 0;

Kalman GreenSF02_KF(process_noise, sensor_noise, estimated_error, initial_value);

void setup()
{
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  pinMode(MotorPWMPin, OUTPUT);
  pinMode(ServoPWMPin, OUTPUT);
  for (int i = 0; i < 5; i++)
  {
    // Distance = ReadDistance();
    // AveragedDistance = ArrayAverage(DistanceArray,Distance);
    ReadDistance(GreenSF02);
    delay(75);
    ReadDistance(GreenSF02);
  }
  delay(10000);
  TwentyMilisecondCycle = micros();
}

void loop()
{
  // put your main code here, to run repeatedly:
  TimeInSeconds = micros() / 1000000.0;
  //Serial.println(TimeInSeconds);
  if (micros() - TwentyMilisecondCycle > 20000)
  {
    TwentyMilisecondCycle = micros();
  }

  if (micros() < TwentyMilisecondCycle)
  {
    TwentyMilisecondCycle = micros();
    IsCounterOverProtection = true;
    Serial.println("Counter OverProtection Last20mSecTimer.~");
  }

  if (IsCounterOverProtection)
  {
    IsCounterOverProtection = false;
    Serial.println("Counter OverProtection enabled.~"); // for testing DO NOT REMOVE
  }
  // SetServoAngle(Right,false);
  // Take Code from SRF02 Demo for reading without delay
  // Distance = ReadDistance();
  // AveragedDistance = ArrayAverage(DistanceArray,Distance);
  ReadDistance(GreenSF02);

  if ((AveragedDistance < 150) && (!DistanceCheckInitiated))
  {
    DistanceCheckInitiated = true;
  }
  else if ((AveragedDistance < 150) && (DistanceCheckInitiated))
  {
    TestCourse();
  }
  else if (AveragedDistance >= 150)
  {
    // Serial.println("Move Forward");
    PWMSignal(MotorPWMPin, 55);
    SetServoAngle(Forward, IsLockedCourse);
  }
}

void PWMSignal(int Channel, int Value)
{
  //Value: 0 - 100
  if (micros() - TwentyMilisecondCycle < 200 * Value) // 20000 / 100 * Value
  {
    digitalWrite(Channel, HIGH);
  }
  else
  {
    digitalWrite(Channel, LOW);
  }
}

// float ReadDistance()
// {
//   float LocalDistance = 100;
//   //Read SRF02
//   //Check how long it takes to read SRF02
//   return LocalDistance;
// }

float ArrayAverage(float Array[], float Measurement)
{
  ArraySum = ArraySum - Array[ArrayCounter];
  Array[ArrayCounter] = Measurement;
  ArraySum = ArraySum + Measurement;
  float Average = ArraySum / 5;
  ArrayCounter++;
  if (ArrayCounter >= 5)
  {
    ArrayCounter = 0;
  }
  return Average;
}

void SetServoAngle(int AngleToSet, bool StaticCourse)
{
  //Left - 45 degrees 1mSec
  //Forward - 90 degrees 1.5mSec
  //Right - 135 degrees 2mSec
  int PWMValue = 1000 * ((2 - 1) * (double)AngleToSet / (135.0 - 45.0) + 0.5);
  // Serial.print("PWMValue: ");
  // Serial.println(PWMValue);
  if (micros() - TwentyMilisecondCycle < PWMValue) // 20000 / 100 * Value
  {
    digitalWrite(ServoPWMPin, HIGH);
  }
  else
  {
    digitalWrite(ServoPWMPin, LOW);
  }
}

void ReadDistance(byte Address)
{
  if (IsDistanceMeasurementCompleted)
  {
    IsDistanceMeasurementCompleted = false;
    // step 1: instruct sensor to read echoes
    Wire.beginTransmission(Address); // transmit to device #112 (0x70)
    // the address specified in the datasheet is 224 (0xE0)
    // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00)); // sets register pointer to the command register (0x00)
    Wire.write(byte(0x51)); // command sensor to measure in "inches" (0x50)
    // use 0x51 for centimeters
    // use 0x52 for ping microseconds
    Wire.endTransmission(); // stop transmitting
    LastMeasurementInitiated = micros();
  }
  if ((micros() - LastMeasurementInitiated >= 70000) && (!IsDistanceMeasurementCompleted))
  {
    // step 2: wait for readings to happen
    //delay(70); // datasheet suggests at least 65 milliseconds

    // step 3: instruct sensor to return a particular echo reading
    Wire.beginTransmission(Address); // transmit to device #112
    Wire.write(byte(0x02));          // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();          // stop transmitting

    // step 4: request reading from sensor
    Wire.requestFrom(Address, 2); // request 2 bytes from slave device #112

    // step 5: receive reading from sensor
    if (2 <= Wire.available())
    {                         // if two bytes were received
      reading = Wire.read();  // receive high byte (overwrites previous reading)
      reading = reading << 8; // shift high byte to be high 8 bits
      reading |= Wire.read(); // receive low byte as lower 8 bits
      //Serial.print("Time: ");
      Serial.print(TimeInSeconds);
      //Serial.print("Measurement Value: ");
      Serial.print(",");
      Serial.print(reading); // print the reading
      AveragedDistance = ArrayAverage(DistanceArray, reading);
      FilteredRange = GreenSF02_KF.getFilteredValue(reading);
      //Serial.print("Filtered Value: ");
      Serial.print(",");
      Serial.print(AveragedDistance);
      Serial.print(",");
      Serial.print(FilteredRange); // print the reading
      Serial.println("");
    }
    IsDistanceMeasurementCompleted = true;
  }
}

void TestCourse()
{
  if (!IsLockedCourse)
  {

    PWMSignal(MotorPWMPin, 0);
    //Check Next Course
    //Check Distance left
    if (!IsLeftTestFinished)
    {
      //Serial.println("Test Left");
      SetServoAngle(Left, false);
      ReadDistance(GreenSF02);
      if (IsDistanceMeasurementCompleted)
      {
        // Serial.print("Counter: ");
        // Serial.println(TestCounter);
        TestCounter++;
      }
      if (TestCounter >= 5)
      {

        DistanceLeft = AveragedDistance;
        IsLeftTestFinished = true;
        TestCounter = 0;
        Serial.print("Test Left Distance: ");
        Serial.println(DistanceLeft);
        Serial.print(DistanceArray[0]);
        Serial.print(", ");
        Serial.print(DistanceArray[1]);
        Serial.print(", ");
        Serial.print(DistanceArray[2]);
        Serial.print(", ");
        Serial.print(DistanceArray[3]);
        Serial.print(", ");
        Serial.print(DistanceArray[4]);
        Serial.println(", ");
      }
    }

    //Check Distance right
    else if ((IsLeftTestFinished) && (!IsRightTestFinished))
    {
      // Serial.println("Test Right");
      SetServoAngle(Right, false);
      ReadDistance(GreenSF02);
      if (IsDistanceMeasurementCompleted)
      {
        TestCounter++;
      }
      if (TestCounter >= 5)
      {

        DistanceRight = AveragedDistance;
        IsRightTestFinished = true;
        TestCounter = 0;
        Serial.print("Test Right Distance: ");
        Serial.println(DistanceRight);
        Serial.print(DistanceArray[0]);
        Serial.print(", ");
        Serial.print(DistanceArray[1]);
        Serial.print(", ");
        Serial.print(DistanceArray[2]);
        Serial.print(", ");
        Serial.print(DistanceArray[3]);
        Serial.print(", ");
        Serial.print(DistanceArray[4]);
        Serial.println(", ");
      }
    }

    else if ((IsLeftTestFinished) && (IsRightTestFinished))
    {
      Serial.println("Choose Direction");
      if (DistanceLeft < DistanceRight)
      {
        SelectedAngle = Right;
      }
      else
      {
        SelectedAngle = Left;
      }
      FixedCourseTime = micros();
      IsLockedCourse = true;
    }
  }

  //Move in selected direction for 0.5 second
  if ((micros() - FixedCourseTime < 2500000) && (IsLockedCourse) && (!IsTurnFinished))
  {
    // Serial.print(TimeInSeconds);
    // Serial.print(", ");
    // Serial.println("Turn");
    PWMSignal(MotorPWMPin, 45);
    SetServoAngle(SelectedAngle, IsLockedCourse);
  }
  else if ((micros() - FixedCourseTime > 2500000) && (IsLockedCourse) && (!IsTurnFinished))
  {
    Serial.println("end turn");

    // IsLeftTestFinished = false;
    // IsRightTestFinished = false;
    IsTurnFinished = true;
    // DistanceCheckInitiated  = false;
  }

  if (IsTurnFinished)
  {
    // PrintText("Test Forward",false);
    SetServoAngle(Forward, false);
    SelectedAngle = Forward;
    ReadDistance(GreenSF02);
    if (IsDistanceMeasurementCompleted)
    {
      Serial.print("Test Forward");
      // PrintText("Counter: ", true);
      // PrintText(String(TestCounter,10), true);
      TestCounter++;
    }
    if (TestCounter >= 5)
    {
      DistanceForward = AveragedDistance;
      IsForwardTestFinished = true;
      TestCounter = 0;
      Serial.print("Test Forward Distance: ");
      Serial.println(DistanceForward);
      Serial.print(DistanceArray[0]);
      Serial.print(", ");
      Serial.print(DistanceArray[1]);
      Serial.print(", ");
      Serial.print(DistanceArray[2]);
      Serial.print(", ");
      Serial.print(DistanceArray[3]);
      Serial.print(", ");
      Serial.print(DistanceArray[4]);
      Serial.println(", ");
    }
  }
  if (IsForwardTestFinished)
  {
    Serial.println("Finished Test Cycle");
    IsForwardTestFinished = false;
    IsLeftTestFinished = false;
    IsRightTestFinished = false;
    DistanceCheckInitiated = false;
    IsTurnFinished = false;
    IsLockedCourse = false;
  }
}
