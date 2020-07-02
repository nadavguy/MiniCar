#include <Wire.h>
#include "kalman1.cpp"
#include "PIDControl.h"
#include "MPU9250.h"

#define Forward 90
#define Left 150
#define Right 30

int MotorPWMPin = 5;
int ServoPWMPin = 6;

int ArrayCounter = 0;
int SelectedPower = 0;
int SelectedAngle = 90;
int SpeedCounter = 0;

bool IsCounterOverProtection = false;
bool CanRead = false;
bool MagWasInMax = false;
bool MagWasInMin = false;

unsigned long PreviousTimer = 0;
unsigned long TwentyMilisecondCycle = 0;
unsigned long FixedCourseTime = 0;
unsigned long LastReceivedMessageTimer = 0;
unsigned long PreviousSensorReadCycle = 0; // in micro seconds
unsigned long PreviousPIDCycle = 0; // in micro seconds

double TimeInSeconds = 0;
double ArraySum = 0;

String TextFromPort = "";
String SticksMessageHeader = "AB56FE21,";
String TempSTR = "";

char terminator = '~';

float ax = 0, ay = 0, az = 0;
float mx = 0, my = 0, mz = 0;
float gx = 0, gy = 0, gz = 0;

MPU9250 IMU(Wire, 0x68);
int status;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial1.setTimeout(1);
  pinMode(MotorPWMPin, OUTPUT);
  pinMode(ServoPWMPin, OUTPUT);
  status = IMU.begin();
      Serial.print("Status: ");
    Serial.println(status);
  if (status < 0)
  {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1)
    {
    }
  }
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_92HZ);
  IMU.setSrd(9);
  TwentyMilisecondCycle = micros();
}

void loop()
{
  // put your main code here, to run repeatedly:
  TimeInSeconds = micros() / 1000000.0;
  //Serial.println(TimeInSeconds);

  if (micros() < TwentyMilisecondCycle)
  {
    TwentyMilisecondCycle = micros();
    IsCounterOverProtection = true;
    // Serial.println("Counter OverProtection Last20mSecTimer.~");
  }

  if (IsCounterOverProtection)
  {
    IsCounterOverProtection = false;
    // Serial.println("Counter OverProtection enabled.~"); // for testing DO NOT REMOVE
  }

  if (micros() - TwentyMilisecondCycle >= 20000)
  {
    TwentyMilisecondCycle = micros();
  } //PreviousSensorReadCycle

  if (micros() - PreviousSensorReadCycle >= 10000) //50Hz
  {
    IMU.readSensor();
    ReadMPUData();
    // Magnet close
    // 195.75, MagX: -858.40, MagY: -1916.38, MagZ: 3762.70
    // Magnet far
    // 179.51, MagX: -181.48, MagY: 1092.53, MagZ: -272.61
    // Runs at 100Hz since counts Max & Min Magneto
    if ( (mz > 3000) && (my < -1500) && (!MagWasInMax) )
    {
      SpeedCounter++;
      MagWasInMax = true;
      MagWasInMin = false;
    }
    else if ( (mz < 0) && (my > 850) && (MagWasInMax) )
    {
      SpeedCounter++;
      MagWasInMax = false;
      MagWasInMin = true;
    }
    PreviousSensorReadCycle = micros();
  } 

  if (micros() - PreviousPIDCycle >= 100000) // 10Hz
  {
    // Serial.print(TimeInSeconds);
    CalcPIDStep(SpeedCounter);
    SelectedPower = (int)map((int)PIDOutput, 0, 35, 0, 255);
    // PIDThrottleOutput = (int)map((int)PIDOutput, -35, 35, -127, 127);
    // ControlDualMotors(PIDThrottleOutput,0); //LeftRightValue
        // Serial.print(", Roll: ");
    // Serial.print(Roll);
    // Serial.print(", Pitch: ");
    // Serial.print(Pitch);
    // // Serial.print(", Yaw: ");
    // // Serial.print(Yaw);
    // Serial.print(", dT: ");
    // Serial.print(dT);
    // Serial.print(", Error: ");
    // Serial.print(Error);
    // Serial.print(", Addetive: ");
    // Serial.print(AddativeError);
    // Serial.print(", Derivative: ");
    // Serial.print(Derivative);
    // Serial.print(", PIDOutput: ");
    // Serial.print(PIDOutput);
    // Serial.print(", PIDThrottleOutput: ");
    // Serial.print(SelectedPower);
    // Serial.print(", SpeedCounter: ");
    // Serial.print(SpeedCounter);
    // Serial.println("~");
    SpeedCounter = 0;
    PreviousPIDCycle = micros();
  }


  PWMSignal(MotorPWMPin, SelectedPower);
  SetServoAngle(SelectedAngle);
}

void PWMSignal(int Channel, int Value)
{
  //Value: 0 - 100
  if (micros() - TwentyMilisecondCycle < 200 * Value) // 20000 / 100 * Value
  {
    CanRead = false;
    digitalWrite(Channel, HIGH);
  }
  else
  {
    CanRead = true;
    digitalWrite(Channel, LOW);
  }
}

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

void SetServoAngle(int AngleToSet)
{
  //Left - 35 degrees 1mSec
  //Forward - 90 degrees 1.5mSec
  //Right - 145 degrees 2mSec
  int PWMValue = 1000 * ((2 - 1) * (double)AngleToSet / (145.0 - 35.0) + 0.5);
  // Serial.print("PWMValue: ");
  // Serial.println(PWMValue);
  if (micros() - TwentyMilisecondCycle <= PWMValue) // 20000 / 100 * Value
  {
    digitalWrite(ServoPWMPin, HIGH);
  }
  else if (micros() - TwentyMilisecondCycle <= 14000)
  {
    digitalWrite(ServoPWMPin, LOW);
    // LastReceivedMessageTimer = micros();
    TextFromPort = Serial1.readStringUntil(terminator);
    if (!TextFromPort.equals("\r\n\n"))
    {
      // Serial.println(TextFromPort);
      //Serial1.println(TextFromPort);
      if (TextFromPort.length() >= 18)
      {
        ParseRFMessage(TextFromPort);
      }
      //GeneralMessage.concat(TextFromPort);
      //SendMEssageRF(GeneralMessageHeader,GeneralMessage);
    }
  }
  else
  {
    /* code */
    digitalWrite(ServoPWMPin, LOW);
  }
  
}

void ParseRFMessage(String MessageToParse)
{
  // AB56FE21,6,156,#127,0
  // Header,Size,Counter,Body
  // Size includes '#' and '~' without Counter
  int HeaderIndex = MessageToParse.indexOf(SticksMessageHeader);
  int EndOfMessageIndex = MessageToParse.indexOf('!');
  // Serial.print("HeaderIndex: ");
  // Serial.println(HeaderIndex);
  // Serial.print("EndOfMessageIndex: ");
  // Serial.println(EndOfMessageIndex);
  if ((HeaderIndex != -1) && (EndOfMessageIndex != -1))
  {
    int MessageSizeEndIndex = MessageToParse.indexOf(",", HeaderIndex + 8);
    String MessageSizeString = MessageToParse.substring(HeaderIndex + 8, MessageSizeEndIndex);
    long MessageSize = MessageSizeString.toInt();
    int MessageBodyBegining = MessageToParse.indexOf("#", HeaderIndex + 8);
    String MessageBody = MessageToParse.substring(MessageBodyBegining + 1, EndOfMessageIndex);
    int MessageBodySeperatorIndex = MessageToParse.indexOf(",", MessageBodyBegining + 1);
    String MessageBodyThrottleString = MessageToParse.substring(MessageBodyBegining + 1, MessageBodySeperatorIndex);
    String MessageBodyLeftRightString = MessageToParse.substring(MessageBodySeperatorIndex + 1, EndOfMessageIndex);

    SelectedPower = MessageBodyThrottleString.toInt();
    SelectedAngle = MessageBodyLeftRightString.toInt();
    // Serial.print("SelectedAngle: ");
    // Serial.println(SelectedAngle);
  }
}

void ReadMPUData()
{

  // Serial.print("Value: ");
  // Serial.println(MInt);
    // display the data
    // dtostrf(IMU.getAccelX_mss(), 10, 7, AccX);
    // dtostrf(IMU.getAccelY_mss(), 10, 7, AccY);
    // dtostrf(IMU.getAccelZ_mss(), 10, 7, AccZ);
    // dtostrf(IMU.getGyroX_rads(), 10, 7, GyroX);
    // dtostrf(IMU.getGyroY_rads(), 10, 7, GyroY);
    // dtostrf(IMU.getGyroZ_rads(), 10, 7, GyroZ);
    // dtostrf(IMU.getMagX_uT(), 10, 7, MagX);
    // dtostrf(IMU.getMagY_uT(), 10, 7, MagY);
    // dtostrf(IMU.getMagZ_uT(), 10, 7, MagZ);

  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();
  PreviousSensorReadCycle = micros();

  // Serial.print(TimeInSeconds);
  // Serial.print(", MagX: ");
  // Serial.print(mx);
  // Serial.print(", MagY: ");
  // Serial.print(my);
  // Serial.print(", MagZ: ");
  // Serial.println(mz);
}

// void serialEvent1()
// {
//   LastReceivedMessageTimer = micros();
//   if (CanRead)
//   {
//     TextFromPort = Serial1.readStringUntil(terminator);
//   }
//   else
//   {
//     TextFromPort = "\r\n\n";
//   }
//   if (!TextFromPort.equals("\r\n\n"))
//   {
//     //Serial.println("aaa");
//     // Serial.println(TextFromPort);
//     //Serial1.println(TextFromPort);
//     if (TextFromPort.length() >= 18)
//     {
//       ParseRFMessage(TextFromPort);
//     }
//     //GeneralMessage.concat(TextFromPort);
//     //SendMEssageRF(GeneralMessageHeader,GeneralMessage);
//   }
// }
