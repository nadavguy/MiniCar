#include <Wire.h>
#include "kalman1.cpp"

#define Forward 90
#define Left 150
#define Right 30

int MotorPWMPin = 5;
int ServoPWMPin = 6;

int ArrayCounter = 0;
int SelectedPower = 0;
int SelectedAngle = 90;

bool IsCounterOverProtection = false;
bool CanRead = false;

unsigned long PreviousTimer = 0;
unsigned long TwentyMilisecondCycle = 0;
unsigned long FixedCourseTime = 0;
unsigned long LastReceivedMessageTimer = 0;

double TimeInSeconds = 0;
double ArraySum = 0;

String TextFromPort = "";
String SticksMessageHeader = "AB56FE21,";
String TempSTR = "";

char terminator = '~';

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial1.setTimeout(1);
  pinMode(MotorPWMPin, OUTPUT);
  pinMode(ServoPWMPin, OUTPUT);
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
  // TempSTR = Serial1.readString();
  // if (!TempSTR.equals(""))
  // {
  //   Serial.println(TempSTR);
  //   ParseRFMessage(TempSTR);
  //   TempSTR = "";
  // }

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
  else
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
