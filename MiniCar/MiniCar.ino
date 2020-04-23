int MotorPWMPin = 2;
int ServoPWMPIN = 3;
int ArrayCounter = 0;

long PreviousTimer = 0;
long TwentyMilisecondCycle = 0;
long FixedCourseTime = 0;

float Distance = 0 ;
float AveragedDistance = 0;
float DistanceArray[5] ={0};
float ArraySum = 0;

bool IsLockedCourse = false;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(MotorPWMPin,OUTPUT);

  for (int i =0 ; i < 5 ; i++)
  {
    Distance = ReadDistance();
    AveragedDistance = ArrayAverage(DistanceArray,Distance);
  }


  TwentyMilisecondCycle = micros();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  if (micros() - TwentyMilisecondCycle > 20000)
  {
    TwentyMilisecondCycle = micros();
  }

  //Take Code from SRF02 Demo for reading without delay
  Distance = ReadDistance();
  AveragedDistance = ArrayAverage(DistanceArray,Distance);

  if (AveragedDistance < 100)
  {
    //Check Next Course
    //Check Distance left
    //Check Distance right
    //Move in selected direction for 0.5 second
    IsLockedCourse = true;
    if (micros() - FixedCourseTime > 500000)
    {
      FixedCourseTime = micros();
    }
  }

}

void PWMSignal(int Channel, int Value)
{

}

float ReadDistance()
{
  float LocalDistance = 100;
  //Read SRF02
  //Check how long it takes to read SRF02
  return LocalDistance;
}

float ArrayAverage(float[] Array, float Measurement)
{
  ArraySum = ArraySum - Array[ArrayCounter];
  Array[ArrayCounter] = Measurement;
  ArraySum = ArraySum + Measurement;
  float Average = ArraySum / 5;
  ArrayCounter++;
  if (ArrayCounter >=5)
  {
    ArrayCounter = 0;
  }
  return Average;

}

void SetServoAngle(int AngleToSet, bool StaticCourse)
{
  //Left - 45 degrees
  //Forward - 90 degrees
  //Right - 135 degrees
}