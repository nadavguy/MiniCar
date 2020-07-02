double SetPoint = 6.0;
// float ProcessValue = 0;
double PreviousError = 0.0;
double AddativeError = 0.0;
double Error = 0.0;
double Derivative = 0.0;
double PIDOutput = 0.0;
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
long PreviousStep = 0;
double dT = 0;

void CalcPIDStep(float ProcessValue)
{
    dT = (micros() - PreviousStep)/1000000.0;
    Error = SetPoint - ProcessValue;
    AddativeError = AddativeError + Error * dT;
    Derivative = (Error - PreviousError) / dT;
    PIDOutput = Kp * Error + Ki * AddativeError + Kd * Derivative;
    PreviousError = Error;
    PreviousStep = micros();
    // Serial.print("PIDOutput: ");
    // Serial.println(PIDOutput);
}

// previous_error := 0
// integral := 0

// loop:
//     error := setpoint − measured_value
//     integral := integral + error × dt
//     derivative := (error − previous_error) / dt
//     output := Kp × error + Ki × integral + Kd × derivative
//     previous_error := error
//     wait(dt)
//     goto loop