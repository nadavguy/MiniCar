# MiniCar project
Small 3D printed 3 wheel car, based on arduino (micro pro).
Controlled by Java [Smartphone application](https://github.com/nadavguy/BlueToothRC).
Connection using DSD TECH HC-03 bluetooth model.

## Software componenets
* MiniCar.ino - main function, handles parsing RF message, setting servo angle, setting motor power.
* Kalman1.cpp - not used.
### Comments
* RF message buffer is read only when servo signal is low.
* UART read message timeout set to 1ms.
* Servo signal High duration is 1-2ms (35 - 145 degrees respectivly) .

## Arduino pinout
* D5 - MOSFET gate.
* D6 - Servo signal.
* RX / TX - TX / RX Bluetooth module respectivly.

## Message ICD
Header,Size,Counter,#Body!~

* Header - AB56FE21: Sticks message.
* Size - includes '#' and '~' without Counter.
* Counter - running counter.
* Body - Throttle,Direction values.
