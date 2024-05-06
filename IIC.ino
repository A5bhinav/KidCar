#include <Wire.h>
//Don't change these:
//These are the ports that all the devices are connected to
byte deviceAddress = 0x10;  // The address of the TF-Luna device is 0x10
const int motorControlOne =  11;
const int motorControlTwo = 10;
const int backLED1 = 5;
const int backLED2 = 6;
const int buttonPin = 2;// the number of the LED pin

//These are all the variables for the lidar sensor
bool buttonState = false;  // variable for reading the pushbutton status
bool go = false;
const double speedIncrease = 2; //This constant changes the rate of acceleration.
//Max speed cannot be more than 255.
const int maxSpeed = 112.5; //This constant is to change the max speed of the car.
int carSpeed = 0;
int count = 0;
int move(bool go1);
int k = 0;
const int correctLidarDis = 50; //Max Distance to stop car
const int correctRangeFinderDis = 50;

void setup() {
  Wire.begin();             // The I2C bus communication starts
  Serial.begin(115200);       // Example Set the baud rate of the serial port to 115200
  pinMode(motorControlOne, OUTPUT);
  pinMode(motorControlTwo, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(backLED1, OUTPUT);
  pinMode(backLED2, OUTPUT);
  pixels.begin();
}


int move(bool go1){
  if ((go1) ) {
    //The car will ramp up at a set pace until it reaches its max speed.
    //Prevents kid from being jerked backwards.
    if(carSpeed < maxSpeed){
      carSpeed += speedIncrease;
    }else{
      carSpeed = maxSpeed;
    }
//    digitalWrite(motorControlOne, HIGH);
  } else if(!go1) {
    //The car will decrease by the speed constant once the kid pushes the button
    //Until it reaches zero. Prevents kid from being jerked forwards.
    if(carSpeed > 0){
      carSpeed -= speedIncrease;
    }else{
      carSpeed = 0;
    }
  }
  return carSpeed;
}

void loop() {
  Wire.beginTransmission(deviceAddress);  // The I2C data transmission starts
  Wire.write(0x00);                       // Send command
  Wire.endTransmission();                 // The I2C data transfer is complete

  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)7);     // Read 7 bytes of data

  if (Wire.available() == 7) {            // 7 bytes of data are available
    byte data[7];
    for (int i = 0; i < 7; i++) {
      data[i] = Wire.read();              // Read data into an array
    }
    
    int distance = (data[1] << 8) | data[0];                   // DistanceValue
    int signalStrength = (data[3] << 8) | data[2];            // signal strength
    

   buttonState = digitalRead(buttonPin);
  //Serial.println("Button State : ");
  //Serial.println(buttonState);
  //Serial.println(buttonState
  if(((buttonState == HIGH))){
    go = true;
    //count++;
  }
  if((distance < correctLidarDis)){
    go = false;
    //count = 0;
  }
  if(buttonState == LOW){
    //count = 0;
    go = false;
  }
//  Serial.println(currentTime - previousTime);
  k = move(go);
  analogWrite(motorControlOne, k);
  analogWrite(motorControlTwo, k);
  
  if(go){
    digitalWrite(backLED1, 0);
    digitalWrite(backLED2, 0);

  }else{
    digitalWrite(backLED1, 255);
    digitalWrite(backLED2, 255);
  }

    // Serial.print("Distance: ");
    // Serial.print(distance);
    // Serial.print(" cm  \n");

    // Serial.print("Signal Strength: ");
    // Serial.print(signalStrength);
    // Serial.print("\n");
  }

  delay(10);              
}

