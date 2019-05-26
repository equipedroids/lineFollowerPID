#include <FalconRobot.h>
#include <Wire.h>
#include <SparkFun_APDS9960.h>

FalconRobotMotors motors(5, 7, 6, 8);
#define defaultSpeed 40

FalconRobotLineSensor rightLightSensor(A0);
FalconRobotLineSensor middleLightSensor(A1);
FalconRobotLineSensor leftLightSensor(A2);
#define rightDefaultInLine 995
#define rightDefaultOutLine 906
#define middleDefaultInLine 983 
#define middleDefaultOutLine 888
#define leftDefaultInLine 999
#define leftDefaultOutLine 929
int rightLightValue, middleLightValue, leftLightValue;

SparkFun_APDS9960 rightColorSensor = SparkFun_APDS9960();
uint16_t rightRedReading = 0, rightBlueReading = 0, rightGreenReading = 0;
int rightRedValue, rightGreenValue, rightBlueValue;

class motorPID{
  private:
    float proportional, integral, derivative;
    int error, lastError = 0, sumError = 0;
  public:
    float Kp = 0, Ki = 0, Kd = 0;
    int defaultLight;
    int motorSpeed;
    int motorDirection;
    void getSpeed(int actualLight){
      error = actualLight - defaultLight;
      proportional = error * Kp;
      sumError += error;
      integral = sumError * Ki;
      derivative = (error - lastError) * Kd;
      lastError = error;
      motorSpeed = proportional + integral + derivative;
    }
    void getDirection(){
      if(motorSpeed < 0){
        motorSpeed *= -1;
        motorDirection = BACKWARD;
      }else{
        motorDirection = FORWARD;
      }
    }
};
motorPID *rightMotor;
motorPID *leftMotor;

void setup(){
  Serial.begin(9600);
  rightColorSensor.init();
  rightColorSensor.enableLightSensor(false);
  rightMotor = new motorPID;
  rightMotor->defaultLight = rightDefaultOutLine;
  rightMotor->Kp = 0.5;
  rightMotor->Ki = 0;
  rightMotor->Kd = 0.1;
  leftMotor = new motorPID;
  leftMotor->defaultLight = leftDefaultOutLine;
  leftMotor->Kp = 0.5;
  leftMotor->Ki = 0;
  leftMotor->Kd = 0.1;
  delay(2000);
}

void readLightSensors(){
  rightLightValue = rightLightSensor.read();
  middleLightValue = middleLightSensor.read();
  leftLightValue = leftLightSensor.read();
}

void readColorSensors(){
  rightColorSensor.readRedLight(rightRedReading);
  rightColorSensor.readGreenLight(rightRedReading);
  rightColorSensor.readBlueLight(rightBlueReading);
  rightRedValue = map(rightRedReading,0,3500,0,255);
  rightGreenValue = map(rightGreenReading,0,3500,0,255);
  rightBlueValue = map(rightBlueReading,0,3500,0,255);
}

int isInLine(){
  // 1
  bool rightInLine = false;
  // 2
  bool middleInLine = false;
  // 3
  bool leftInLine = false;
  
  if(rightLightValue >= rightDefaultInLine-20){ rightInLine = true; }
  if(middleLightValue >= middleDefaultInLine-20){ middleInLine = true; }
  if(leftLightValue >= leftDefaultInLine-20){ leftInLine = true; }

  if(rightInLine && middleInLine && leftInLine){ return 123; }
  else if(rightInLine && middleInLine){ return 12; }
  else if(leftInLine && middleInLine){ return 23; }
  else if(rightInLine){ return 1; }
  else if(middleInLine){ return 2; }
  else if(leftInLine){ return 3; }
  else{ return 0; }
}

void loop(){
  readLightSensors();
  readColorSensors();

  switch(isInLine()){
    case 123:
      Serial.println("all in");
      motors.drive(defaultSpeed, FORWARD);
      break;
    case 12:
      Serial.println("just right and middle in");
      motors.rightDrive(defaultSpeed-5, FORWARD);
      motors.leftDrive(defaultSpeed-10, BACKWARD);
      break;
    case 23:
      Serial.println("just left and middle in");
      motors.rightDrive(defaultSpeed-10, BACKWARD);
      motors.leftDrive(defaultSpeed-5, FORWARD);
      break;
    /*case 0:
      //delay(500);
      Serial.println("all out");
      motors.drive(defaultSpeed-10, FORWARD);
      break;*/
    case 1:
      Serial.println("just right in");
      rightMotor->getSpeed(rightLightValue);
      rightMotor->getDirection();
      leftMotor->getSpeed(leftLightValue);
      leftMotor->getDirection();
      motors.rightDrive(rightMotor->motorSpeed, rightMotor->motorDirection);
      motors.leftDrive(leftMotor->motorSpeed, leftMotor->motorDirection);
      break;
    case 2:
      Serial.println("just middle in");
      motors.drive(defaultSpeed, FORWARD);
      break;
    case 3:
      Serial.println("just left in");
      rightMotor->getSpeed(rightLightValue);
      rightMotor->getDirection();
      leftMotor->getSpeed(leftLightValue);
      leftMotor->getDirection();
      motors.rightDrive(rightMotor->motorSpeed, rightMotor->motorDirection);
      motors.leftDrive(leftMotor->motorSpeed, leftMotor->motorDirection);
      break;
  }
  
  //delay(1000);
}
