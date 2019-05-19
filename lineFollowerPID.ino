/* LIBRARIES */
// Falcon library
#include <FalconRobot.h>

/* CLASSES */
// Class to make robot drives straight
class motorPID{
  private:
    float Kp = 5, Ki = 0, Kd = 0;
    //float Kp = 0.7, Ki = 0.5, Kd = 0.9;
    float proportional, integral, derivative;
    int error, lastError = 0, sumError = 0;
  public:
    int motorSpeed;
    int motorDirection;
    void getSpeed(int actualLight, int defaultLight){
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

/* VARIABLES */
// Motors
FalconRobotMotors motors(5, 7, 6, 8);
// Line sensors
FalconRobotLineSensor leftLightSensor(A2);
FalconRobotLineSensor rightLightSensor(A3);
// Default values for line sensors
#define LEFT_INLINE 1005
#define LEFT_OUTLINE 930
#define RIGHT_INLINE 1010
#define RIGHT_OUTLINE 945
#define DEFAULT_SPEED 250
// Line sensors variables
int rightLightValue, leftLightValue;
// PID variables
motorPID *rightMotor;
motorPID *leftMotor;

/* INITIAL FUNCTION */
void setup() {
  Serial.begin(9600);
  delay(5000);

  rightMotor = new motorPID();
  leftMotor = new motorPID();
}

/* FUNCTION TO VERIFY IF ROBOT IS IN LINE */
bool isInLine(){
  if(rightLightValue <= RIGHT_INLINE-30 && leftLightValue <= LEFT_INLINE-30){ return false; }
  else{ return true; }
}

/* MAIN FUNCTION */
void loop() {
  // Line sensor reading
  leftLightValue = leftLightSensor.read();
  rightLightValue = rightLightSensor.read();
  
  if(!isInLine()){
    motors.drive(DEFAULT_SPEED, FORWARD);
  }
  else{
    rightMotor->getSpeed(rightLightValue, RIGHT_OUTLINE);
    rightMotor->getDirection();
    motors.rightDrive(rightMotor->motorSpeed, rightMotor->motorDirection);
    leftMotor->getSpeed(leftLightValue, LEFT_OUTLINE);
    leftMotor->getDirection();
    motors.leftDrive(leftMotor->motorSpeed, leftMotor->motorDirection);
  }
}
