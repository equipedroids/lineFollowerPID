/*
  BIBLIOTECAS:
  Falcon -> utilizada para manipular os motores e os sensores de reflectância de maneira mais dinâmica
  Wire e SparkFun: utilizadas para manipular os sensores de cor de maneia mais dinâmica
*/
#include <FalconRobot.h>
#include <Wire.h>
#include <SparkFun_APDS9960.h>

/*
  VARIÁVEIS DOS MOTORES:
  motors-> motores
  defaultSpeed-> velocidade padrão 
*/
FalconRobotMotors motors(5, 7, 6, 8);
#define defaultSpeed 40

/*
  VARIÁVEIS DOS SENSORES DE REFLECTÂNCIA:
  right, middle e leftLightSensor -> os prórpios sensores, configurados nas portas analógicas 0, 1 e 2, respectivamente
  right, middle e leftDeaultInLine -> valores padrões de leitura quando situados DENTRO da linha preta 
  right, middle e leftDeaultOutLine -> valores padrões de leitura quando situados FORA da linha preta
  right, middle e leftValue -> valores lidos pelos sensores de reflectância
*/
FalconRobotLineSensor rightLightSensor(A0);
FalconRobotLineSensor middleLightSensor(A1);
FalconRobotLineSensor leftLightSensor(A2);
#define rightDefaultInLine 988
#define rightDefaultOutLine 912
#define middleDefaultInLine 985 
#define middleDefaultOutLine 893
#define leftDefaultInLine 1000
#define leftDefaultOutLine 931
int rightLightValue, middleLightValue, leftLightValue;

/*
  VARIÁVEIS DOS SENSORES DE COR:
  rightColorSensor -> o próprio sensor de cor, configurado de acordo com a biblioteca externa
  red, green e blueReading -> valores lidos pelos sensores de cor
  red, green e blueValue -> valores lidos pelos sensores de cor MAPEADOS
*/
SparkFun_APDS9960 rightColorSensor = SparkFun_APDS9960();
uint16_t rightRedReading = 0, rightBlueReading = 0, rightGreenReading = 0;
int rightRedValue, rightGreenValue, rightBlueValue;

/*
  Classe utilizada para equacionar e calcular a velocidade e direção de cada motor durante o segue-linha com base em um sistema proporcional, integral e derivativo
*/
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
/*
  Objetos da classe <motorPID>
*/
motorPID *rightMotor;
motorPID *leftMotor;

void setup(){
  // Inicialização serial
  Serial.begin(9600);
  // Inicialização dos sensores de cor
  rightColorSensor.init();
  rightColorSensor.enableLightSensor(false);
  // Iniciação dos objetos da classe <motorPID> e calibragem de suas respectivas constantes PID
  rightMotor = new motorPID;
  rightMotor->defaultLight = rightDefaultOutLine;
  rightMotor->Kp = 0.55;
  rightMotor->Ki = 0;
  rightMotor->Kd = 0.1;
  leftMotor = new motorPID;
  leftMotor->defaultLight = leftDefaultOutLine;
  leftMotor->Kp = 0.55;
  leftMotor->Ki = 0;
  leftMotor->Kd = 0.1;
  // Tempo de espera antes de iniciar a função principal
  delay(2000);
}

/*
  LEITURA DOS SENSORES DE REFLECTÂNCIA
*/
void readLightSensors(){
  // Leitura
  rightLightValue = rightLightSensor.read();
  middleLightValue = middleLightSensor.read();
  leftLightValue = leftLightSensor.read();
}

/*
  LEITURA DOS SENSORES DE COR
*/
void readColorSensors(){
  // Leitura
  rightColorSensor.readRedLight(rightRedReading);
  rightColorSensor.readGreenLight(rightGreenReading);
  rightColorSensor.readBlueLight(rightBlueReading);
  // Mapeamento
  rightRedValue = map(rightRedReading,0,3500,0,3500);
  rightGreenValue = map(rightGreenReading,0,3500,0,3500);
  rightBlueValue = map(rightBlueReading,0,3500,0,3500);
}

/*
  VERIFICAÇÃO DA POSIÇÃO DOS SENSORES DE REFLECTÂNCIA
*/
int isInLine(){
  // Variáveis de verificação
  bool rightInLine = false;
  bool middleInLine = false;
  bool leftInLine = false;
  if(rightLightValue >= rightDefaultInLine-20){ rightInLine = true; }
  if(middleLightValue >= middleDefaultInLine-20){ middleInLine = true; }
  if(leftLightValue >= leftDefaultInLine-20){ leftInLine = true; }

  // Verificação da posição dos sensores em relação à linha preta
  if(rightInLine && middleInLine && leftInLine){ return 123; }    // ambos os sensores
  else if(rightInLine && middleInLine){ return 12; }              // apenas o direito e médio
  else if(leftInLine && middleInLine){ return 23; }               // apenas o esquerdo e médio
  else if(rightInLine){ return 1; }                               // apenas o direito
  else if(middleInLine){ return 2; }                              // apenas o médio
  else if(leftInLine){ return 3; }                                // apenas o esquerdo
  else{ return 0; }                                               // nenhum
}

int solveIntersection(){
  // Variáveis de verificação
  bool rightInGreen = false;
  if(rightRedValue >= 5 && rightRedValue <= 10 && rightGreenValue >= 7 && rightGreenValue <= 15 && rightBlueValue >= 5 && rightBlueValue <= 10){ rightInGreen = true; }
  
  // Verificação da posição dos sensores de cor em relação à linha verde
  if(rightInGreen){ return 1; }
  else{ return 0; }
}

/*
  SEGUE-LINHA
*/
void lineFollower(){
  // Calculo da velocidade e direção de cada motor
  rightMotor->getSpeed(rightLightValue);
  rightMotor->getDirection();
  leftMotor->getSpeed(leftLightValue);
  leftMotor->getDirection();
  // Aplicação proporcional, integral e derivativa
  motors.rightDrive(rightMotor->motorSpeed, rightMotor->motorDirection);
  motors.leftDrive(leftMotor->motorSpeed, leftMotor->motorDirection);
}

/*
  CURVAS DE 90º
*/
void rotate(String orientation){
  // curva de 90º para a direita
  if(orientation == "to right"){
    motors.rightDrive(defaultSpeed-5, FORWARD);
    motors.leftDrive(defaultSpeed-10, BACKWARD);
  }
  // curva de 90º para a esquerda
  else if(orientation == "to left"){
    motors.rightDrive(defaultSpeed-10, BACKWARD);
    motors.leftDrive(defaultSpeed-5, FORWARD);
  }
}

/*
  FUNÇÃO PRINCIPAL
*/
void loop(){
  readLightSensors();
  /*readColorSensors();

  Serial.print(rightRedValue);
  Serial.print("\t");
  Serial.print(rightGreenValue);
  Serial.print("\t");
  Serial.println(rightBlueValue);
  
  switch(solveIntersection()){
    case 1:
      //rotate("to right");
      motors.stop();
      delay(70000);
      break;
  }*/

  switch(isInLine()){
    case 123:
      motors.drive(defaultSpeed, FORWARD);
      break;
    case 12:
      rotate("to right");
      break;
    case 23:
      rotate("to left");
      break;
    case 1:
      lineFollower();
      break;
    case 2:
      motors.drive(defaultSpeed, FORWARD);
      break;
    case 3:
      lineFollower();
      break;
  }
}
