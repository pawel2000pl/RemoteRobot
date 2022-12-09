#include <Servo.h>

#define PWM_ENGINE_A 2
#define PWM_ENGINE_B 3

#define ENGINE_A_IN_2 14
#define ENGINE_A_IN_1 15

#define ENGINE_STAND_BY 16

#define ENGINE_B_IN_1 17
#define ENGINE_B_IN_2 18

#define ENCODER_A 22
#define ENCODER_B 23

#define BEEP 13

#define LED 10

#define GRAB 9
#define CAMX 6
#define CAMY 5

#define VOLTAGE 20

#define ID_LEFT_ENGINE 1
#define ID_RIGHT_ENGINE 2
#define ID_CAMERA_X 3
#define ID_CAMERA_Y 4
#define ID_GRAB 5
#define ID_BEEP_ON 6
#define ID_BEEP_OFF 7
#define ID_LED_ON 8
#define ID_LED_OFF 9
#define ID_MAX_ENGINE_CORRECTION 10

#define HELLO_VALUE 85

#define ENCODER_CHANGES_PER_ROUND 40

int sign(const double x)
{ 
  if (x<0) return -1;
  if (x>0) return 1;
  return 0;
}

int signInt(const int x)
{ 
  if (x<0) return -1;
  if (x>0) return 1;
  return 0;
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double voltage;
Servo grab_servo, cam_x_servo, cam_y_servo;
double VelocityA=0, VelocityB=0;
double averVelocityA=0, averVelocityB=0;
int encoderAStatus=LOW, encoderBStatus=LOW;

double engineCorrection = 0;// A- / B+
unsigned long long int timeA = micros(), timeB = micros(), deltaTimeA = 0, deltaTimeB = 0, equalTime = 0;

double powerA = 0, powerB = 0;
int maxEngineCorrection = 64;

bool updateVelocityStatus(int encoderA, int encoderB)
{
  unsigned long long int currTime = micros();
  bool result = false;
  if (encoderA != encoderAStatus)
  {
    encoderAStatus = encoderA;
    if (encoderA)
    {
    deltaTimeA = currTime-timeA;
    VelocityA = 1000000*(4*PI/ENCODER_CHANGES_PER_ROUND)/deltaTimeA;
    timeA = currTime; 
    result = true;   
    }
  }
  else if (currTime-timeA > 100000)
      VelocityA = 0;
    
  if (encoderB != encoderBStatus)
  {
    encoderBStatus = encoderB;
    if (encoderB)
    {
    deltaTimeB = currTime-timeB;
    VelocityB = 1000000*(4*PI/ENCODER_CHANGES_PER_ROUND)/deltaTimeB;
    timeB = currTime;  
    result = true;  
    }
  }
  else if (currTime-timeB > 100000)
      VelocityB = 0;
  return result;
}

void controlEngine(int value, int in1, int in2, int pwm)
{
    digitalWrite(in1, value > 0);
    digitalWrite(in2, value < 0);
    analogWrite(pwm, value?abs(value):255);
}

void EnginesControl(int A, int B)
{
  digitalWrite(ENGINE_STAND_BY, true);
  A = constrain(A, -255, 255);
  B = constrain(B, -255, 255);
  controlEngine(A, ENGINE_A_IN_1, ENGINE_A_IN_2, PWM_ENGINE_A);
  controlEngine(B, ENGINE_B_IN_1, ENGINE_B_IN_2, PWM_ENGINE_B);
}

void grab(const int x)
{
  grab_servo.write(map(x, 0, 255, 10, 100));
}

void SetCamera(const int x, const int y)
{
  cam_x_servo.write(map(x, -255, 255, 10, 170));
  cam_y_servo.write(map(y, -255, 255, 5, 165));
}

void setup()
{
  pinMode(ENGINE_STAND_BY, OUTPUT);
  pinMode(ENGINE_A_IN_2, OUTPUT);
  pinMode(ENGINE_A_IN_1, OUTPUT);
  pinMode(ENGINE_B_IN_1, OUTPUT);
  pinMode(ENGINE_B_IN_2, OUTPUT);

  pinMode(PWM_ENGINE_A, OUTPUT);
  pinMode(PWM_ENGINE_B, OUTPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  pinMode(BEEP, OUTPUT);

  pinMode(GRAB, OUTPUT);
  grab_servo.attach(GRAB);
  pinMode(CAMX, OUTPUT);
  cam_x_servo.attach(CAMX);
  pinMode(CAMY, OUTPUT);
  cam_y_servo.attach(CAMY);

  digitalWrite(ENGINE_STAND_BY, true);
  pinMode(VOLTAGE, INPUT);

  grab(0);
  SetCamera(0, 0);

  Serial.begin(115200);
}

unsigned long long int LastMessageTime = 0, Tick = 0, LastSendInfoTime = 0;
unsigned long long int startTime = millis();
unsigned long long int loopTime = micros();
int leftEngine = 0, rightEngine = 0;
bool beep = false, led = false;
int cameraX = 0, cameraY = 0;
signed char ID = 0;
int Value = 0;

double updateLoopTime()
{
  unsigned long long int current = micros();
  double result = 1e-6 * (current - loopTime);
  loopTime = current;
  return result;
}

void loop()
{
  voltage = (double)(analogRead(VOLTAGE)) * 6.6 / ((double)(1024));

  do {

    while (Serial.available() && Serial.read() == HELLO_VALUE)
    {
      ID = Serial.read();
      Value = (int)((signed char)Serial.read());
      int ecs = Serial.read();
      int cs = ((ID+1)*(Value+1)) & 255;
      if (cs == ecs)
      {
        switch (ID)
        {
          case ID_LEFT_ENGINE : leftEngine = 2 * Value; break;
          case ID_RIGHT_ENGINE : rightEngine = 2 * Value; break;
          case ID_CAMERA_X : cameraX = -2 * Value; break;
          case ID_CAMERA_Y : cameraY = -2 * Value; break;
          case ID_GRAB : grab(2 * Value); break;
          case ID_BEEP_ON: beep = true; break;
          case ID_BEEP_OFF: beep = false; break;
          case ID_LED_ON: led = true; break;
          case ID_LED_OFF: led = false; break;
          case ID_MAX_ENGINE_CORRECTION: maxEngineCorrection = constrain(Value, 0, 127); break;
        }
        LastMessageTime = millis();
        leftEngine = constrain(leftEngine+signInt(leftEngine), -255, 255);
        rightEngine = constrain(rightEngine+signInt(rightEngine), -255, 255);
      }
    }
    
  
    if (millis() - LastMessageTime > 1000)
    {
      ID = 0;
      leftEngine = 0;
      rightEngine = 0;
      beep = false;
      led = false;
      if (millis() - LastMessageTime > 10000)
      { 
        digitalWrite(ENGINE_STAND_BY, false);
        delay(300);
      }  
    }

  } while (millis() - LastMessageTime > 10000);
  
  unsigned long long int someTime = micros();
  while (micros()-someTime<15000)
  {
    updateVelocityStatus(digitalRead(ENCODER_A), digitalRead(ENCODER_B));
  }

  engineCorrection += 0.1*mapDouble((VelocityA*abs(rightEngine) - VelocityB*abs(leftEngine))*2.0/(1+abs(leftEngine)+abs(rightEngine)), 0, 12, 0, 255);
  engineCorrection = constrain(engineCorrection, -2*maxEngineCorrection, +2*maxEngineCorrection);
  powerA = abs(leftEngine)+((engineCorrection<0)?engineCorrection:0);
  powerB = abs(rightEngine)-((engineCorrection>0)?engineCorrection:0);
  EnginesControl(sign(leftEngine)*powerA, sign(rightEngine)*powerB);

  bool lowBattery = voltage <= 3.0;
  bool lowBatterySignal = lowBattery && (millis() % 300 < 150);
  digitalWrite(BEEP, beep || lowBatterySignal);
  digitalWrite(LED, led);
  SetCamera(cameraX, cameraY);

  averVelocityA = (9*averVelocityA + VelocityA)/10.0;
  averVelocityB = (9*averVelocityB + VelocityB)/10.0;
  if (millis() - LastSendInfoTime > 250)
  {
    LastSendInfoTime = millis();
    Serial.write(13);
    Serial.write(10);
    Serial.print("{\"voltage\": ");
    Serial.print(voltage);
    Serial.print(", \"velocityA\": ");
    Serial.print(averVelocityA);
    Serial.print(", \"velocityB\": ");
    Serial.print(averVelocityB);
    Serial.print("}");
    Serial.write(13);
    Serial.write(10);
    Serial.flush();
  }
  
  Tick++;
}
