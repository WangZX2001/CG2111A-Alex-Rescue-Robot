#include <AFMotor.h>

// Motor control
#define FRONT_LEFT   3 // M4 on the driver shield
#define FRONT_RIGHT  4 // M1 on the driver shield
#define BACK_LEFT    2 // M3 on the driver shield
#define BACK_RIGHT   1 // M2 on the driver shield

AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);

void move(float speed, int direction)
{
  int speed_scaled = (speed/100.0) * 255; 
  motorFL.setSpeed(speed_scaled);
  motorFR.setSpeed(speed_scaled);
  motorBL.setSpeed(speed_scaled);
  motorBR.setSpeed(speed_scaled);

  switch(direction)
    {
      case BACK:
        motorFL.run(BACKWARD);
        motorFR.run(BACKWARD);
        motorBL.run(BACKWARD);
        motorBR.run(BACKWARD); 
      break;
      case GO:
        motorFL.run(FORWARD);
        motorFR.run(FORWARD);
        motorBL.run(FORWARD);
        motorBR.run(FORWARD); 
      break;
      case CW:
        motorFL.run(FORWARD);
        motorFR.run(BACKWARD);
        motorBL.run(FORWARD);
        motorBR.run(BACKWARD); 
      break;
      case CCW:
        motorFL.run(BACKWARD);
        motorFR.run(FORWARD);
        motorBL.run(BACKWARD);
        motorBR.run(FORWARD); 
      break;
      case STOP:
      default:
        motorFL.run(STOP);
        motorFR.run(STOP);
        motorBL.run(STOP);
        motorBR.run(STOP); 
    }
}

void forward(float dist, double speed)
{
  if(dist > 0) deltaDist = dist;
  else deltaDist=9999999;
  newDist=forwardDist + deltaDist;
  dir = (TDirection) FORWARD;
  move(speed, FORWARD);
}

void backward(float dist, double speed)
{
  if(dist > 0) deltaDist = dist;
  else deltaDist=9999999; 
  newDist= reverseDist + deltaDist;
  
  dir = (TDirection) BACKWARD;
  move(speed, BACKWARD);
}

void ccw( float ang, double speed)
{
  // left
  if(ang == 0) deltaTicks=99999999;
  else deltaTicks=computeDeltaTicks(ang);
  targetTicks = leftReverseTickTurns + deltaTicks;

  dir = (TDirection) LEFT;
  move(speed, CCW);
}

void cw( float ang, double speed)
{ 
  // right

  if(ang == 0) deltaTicks=99999999;
  else deltaTicks=computeDeltaTicks(ang);
  targetTicks = rightReverseTickTurns + deltaTicks;

  dir = (TDirection) RIGHT;
  move(speed, CW);
}

void stop()
{
  dir = (TDirection) STOP;
  move(0, STOP);
}

void moveOneMotor(){
  motorFL.setSpeed(200);
  motorFR.setSpeed(200);
  motorBL.setSpeed(200);
  motorBR.setSpeed(200);

  motorFL.run(FORWARD);

  delay(10000);

  motorFR.run(FORWARD); // FR is wrong

  delay(10000);

  motorBL.run(FORWARD); // correct

  delay(10000);

  motorBR.run(FORWARD); // BR is wrong

  delay(10000);
}

void getTickPerRotation(){
  motorFL.setSpeed(200);
  motorFR.setSpeed(200);

  dir = (TDirection) FORWARD;
  motorFL.run(FORWARD);
  motorFR.run(FORWARD);
  
  delay(5000);

  Serial.println("Left:");
  Serial.println(leftForwardTicks);
  Serial.println("Right:");
  Serial.println(rightForwardTicks);

  stop();
  delay(10 * 1000);
}
