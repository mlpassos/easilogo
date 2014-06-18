//#include <Servo.h>
//#include <Stepper.h>
#include <AFMotor.h>
#include <AFStepper.h>
//#include <AccelStepper.h>

/* Version of this program */
float EaS_Version = 0.01;

static int debug_level = 1;
static int backlash = 16; // 16 +0 +0 is ballpark
static int xbacklash = backlash, ybacklash = backlash-2;

/* Development functions - broken code*/
//#define BUILTIN 1
//#define BROKEN 1

/* Conversion factor of steps per millimeter = -6 */
 // with a step of -2,   X +/-220 Y +/-135 - not quite a match for pixels in LOGO
float stepsPerMillimeter_X =  1.0; // was 2.0 //-6 * 50/170; // -6 is very close to setting the scale to absolute millimeters,
float stepsPerMillimeter_Y = -1.0; // was 2.0 //-6 * 50/170; // but multiplying by 5/17 makes us use the same coord system as the logo screen
 //float stepsPerMillimeter_Z = 200 / 17.1;

/* Unit conversion factor */
float conversionFactor = 1.0;  // 1 for mm 25.4 for inches

/* Stepper library initialization */
const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution

AF_Stepper myStepper1(stepsPerRevolution, 1 /*8,9,10,11*/);
AF_Stepper myStepper2(stepsPerRevolution, 2 /*4,5,6,7*/);            

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
 void forwardstep1() {  
  myStepper1.onestep(FORWARD, SINGLE);
}
void backwardstep1() {  
  myStepper1.onestep(BACKWARD, SINGLE);
}
// wrappers for the second motor!
void forwardstep2() {  
  myStepper2.onestep(FORWARD, SINGLE);
 }
void backwardstep2() {  
  myStepper2.onestep(BACKWARD, SINGLE);
}

// Motor shield has two motor ports, now we'll wrap them in an AccelStepper object
//AccelStepper AccelStepper1(forwardstep1, backwardstep1);
 //AccelStepper AccelStepper2(forwardstep2, backwardstep2);
//Stepper myStepper3(stepsPerRevolution, 17,16,19,18);

/* Servo functions and limits */
//Servo myServo;

//int servoPosMax=83;
//int servoPosMin=70;
 //int servoToolInc=10;
//float servoPosZfactor=1.0;

/* Mode selector for the motors (see documentation) */
int   motorMode = 0;

/* X,Y,Z in absolute steps position */
int X = 0;
int Y = 0;
//int Z = 0;

/* X,Y,Z in measurement value*/
float posX = 0.0;
float posY = 0.0;
//float posZ = 0.0;

/* Tools and Feeds and Coolants */
//int tool     = 0;
int spindle  = 0;
//int coolant1 = 0;
//int coolant2 = 0;

/* Spindle speed (M3 parameter)*/
//int spindleSpeed = 0;

int led = 13;

#define COMMAND_SIZE 128
uint8_t command_line[COMMAND_SIZE];
uint8_t sin_count=0;
uint16_t no_data = 0;
uint8_t asleep = 0;

/*
 * This file is part of EaS_controller.
 *
 * Copyright (C) 2014  D.Herrendoerfer
 *
 *   EaS_controller is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
  *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   EaS_controller is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
  *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with EaS_controller.  If not, see <http://www.gnu.org/licenses/>.
  *
 * This code is sourced from rsteppercontroller 
 *   the original code can be found here: 
 *          http://code.google.com/p/rsteppercontroller/
  */

#define MAX_COMMANDS 8 

struct command_t {
        uint8_t type; //i.e. G or M
        double  value; //string value associated
        //struct command_t *next;
};

//str: token in the form of Xnnn
 //old: head of object chain else null
//returns: head of object chain

struct command_t command_list[MAX_COMMANDS];
uint8_t commandLength = 0;

void addObj(uint8_t *str) 
{
  struct command_t *c;
   if (commandLength == MAX_COMMANDS) {
     return;
  }
  c = &command_list[commandLength++];
  c->type   = str[0];
  c->value  = strtod((const char*)&str[1], NULL);
}

void purge_commands() 
 {
  commandLength = 0;
}

void parse_commands(uint8_t *str) 
{
  uint8_t *token;
  uint8_t index = 0;
/* 
  do {
    token = (uint8_t*)strtok((char*)str, " \t"); //split on spaces and tabs
     str = NULL;
    if (token) addObj(token);
  } while (token);
*/

  while (str[index]) {
    token=str+index;
    while (str[index] > 'A' && str[index] < 'Z')
      index++;
       
    if (str[index] == ' ')
      index++;
    
    while (str[index] != ' ' && str[index] != 0)
      index++;

    if (str[index] == ' ')
      index++;
    
     addObj(token);
  }
}

//returns zero if value does not exist.
double getValue(const char x) 
{
  int i;
  //find entry
  for (i=0; i<commandLength; i++) {
    if (command_list[i].type == x) 
       break;
  }
  //did we find or run out?
  if (i==commandLength) 
    return 0;
 
  return command_list[i].value;
}


bool command_exists(const char x) 
{
  for (int i=0; i<commandLength; i++) {
     if (command_list[i].type == x) return 1;
  }
  return 0;
}

/*
 * This file is part of EaS_controller.
 *
 * Copyright (C) 2014  D.Herrendoerfer
 *
 *   EaS_controller is free software: you can redistribute it and/or modify
  *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   EaS_controller is distributed in the hope that it will be useful,
  *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
  *   along with EaS_controller.  If not, see <http://www.gnu.org/licenses/>.
 *
 ********************************************************************************
 *
  *  From the documentation: These are the supported modes for the steppers 
 *                           and light/servo drivers
 *                           
 *    M150: Set Z-Axis mode
 *    ---------------------
 *    Defines the mode how the Steppers, the servo and the laser driver are
 *    used by the controller and which codes modify their states
 *      Mode 0: Stepper 1 is X
 *              Stepper 2 is Y
 *              Stepper 3 is Z
  *              Servo is spindle speed
 *
 *              
 *      Mode 1: Stepper 1 is X
 *              Stepper 2 is Y
 *              Stepper 3 is Y
 *              Servo     is Z (down for Z<0)
  *              Laser     is Z (on for Z<0)
 *    
 *      Mode 2: Stepper 1 is X
 *              Stepper 2 is Y
 *              Stepper 3 is Y
 *                Servo     is Z (+90 to -90)
 *               Laser     is driven by spindle
  *              
 *      Mode 3: Stepper 1 is X
 *              Stepper 2 is Y
 *              Stepper 3 is Y
 *              Servo     is tool
 *                Laser     is Z (on for Z<0)
 *              
  *      Sample:
 *        M150 S2   ;Sets mode 2
 *
 *
 */

/* IMPORTANT: All direct motor controls are here, everywhere else
              these functions here are used to keep this code portable 
               
              The functions are grouped into those using the native steps
              values or the measurement (float) values. */

#define MAX_SPEED 200
//#define DRILL_SPEED 0.1

float motorSpeed = MAX_SPEED;

void initMotors()
{
  int i;
  myStepper1.setSpeed(MAX_SPEED);
  myStepper2.setSpeed(MAX_SPEED);
//  myStepper3.setSpeed(MAX_SPEED);
//  AccelStepper1.setMaxSpeed(MAX_SPEED);//200.0
//  AccelStepper1.setSpeed(MAX_SPEED/4.0);//200.0
 //  AccelStepper1.setAcceleration(25.0);//100.0
//  AccelStepper2.setMaxSpeed(MAX_SPEED);//200.0
//  AccelStepper2.setSpeed(MAX_SPEED/4.0);//200.0
//  AccelStepper2.setAcceleration(25.0);//100.0

//  myServo.attach(12);
 //  myServo.write(servoPosMax);

  // initialise backlash in positive direction
  //for (i = 0; i < 50; i++) myStepper1.step(BACKWARD, SINGLE/*-1400*/);
  //for (i = 0; i < 50; i++) myStepper2.step(BACKWARD, SINGLE/*-1500*/);

  //for (i = 0; i < 50; i++) myStepper2.step(FORWARD, SINGLE/*400*/);
  //for (i = 0; i < 50; i++) myStepper1.step(FORWARD, SINGLE/*75*/);
}

void setXYSpeed(float speed) // between >0 and 1.
{
   // remember
  motorSpeed = speed;

  if (speed == 0 || speed > 1) {
    speed = MAX_SPEED;
  } else {
    speed *= MAX_SPEED;
  }
  
  myStepper1.setSpeed((int)speed);
  myStepper2.setSpeed((int)speed);
 //  myStepper3.setSpeed((int)speed);
//  AccelStepper1.setSpeed((int)speed/4.0);
//  AccelStepper2.setSpeed((int)speed/4.0);
}

void moveX(int dX)
{
  static int lastdx = 0;
  int i, correction;
  
  if ((X + dX > 250) || (X + dX < -250)) return;

  correction = 0;
  X = X + dX; // global absolute X

  if ((dX * lastdx) < 0) {
    // Backlash correction needed
    if (debug_level > 0) Serial.println("; X BACKLASH CORRECTION NEEDED");
    correction = xbacklash;
  }
  if (dX != 0) lastdx = dX;

  if (dX > 0) {
    if (debug_level > 3) Serial.print("; moveX forward: ");
    if ((correction != 0) && (debug_level > 3)) {
      Serial.print(correction);
      Serial.print('+');
    }
    if (debug_level > 3) Serial.println(dX);
    myStepper1.step(dX+correction, FORWARD, SINGLE);
  }
  if (dX < 0) {
    if (debug_level > 3) Serial.print("; moveX backward: ");
    if ((correction != 0) && (debug_level > 3)) {
      Serial.print(correction);
      Serial.print('+');
    }
    if (debug_level > 3) Serial.println(-dX);
    myStepper1.step(correction-dX, BACKWARD, SINGLE);
  }
}

void moveY(int dY)
{
  static int lastdy = 0;
  int i, correction;

  if ((Y + dY > 200) || (Y + dY < -200)) return;

  correction = 0;
  Y = Y + dY; // global absolute Y

  if (dY * lastdy < 0) {
    // Backlash correction needed
    if (debug_level > 0) Serial.println("; Y BACKLASH CORRECTION NEEDED");
    correction = ybacklash;
  }
  if (dY != 0) lastdy = dY;
    
  if (dY > 0) {
    if (debug_level > 3) Serial.print("; moveY forward: ");
    if ((correction != 0) && (debug_level > 3)) {
      Serial.print(correction);
      Serial.print('+');
    }
    if (debug_level > 3) Serial.println(dY);
    myStepper2.step(dY+correction, FORWARD, SINGLE);
  }
  if (dY < 0) {
    if (debug_level > 3) Serial.print("; moveY backward: ");
    if ((correction != 0) && (debug_level > 3)) {
      Serial.print(correction);
      Serial.print('+');
    }
    if (debug_level > 3) Serial.println(-dY);
    myStepper2.step(correction-dY, BACKWARD, SINGLE);
  }
}

void resetXY()
{
  Y = 0; // center of etch-a-sketch screen 500x400, origin lower-left corner
  X = 0;
}

void penDown()
{
  delay(30);
  digitalWrite(led, HIGH);
  //myServo.write(servoPosMin);
   delay(100);
}

void penUp()
{
  digitalWrite(led, LOW);
  //myServo.write(servoPosMax);
  delay(100);
}

//void updateServo(int servoPos)
//{
//  if (servoPos>servoPosMax)
//    servoPos = servoPosMax;
 //  if (servoPos<servoPosMin)
//    servoPos = servoPosMin;
//  
//  myServo.write(servoPos);
//}

//void servoZ()
//{
//  int servoPos = ((float)posZ*servoPosZfactor) + 90;
//  updateServo(servoPos);
 //}

//void moveZ(int dZ)
//{
//  Z = Z + dZ;  
//
//  switch(motorMode) {
//  case 0:
//    myStepper3.setSpeed((int)motorSpeed); //stepper3 is always driven with stepper2
//    myStepper3.step(dZ);
 //    break;
//  case 1:
//    digitalWrite(led,(posZ < 1) ? HIGH : LOW);
//    updateServo((posZ < 1) ? servoPosMin : servoPosMax);
//    delay(100);
//    break;
//  case 2:
//    updateServo(servoPosMin + (servoPosZfactor*posZ));
 //    delay(200);
//    break;
//  case 3:
//    digitalWrite(led,(posZ < 1) ? HIGH : LOW);
//    break;
//  }
//}

void powerdown()
{
  // Stepper 1
  myStepper1.release();
//  digitalWrite(2/*4*/,LOW);
 //  digitalWrite(3/*5*/,LOW);
//  digitalWrite(4/*6*/,LOW);
//  digitalWrite(5/*7*/,LOW);
  // Stepper 2  
  myStepper2.release();
//  digitalWrite(8,LOW);
//  digitalWrite(9,LOW);
//  digitalWrite(10,LOW);
 //  digitalWrite(11,LOW);
  // Stepper 3  
//  digitalWrite(16,LOW);
//  digitalWrite(17,LOW);
//  digitalWrite(18,LOW);
//  digitalWrite(19,LOW);
}

void updateMotorCodes()
{
  //digitalWrite(2,(coolant1 == 1) ? HIGH : LOW);
   //digitalWrite(3,(coolant2 == 1) ? HIGH : LOW);
  
  switch(motorMode) {
  case 0:
//    myServo.write(spindleSpeed);
    digitalWrite(led,(spindle == 1) ? HIGH : LOW);
    break;
  case 2:
    digitalWrite(led,(spindle == 1) ? HIGH : LOW);
     break;
  }
}

void updateToolCodes()
{
//  if (motorMode == 3) {
//    updateServo(servoPosMin + (tool * servoToolInc));
//  }
}

/* No direct IO below this line */

void moveToX(int pX)
{
  if (debug_level > 2) Serial.print("; moveToX: ");
  if (debug_level > 2) Serial.println(pX);
  moveX(pX - X);
}

void moveToY(int pY)
{
  if (debug_level > 2) Serial.print("; moveToY: ");
   if (debug_level > 2) Serial.println(pY);
  moveY(pY - Y);
}

void moveToXY(int pX, int pY)
{
  if (debug_level > 1) Serial.print("; moveToXY: ");
  if (debug_level > 1) Serial.print(pX);
   if (debug_level > 1) Serial.print(" ");
  if (debug_level > 1) Serial.println(pY);
  // TO DO: If X or Y component of move is 0, use AccelStepper
  moveX(pX - X);
  moveY(pY - Y);
}

 //void moveToXY/*Z*/(int pX, int pY, int pZ)
//{
//  if (pX - X)
//    moveX(pX - X);
//  if (pY - Y)
//    moveY(pY - Y);
//  if (pZ - Z)
//    moveZ(pZ - Z);
//}

//void drill()
//{
//  int oZ;
 //  float oSpeed;
//  
//  delay(500);
//  oZ = Z;
//  oSpeed=motorSpeed;
//  
//  setXYSpeed(DRILL_SPEED);
//  
//  moveToXY/*Z*/(X,Y,Z-(10*stepsPerMillimeter_X));
//  delay(500);
//  setXYSpeed(oSpeed);
 //  moveToXY/*Z*/(X,Y,oZ);
//}

int sgn(int value)
{
  return value < 0 ? -1 : value > 0;
}

void lineXY/*Z*/(int x2, int y2/*, int z2*/)
{
  int n, deltax, deltay, /*deltaz,*/ sgndeltax, sgndeltay, /*sgndeltaz,*/ deltaxabs, deltayabs, /*deltazabs,*/ x, y, /*z,*/ drawx, drawy/*, drawz*/;

  deltax = x2 - X;
  deltay = y2 - Y;
//  deltaz = z2 - Z;
  deltaxabs = abs(deltax);
  deltayabs = abs(deltay);
//  deltazabs = abs(deltaz);
  sgndeltax = sgn(deltax);
  sgndeltay = sgn(deltay);
 //  sgndeltaz = sgn(deltaz);
  x = deltayabs >> 1;
  y = deltaxabs >> 1;
//  z = deltazabs >> 1;
  drawx = X;
  drawy = Y;
//  drawz = Z;
  
  moveToXY/*Z*/(drawx, drawy/*, drawz*/);
   
  // dX is biggest
  if(deltaxabs >= deltayabs /*&& deltaxabs >= deltazabs*/){
    for(n = 0; n < deltaxabs; n++){
      y += deltayabs;
      if(y >= deltaxabs){
        y -= deltaxabs;
         drawy += sgndeltay;
      }
//      z += deltazabs;
//      if(z >= deltaxabs){
//        z -= deltaxabs;
//        drawz += sgndeltaz;
//      }

      drawx += sgndeltax;
      moveToXY/*Z*/(drawx, drawy/*, drawz*/);
     }
    return;
  }
  // dY is biggest
  if(deltayabs >= deltaxabs /*&& deltayabs >= deltazabs*/){
    for(n = 0; n < deltayabs; n++){
      x += deltaxabs;
      if(x >= deltayabs){
         x -= deltayabs;
        drawx += sgndeltax;
      }
//      z += deltazabs;
//      if(z >= deltayabs){
//        z -= deltayabs;
//        drawz += sgndeltaz;
//      }
      drawy += sgndeltay;
       moveToXY/*Z*/(drawx, drawy/*, drawz*/);
    }
    return;
  }
  // dZ is biggest
//  if(deltazabs >= deltaxabs && deltazabs >= deltayabs){
//    for(n = 0; n < deltazabs; n++){
 //      x += deltaxabs;
//      if(x >= deltazabs){
//        x -= deltazabs;
//        drawx += sgndeltax;
//      }
//      y += deltayabs;
//      if(y >= deltazabs){
//        y -= deltazabs;
 //        drawy += sgndeltay;
//      }
//      drawz += sgndeltaz;
//      moveToXY/*Z*/(drawx, drawy, drawz);
//    }
//    return;
//  }
}

void drawline(int x1, int y1, int x2, int y2)
{
   penUp();
  lineXY/*Z*/(x1,y1/*,5*/);
  penDown();
  lineXY/*Z*/(x2,y2/*,-5*/);
  penUp();
}

/* No direct use of step coordinates below this line */

int convertPosX(float pos)
{ 
  return (int)(pos*conversionFactor*stepsPerMillimeter_X);
 }

int convertPosY(float pos)
{ 
  return (int)(pos*conversionFactor*stepsPerMillimeter_Y);
}

//int convertPosZ(float pos)
//{ 
//  return (int)(pos*conversionFactor*stepsPerMillimeter_Z);
//}

void drawlinePos(float x1, float y1, float x2, float y2)
{
  drawline(convertPosX(x1),convertPosY(y1),convertPosX(x2),convertPosY(y2));
  posX=x2;
  posY=y2;
}

void linePos(float x2, float y2/*, float z2*/)
 {
  lineXY/*Z*/(convertPosX(x2),convertPosY(y2)/*,convertPosZ(z2)*/);
  posX=x2;
  posY=y2;
//  posZ=z2;
}

#ifdef BROKEN
void arcPos(int direction, float x2, float y2, float i, float j)
{
//      case 2://Clockwise arc
 //      case 3://Counterclockwise arc
      float angleA, angleB, angle, radius, aX, aY, bX, bY, length, x, y;

      // Center coordinates are always relative

      aX = (X - i);
      aY = (Y - j);
       bX = (x2 - i);
      bY = (y2 - j);

      Serial.println("; aX");
      Serial.println(aX);
      Serial.println("; aY");
      Serial.println(aY);

      Serial.println("; bX");
      Serial.println(bX);
      Serial.println("; bY");
      Serial.println(bY);

      if (direction == 0) { // Clockwise
        angleA = atan2(bY, bX);
        angleB = atan2(aY, aX);
      }
       else { // Counterclockwise
        angleA = atan2(aY, aX);
        angleB = atan2(bY, bX);
      }

      Serial.println("; AngleA");
      Serial.println(angleA);
      Serial.println("; AngleB");
      Serial.println(angleB);


      // Make sure angleB is always greater than angleA
      // and if not add 2PI so that it is (this also takes
      // care of the special case of angleA == angleB,
      // ie we want a complete circle)
       if (angleB <= angleA) angleB += 2 * M_PI;
      angle = angleB - angleA;

      radius = sqrt(aX * aX + aY * aY);

      Serial.println("; Radius");
      Serial.println(radius);

       length = radius * angle;

      Serial.println("; Length");
      Serial.println(length);

      int steps, s, step;
//      steps = length / ((stepsPerMillimeter_X + stepsPerMillimeter_Y)/4); // approx 0.5mm
       steps = length / 4; // approx 0.5mm

      Serial.println("; Steps");
      Serial.println(steps);


      for (s = 1; s <= steps; s++) {
        step = (direction == 1) ? s : steps - s; // Work backwards for CW
         x = i + radius * cos(angleA + angle * ((float) step / steps));
        y = j + radius * sin(angleA + angle * ((float) step / steps));

      Serial.println("; X");
      Serial.println(x);
      Serial.println("; Y");
      Serial.println(y);

        // straight line to new points
        linePos(x,y/*,posZ*/);
      }
      
  linePos(x2,y2/*,posZ*/);
}
#endif /*BROKEN*/

void movePosXY/*Z*/ (float x2, float y2, /*float z2,*/ float speed )
 {
  setXYSpeed(speed);
  linePos(x2, y2/*, z2*/);
}

/*
 * This file is part of EaS_controller.
 *
 * Copyright (C) 2014  D.Herrendoerfer
 *
 *   EaS_controller is free software: you can redistribute it and/or modify
  *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   EaS_controller is distributed in the hope that it will be useful,
  *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
  *   along with EaS_controller.  If not, see <http://www.gnu.org/licenses/>.
 */

/*Settings */

boolean abs_mode = true;
float _feedrate = 0.6;


 #define DEBUG 1

struct f_PosXY/*Z*/ {
  float x;
  float y;
//  float z;
};

void setXY/*Z*/(struct f_PosXY/*Z*/ *fp) {
  fp->x = (command_exists('X')) ? (getValue('X') + ((abs_mode) ? 0 : posX)) : posX;
   fp->y = (command_exists('Y')) ? (getValue('Y') + ((abs_mode) ? 0 : posY)) : posY;
//  fp->z = (command_exists('Z')) ? (getValue('Z') + ((abs_mode) ? 0 : posZ)) : posZ;
}


void process_command(uint8_t *command_string)
{
  uint8_t code;
  uint16_t k;
  float temp;
  f_PosXY/*Z*/ fp;


  //the character / means delete block... used for comments and stuff.
  if (command_string[0] == '/') {
     Serial.println("ok");
    return;
  }

  purge_commands(); //clear old commands
  parse_commands(command_string); //create linked list of arguments

  if (command_exists('G')) {
     code = getValue('G');

    switch(code) {
    case 0: //Rapid Motion
      setXY/*Z*/(&fp);
      movePosXY/*Z*/ (fp.x, fp.y, /*fp.z,*/ 0);
      break;
    case 1: //Coordinated Motion
       setXY/*Z*/(&fp);
      if (command_exists('F')) _feedrate = getValue('F'); //feedrate persists till changed.
      movePosXY/*Z*/ (fp.x, fp.y, /*fp.z,*/ _feedrate);
      break;
#ifdef BROKEN
     case 2: //Coordinated Motion
    case 3: //Coordinated Motion counterclockwise
      float I,J;
      setXY/*Z*/(&fp);
      if (command_exists('F')) _feedrate = getValue('F'); //feedrate persists till changed.
       if (command_exists('I')) I = getValue('I');
      if (command_exists('J')) J = getValue('J');
      arcPos((code-2), fp.x, fp.y, fp.x+I, fp.y+J);
      break; 
#endif /*BROKEN*/   
     case 4: //Dwell
      delay((int)getValue('P'));
      break;
    case 20: //Inches for Units
      conversionFactor = 25.4;  // 1 for mm 25.4 for inches
      break;
    case 21: //mm for Units
       conversionFactor = 1;  // 1 for mm 25.4 for inches      
      break;
    case 28: //go home.
      movePosXY/*Z*/ (0, 0, /*0,*/ 0);
      break;
    case 30://go home via an intermediate point.
      setXY/*Z*/(&fp);
       movePosXY/*Z*/ (fp.x, fp.y, /*fp.z,*/ 0);
      movePosXY/*Z*/ (0, 0, /*0,*/ 0);
      break;
    case 81: // drilling operation
      setXY/*Z*/(&fp);
      linePos (fp.x, fp.y/*, fp.z*/ );
//      drill();
       break;
    case 90://Absolute Positioning
      abs_mode = true;
      break;
    case 91://Incremental Positioning
      abs_mode = false;
      break;
    case 92://Set as home
      resetXY();
      posX = 0.0; // center of etch-a-sketch screen
      posY = 0.0;
//      posZ = 0.0;
      break;
    case 93://Inverse Time Feed Mode
      break;  //TODO: add this
    case 94://Feed per Minute Mode
      break;  //TODO: add this
     default:
      Serial.print("; huh? G");
      Serial.println(code,DEC);
      return;
    }
  }

//  if (command_exists('T')) {
//    tool = getValue('&');
//    updateToolCodes();
 //  }
  
  if (command_exists('M')) {
    code = getValue('M');

    switch(code) {
    case 3:
    case 4:
//      if (command_exists('S')) spindleSpeed = (int)getValue('S');
       spindle=1;
      break;
    case 5:
      spindle=0;
      break;
    case 7:
    case 8:
//      coolant1=1;
      break;
    case 9:
//      coolant1=0;
      break;
    case 10:
 //      coolant2=1;
      break;
    case 11:
//      coolant2=0;
      break;
    case 18:
      powerdown();
      break;
    case 150:
      if (command_exists('S')) motorMode = (int)getValue('S');
       break;
//    case 151:
//      if (command_exists('S')) servoPosMax = (int)getValue('S');
//      break;
//    case 152:
//      if (command_exists('S')) servoPosMin = (int)getValue('S');
 //      break;
//    case 153:
//      if (command_exists('S')) servoToolInc = (int)getValue('S');
//      break;
//    case 154:
//      if (command_exists('S')) servoPosZfactor = getValue('S');
 //      break;
    case 160:
      if (command_exists('S')) stepsPerMillimeter_X = getValue('S');
      break;
    case 161:
      if (command_exists('S')) stepsPerMillimeter_Y = getValue('S');
       break;
//    case 162:
//      if (command_exists('S')) stepsPerMillimeter_Z = getValue('S');
//      break;
    }
    updateMotorCodes();
  }
  
  if (command_exists('&')) {
     code = getValue('&');

    switch(code) {
    case 0: // Self Test and calibrate pattern
      //calibratePattern();
      break;
    case 1: // Diag info
      dumpSettings();
      break;
     }
  }

  Serial.println("ok");
}


void setup() {
  Serial.begin(9600);
  
  // LED (Laser output)
  pinMode(led, OUTPUT);
  // General pupose (coolant 1) output
//  pinMode(2, OUTPUT);
   // General pupose (coolant 2) output
//  pinMode(3, OUTPUT);
  
  /* Init the steppers and servo */
  initMotors();
  powerdown();
}

void clear_command_string() {
  for (int i=0; i<COMMAND_SIZE; i++) 
     command_line[i] = 0;
  sin_count = 0;
}

void dumpSettings()
{
  Serial.print("Hello, this is Etch-a-Sketch GCode controller version ");
  Serial.println(EaS_Version,1);
  Serial.println("");
   Serial.println("current settings are:");
  Serial.println("");
  Serial.print("motorMode:");
  Serial.println(motorMode);
  Serial.println("Stepper positions:");
  Serial.print("X:");
   Serial.println(X);
  Serial.print("Y:");
  Serial.println(Y);
//  Serial.print("Z:");
//  Serial.println(Z);
  Serial.println("Absolute positions:");
  Serial.print("posX:");
   Serial.println(posX,3);
  Serial.print("posY:");
  Serial.println(posY,3);
//  Serial.print("posZ:");
//  Serial.println(posZ,3);
  Serial.println("Stepper settings:");
  Serial.print("stepsPerMillimeter_X:");
   Serial.println(stepsPerMillimeter_X, 3);
  Serial.print("stepsPerMillimeter_Y:");
  Serial.println(stepsPerMillimeter_Y, 3);
//  Serial.print("stepsPerMillimeter_Z:");
//  Serial.println(stepsPerMillimeter_Z, 3);
 //  Serial.println("Servo settings:");
//  Serial.print("servoPosZfactor:");
//  Serial.println(servoPosZfactor, 3);
//  Serial.print("servoPosMax:");
//  Serial.println(servoPosMax);
 //  Serial.print("servoPosMin:");
//  Serial.println(servoPosMin);
//  Serial.print("servoToolInc:");
//  Serial.println(servoToolInc);
  Serial.print("Units: 1 unit = ");
  Serial.print(conversionFactor);
   Serial.println("mm");
  Serial.print("EOM."); 
}

void calibratePattern()
{
  int i=0;

  /*Draw a line pattern in absolute step mode*/
  drawline(50,50,1000,50);
  drawline(1000,150,50,150);
   drawline(50,250,1000,250);
  drawline(1000,350,50,350);

  for (i=50; i<1000; i+=50)
    drawline(i,30,i,70);

  for (i=100; i<1000; i+=100)
    drawline(i,30,i,170);

  for (i=200; i<1000; i+=200)
     drawline(i,30,i,270);

  for (i=400; i<1000; i+=400)
    drawline(i,30,i,370);

  /*Draw a line pattern in 5,10,20,40mm spacing*/
  drawlinePos(5,25,60,25);
  drawlinePos(60,30,5,30);
  drawlinePos(5,35,60,35);
   drawlinePos(60,40,5,40);
  
  for (i=5; i<60; i+=5)
    drawlinePos(i,23,i,27);
  
  for (i=10; i<60; i+=10)
    drawlinePos(i,23,i,32);

  for (i=20; i<60; i+=20)
    drawlinePos(i,23,i,37);

  for (i=40; i<60; i+=40)
    drawlinePos(i,23,i,42);
}


void loop() {
  uint8_t c;
  
  Serial.println("ready");

  while (true) {
    //read in characters if we got them.
     if (Serial.available() > 0)   {
      c = (uint8_t)Serial.read();
      no_data = 0;
      asleep = 0;
      command_line[sin_count++] = c;
    }
    else {
      no_data++;
      delayMicroseconds(150);
     }
  
    if (sin_count && (c == '\n' || no_data > 100)) {
      command_line[sin_count] = 0;
      process_command(command_line);
      //sin_count=0; 
      clear_command_string(); 
     }
  
    if (no_data == 60000)  {
      if (!asleep) {
        powerdown();
        if (debug_level >= 2) Serial.println("; Torque disengaged...");
        asleep=1;
      }
    }
   }
}


