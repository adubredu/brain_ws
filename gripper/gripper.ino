//HS785HB Servo step input controller

/**
 * Servo response has noticable latency when stopping or changing direction probably because the delay between steps in position
 * is too small to the servo can't keep up with the loop. When the stop command is given, the unfinished stepping loops still need
 * to finish. Chenlu recommends delat of at least 10-20 ms. The high torque on the servo also probaly causes slow deceleration,
 * because servo has a greater range of torque to decrease when approaching the desired position. Smaller torque would probably
 * decrease the deceleration time
  */

#include <Servo.h>

Servo gripper_servo;
Servo extension_servo;


char user_input = 's'; 
char state = 's';

int n;
int m; 
int p;

int edirec =1;

int INITPOS = 1000;
int MINPOS = 1000;
int MAXPOS = 2500;
int STEPSIZE = 40; 

int gripper_pin = 6;
int extension_pin = 5;



void open_gripper()
{
   gripper_servo.writeMicroseconds(1600);
}


void close_gripper()
{
  gripper_servo.writeMicroseconds(1000);
}


void extend_or_retract_gripper()
{
     if (p >= MINPOS and p <= MAXPOS) {
      p = p + (edirec * STEPSIZE);
      extension_servo.writeMicroseconds(p);
      

      if (p < MINPOS or p > MAXPOS) {
        if (p < MINPOS) {
          p = MINPOS;
        }
        else {
          p = MAXPOS;
        }
      }

      delay(100);
    }
}


void setup() {
  Serial.begin(9600);
  n = INITPOS;
  m = INITPOS;
  p = INITPOS;
  
  extension_servo.attach(extension_pin, MINPOS, MAXPOS);
  gripper_servo.attach(gripper_pin);

  extension_servo.writeMicroseconds(MINPOS);
  gripper_servo.writeMicroseconds(1600);

}

void loop() {
  while (Serial.available() > 0) {
    user_input = Serial.read();
    setState(user_input);
 
  }
}

void setState(char inputState)
{
  state = inputState;

  if (state == 'p')
  { 
    open_gripper();
  }

  if (state == 'l')
  {
    close_gripper();
  }

  //extend or retract gripper
  if (state == 'y')
  {
    edirec = 1;
    extend_or_retract_gripper();
  }

  else if (state == 'h')
  {
    edirec = -1;
    extend_or_retract_gripper();
  }
}
