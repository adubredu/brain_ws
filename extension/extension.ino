//HS785HB Servo step input controller

/**
 * Servo response has noticable latency when stopping or changing direction probably because the delay between steps in position
 * is too small to the servo can't keep up with the loop. When the stop command is given, the unfinished stepping loops still need
 * to finish. Chenlu recommends delat of at least 10-20 ms. The high torque on the servo also probaly causes slow deceleration,
 * because servo has a greater range of torque to decrease when approaching the desired position. Smaller torque would probably
 * decrease the deceleration time
  */

#include <Servo.h>
Servo picker_raise_servo;  // create servo object to control a servo 
Servo picker_servo;

Servo gripper_raise_servo;


char user_input = 's'; 
char state = 's';

int n;
int m; 
int p;
int direc=1; 
int gdirec=1;
int edirec =1;

int INITPOS = 1000;
int MINPOS = 1000;
int GMINPOS=1000;
int GMAXPOS = 2500;
int MAXPOS = 2500;
int STEPSIZE = 40; 
String print_message;

int gripper_raise_pin = 5;
int picker_pin = 11;
int picker_raise_pin = 10;


void open_picker()
{
   picker_servo.writeMicroseconds(1500);
}


void close_picker()
{
  picker_servo.writeMicroseconds(2500);
}


void raise_or_lower_picker()
{
     if (n >= MINPOS and n <= MAXPOS) {
      n = n + (direc * STEPSIZE);
      picker_raise_servo.writeMicroseconds(n);
      

      if (n < MINPOS or n > MAXPOS) {
//        setState('s');
        if (n < MINPOS) {
          n = MINPOS;
        }
        else {
          n = MAXPOS;
        }
      }

      delay(100);
    }
}


void raise_or_lower_gripper()
{
     if (m >= GMINPOS and m <=GMAXPOS) {
      m = m + (gdirec * STEPSIZE);
      gripper_raise_servo.writeMicroseconds(m);
      

      if (m < GMINPOS or m > GMAXPOS) {
        setState('m');
        if (m < GMINPOS) {
          m = GMINPOS;
        }
        else {
          m = GMAXPOS;
        }
      }

      delay(100);
    }
}



void setup() {
  Serial.begin(9600);
  n = INITPOS;
  m = INITPOS;
  
  
  picker_raise_servo.attach(picker_raise_pin, MINPOS, MAXPOS); 
  gripper_raise_servo.attach(gripper_raise_pin, GMINPOS, GMAXPOS);
  
  picker_servo.attach(picker_pin);
  
  picker_servo.writeMicroseconds(2500);

  picker_raise_servo.writeMicroseconds(MINPOS); 
  gripper_raise_servo.writeMicroseconds(GMINPOS);

  
//  setState('s');
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
  if (state == 'f')
  {
    direc = 1;
    raise_or_lower_picker();
  }
  
  else if (state == 'r')
  {
    direc = -1;
    raise_or_lower_picker();
  }
  
  else if (state == 's')
  {
    direc = 0;
    raise_or_lower_picker();
  }


  if (state == 'c')
  {
    close_picker();
    
  }

  if (state == 'o')
  {
    open_picker();
  }

  //raise gripper
  if (state == 'i')
  { 
    gdirec = 1;
    raise_or_lower_gripper();
    Serial.println(m);
  }

  //lower gripper
  else if (state == 'k')
  { 
    gdirec = -1;
    raise_or_lower_gripper();
  }

  else if (state == 'm')
  { 
    gdirec = 0;
    raise_or_lower_gripper();
  }

}
