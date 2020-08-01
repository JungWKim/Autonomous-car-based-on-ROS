#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

/*----------------
 * How to control motor
 * 
 * 1. 1A(HIGH) && 2A(LOW) >> reverse clock direction
 * 2. 1A(LOW) && 2A(HIGH) >> clock direction
 * 3. 3B(HIGH) && 4B(LOW) >> reverse clock direction
 * 4. 3B(LOW) && 4B(HIGH) >> clock direction
 * 
 */



//  Assigning pin numbers
//-----------------------------------------------

#define EA_R   0
#define A1_R  53
#define A2_R  51

#define EB_R  10
#define B3_R  43
#define B4_R  41

#define EA_L   8
#define A1_L  22
#define A2_L  24

#define EB_L   3
#define B3_L   4
#define B4_L   5



//   Basic declaration to use rosserial
//------------------------------------------------
ros::NodeHandle  nh;
//  1. Setup all pins as output
//  2. Push and pull topics
//----------------------------------------------
void setup()
{


  nh.initNode();

  analogWrite(EA_L, 40);
  analogWrite(EA_R, 40);
  analogWrite(EB_R, 40);
  analogWrite(EB_L, 40);
  
  pinMode(B4_R, OUTPUT);
  pinMode(B3_R, OUTPUT);

  pinMode(A1_L, OUTPUT);
  pinMode(A2_L, OUTPUT);
  
  pinMode(A2_R, OUTPUT);
  pinMode(A1_R, OUTPUT);

  pinMode(B3_L, OUTPUT);
  pinMode(B4_L, OUTPUT);
}


//   Publish status information According to the status_analyze()
//---------------------------------------------
void loop()
{    
#if 1

  digitalWrite(A1_L, HIGH);
  digitalWrite(A2_L, LOW);
  
#endif
  
#if 1

  digitalWrite(B3_L, HIGH);
  digitalWrite(B4_L, LOW);
  
#endif

#if 1

  digitalWrite(A1_R, LOW);
  digitalWrite(A2_R, HIGH);

#endif

#if 1

  digitalWrite(B3_R, LOW);
  digitalWrite(B4_R, HIGH);
  
#endif
  
  nh.spinOnce();
  delay(100);
}
