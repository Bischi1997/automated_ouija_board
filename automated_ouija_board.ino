#include <Arduino.h>
#include <SoftwareSerial.h>
//#include <DFRobotDFPlayerMini.h>
#include <Servo.h>


Servo servo_base;   //give a name to the servo
Servo servo_arm;

void setup() 
{
  Serial.begin(9600);
  //servo_base.attach(D0);
  //servo_arm.attach(D3);
}

void loop() 
{
  struct PositionStruct {
    String character;
    float x;
    float y;
  };
  
  const PositionStruct alphabet[] = {
    //{char, x, y}
    {"a", -77.50, 32.00},
    {"b", -71.00, 45.00},
    {"c", -62.50, 56.00},
    {"d", -52.50, 65.50},
    {"e", -40.50, 73.50},
    {"f", -28.50, 79.00},
    {"g", -14.50, 83.00},
    {"h",  1.00, 84.00},
    {"i", 13.50, 83.00},
    {"j", 23.00, 80.50},
    {"k", 35.00, 76.00},
    {"l", 47.50, 69.00},
    {"m", 59.50, 59.00},
    {"n", 70.50, 46.00},
    {"o", 77.50, 32.50},
    {"p", -48.00, 42.00},
    {"q", -37.00, 52.00},
    {"r", -24.50, 59.00},
    {"s", -10.50, 63.00},
    {"t", 3.00, 64.00},
    {"u", 17.50, 61.50},
    {"v", 31.50, 56.00},
    {"w", 45.50, 45.00},
    {"x", -14.00, 41.50},
    {"y", 0.50, 44.00},
    {"z", 14.50, 41.50},
    {"yes", -20.00, 20.00},
    {"no", 22.00, 20.00}
  };
  
  const float arm1length = 49.2; // in mm
  const float arm2length = 36.0; // in mm
  const char string[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  //ABCDEFGHIJKLMNOPQRSTUVWXYZ
  for(int i =0; i < strlen(string); i++ ) {
    double x;
    double y;
    char c = string[i];
    Serial.println((String)"Angles for character " + c);
    int asciiVal = (int) c;
    if(asciiVal > 96){
      asciiVal = int(asciiVal) - 97;
      x = alphabet[asciiVal].x;
      y = alphabet[asciiVal].y;
    }
    else{
      asciiVal = int(asciiVal) - 65;
      x = alphabet[asciiVal].x;
      y = alphabet[asciiVal].y;
    }


    int servo1Angel = calc_servo1(x, y, arm1length, arm2length);
    int servo2Angel = calc_servo2(x, y, arm1length, arm2length);
    
    Serial.println((String)"Servo1Angle: " + servo1Angel);
    //servo_base.write(servo1Angel);
    Serial.println((String)"Servo2Angle: " + servo2Angel);
    //servo_arm.write(servo2Angel);
    

    delay(1000);
  }
  Serial.println("\t");
}


//returns 2 angles in degrees for servos
//based on https://automaticaddison.com/how-to-do-the-graphical-approach-to-inverse-kinematics/
int calc_servo1(double x, double y, double arm1length, double arm2length){
    double r = sqrt(pow(x, 2) + pow(y, 2));
    double upper = (pow(arm2length, 2) - pow(r, 2) - pow(arm1length, 2));
    double lower = -2*r*arm1length;
    double phi1 = acos(upper/lower);
    double phi2 = atan2(y,x);

    double theta1 = (phi2 - phi1);
    
    theta1 = theta1 * RAD_TO_DEG; // Joint 1

    return (int) round(theta1);
}

int calc_servo2(double x, double y, double arm1length, double arm2length){    
    double r = sqrt(pow(x, 2) + pow(y, 2));
    double phi3 = acos((pow(r, 2) - pow(arm1length, 2) - pow(arm2length, 2))/(-2.0 * arm1length * arm2length));
    double theta2 = PI - phi3;

    theta2 = theta2 * RAD_TO_DEG; // Joint 2
    theta2 = calc_servo_1_angle(theta2);
    
    return (int) round(theta2);
}

double calc_servo_1_angle (double input_angle) {
  int result;
  result = map(input_angle, -90, 90, 0, 180);
  return result;
}

//https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
