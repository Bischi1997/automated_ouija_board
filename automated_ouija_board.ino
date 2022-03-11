void setup() 
{
  Serial.begin(9600);
  //servo1.attach(8);
  //servo2.attach(7);
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
  
  float arm1length = 49.2; // in mm
  float arm2length = 36.0; // in mm

  const char string[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

  for(int i =0; i < strlen(string); i++ ) {
    float x;
    float y;
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
    
    double servo2Angle = calc_servo2_angle(x, y, arm1length, arm2length);
    Serial.println((String)"Servo2Angle: " + servo2Angle);
    //Servo2.write(servo2Angle)
    double servo1Angle = calc_servo1_angle(x, y, arm1length, arm2length);
    Serial.println((String)"Servo1Angle: " + servo1Angle);
    //Servo1.write(servo1Angle)
    delay(1000);
  }
  Serial.println("\t");
}

double calc_q2(float x, float y, float arm1length, float arm2length){
  double upper = (pow(abs(x), 2) + pow(y, 2) - pow(arm1length, 2) - pow(arm2length, 2));
  double lower = 2*arm1length*arm2length;
  double result = (acos(upper/lower)*180)/PI;
  return result;
}

//https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
double calc_servo2_angle(float x, float y, float arm1length, float arm2length){
  double q2 = calc_q2(x, y, arm1length, arm2length);
  return 180 - q2;
}

double calc_servo1_angle(float x, float y, float arm1length, float arm2length){
  double q2 = calc_q2(x, y, arm1length, arm2length);
  double temp = ((arm2length*(sin(q2)*180)/PI))/((arm1length+arm2length)*((cos(q2)*180)/PI));

  //Serial.println(q2);
  //Serial.println(temp);
  
  double q1 = ((atan(y/x)*180)/PI) - ((atan(temp)*180)/PI);
  //Serial.println(q1);
  return abs(q1);
}
