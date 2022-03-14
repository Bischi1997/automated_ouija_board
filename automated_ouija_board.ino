// Import required libraries
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncTCP.h>

//give a name to the servo
Servo servo_base;   
Servo servo_arm;

//Softwareserial für Audio Player
SoftwareSerial mySoftwareSerial(D2, D1); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);


// Replace with your network credentials
const char* ssid = "Simon";
const char* password = "qwe12345";


// Define a web server at port 80 for HTTP
AsyncWebServer server(80);


const char* PARAM_INPUT_1 = "input1";

// HTML web page to handle 1 input fields (input1, input2, input3)
//https://randomnerdtutorials.com/esp32-esp8266-input-data-html-form/
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <form action="/get">
    input1: <input type="text" name="input1">
    <input type="submit" value="Submit">
  </form><br>
</body></html>)rawliteral";


void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}




void setup(){

  //Servos Anbinden
  Serial.begin(115200);
  servo_base.attach(D0);
  servo_arm.attach(D3);


  //Verbindung zu Audioplayer prüfen
  mySoftwareSerial.begin(9600);
  
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  };
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.volume(30);  //Set volume value. From 0 to 30
  myDFPlayer.loop(1);  //Play the first mp3



  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  
  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
    }
    else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
    Serial.println(inputMessage);
    request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
                                     + inputParam + ") with value: " + inputMessage +
                                     "<br><a href=\"/\">Return to Home Page</a>");
  });

  server.onNotFound(notFound);
  server.begin();


}

//Meldungen von Audioplayer ausgeben
void printDetail(uint8_t type, int value){
switch (type) {
  case TimeOut:
    Serial.println(F("Time Out!"));
    break;
  case WrongStack:
    Serial.println(F("Stack Wrong!"));
    break;
  case DFPlayerCardInserted:
    Serial.println(F("Card Inserted!"));
    break;
  case DFPlayerCardRemoved:
    Serial.println(F("Card Removed!"));
    break;
  case DFPlayerCardOnline:
    Serial.println(F("Card Online!"));
    break;
  case DFPlayerPlayFinished:
    Serial.print(F("Number:"));
    Serial.print(value);
    Serial.println(F(" Play Finished!"));
    break;
  case DFPlayerError:
    Serial.print(F("DFPlayerError:"));
    switch (value) {
      case Busy:
        Serial.println(F("Card not found"));
        break;
      case Sleeping:
        Serial.println(F("Sleeping"));
        break;
      case SerialWrongStack:
        Serial.println(F("Get Wrong Stack"));
        break;
      case CheckSumNotMatch:
        Serial.println(F("Check Sum Not Match"));
        break;
      case FileIndexOut:
        Serial.println(F("File Index Out of Bound"));
        break;
      case FileMismatch:
        Serial.println(F("Cannot Find File"));
        break;
      case Advertise:
        Serial.println(F("In Advertise"));
        break;
      default:
        break;
    }
    break;
  default:
    break;
}

}



void loop() 
{


  if (myDFPlayer.available()) {
      printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
    };

  


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
    servo_base.write(servo1Angel);
    Serial.println((String)"Servo2Angle: " + servo2Angel);
    servo_arm.write(servo2Angel);
    

    delay(1000);
  }
  Serial.println("\t");
}


//returns 2 angles in degrees for servos
//based on https://automaticaddison.com/how-to-do-the-graphical-approach-to-inverse-kinematics/
int calc_servo1(double x, double y, double arm1length, double arm2length){
    double r = sqrt(pow(x, 2) + pow(y, 2));
    double upper = (pow(arm2length, 2) - pow(r, 2) - pow(arm1length, 2));
    double lower = (-2.0)*r*arm1length;
    double phi1 = acos(upper/lower);
    double phi2 = atan2(y,x);

    double theta1 = PI - (phi2 - phi1);
    
    theta1 = theta1 * RAD_TO_DEG; // Joint 1

    return (int) round(theta1);
}

int calc_servo2(double x, double y, double arm1length, double arm2length){    
    double r = sqrt(pow(x, 2) + pow(y, 2));
    double phi3 = acos((pow(r, 2) - pow(arm1length, 2) - pow(arm2length, 2))/((-2.0) * arm1length * arm2length));
    double theta2 = PI - phi3;

    theta2 = theta2 * RAD_TO_DEG; // Joint 2
    theta2 = calc_servo_1_angle(theta2);
    
    return (int) round(theta2);
}

double calc_servo_1_angle (double input_angle) {
  int result;
  result = map(input_angle, 0, 180, 180, 0);
  return result;
}


