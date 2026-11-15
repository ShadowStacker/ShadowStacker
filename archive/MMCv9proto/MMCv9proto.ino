// Macro Motor Controller
// For controlling a stepper motor-driven focus-stacking camera rig for macro photography
// Can also be used as just an intervalometer. With Serial Bluetooth, and WIFI OTA updability
// David M. Helton - 2023 ESP32 Edition

// WIFI Update libraries
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
//WIFI Setup stuff:
const char* host = "macro";
char ssid[32] = "ssid";
char password[32] = "pw";

WebServer server(80);

/*
 * Login page
 */

const char* loginIndex =
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>Macro Stacker Wifi BIN Updater</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td><center><input type='submit' onclick='loadServerIndex()' value='Login'></td></center>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function loadServerIndex() {"
        "window.open('/serverIndex')" // redirect to serverIndex page
    "}"
"</script>";

/*
 * Server Index Page
 */

const char* serverIndex =
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')"
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";

#include <Stepper.h>
#include "BluetoothSerial.h"
//Bluetooth config nonesense:
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif
BluetoothSerial SerialBT;

//Change this to fit the number of steps per revolution for your motor
const int stepsPerRevolution = 2038; 
//Pin that controls the motor direction
const int directionPin = 25;  //25 on broken protoboard
//Pin that controls the motor steps
const int stepPin = 26;
//Pin that controls enables motor power
const int enablePin = 33; //27 on broken protoboard
//Variable to store the RPM of the motor
float motorRPM = 8;
//Variable to store whether the motor is running or not
bool running = false;

//The number of the shutter pin
const int shutterPin = 18;  //Was 18
//The led pin
const int ledPin = 2;
//The number of the pushbutton pin to move clockwise
const int ccbuttonPin = 34; //was 34
//Variable for reading the ccpushbutton status
int ccbuttonState = 0;  
//The number of the pushbutton pin to move counterclockwise  
const int ccwbuttonPin = 35; //was 35
//Variable for reading the ccwpushbutton status
int ccwbuttonState = 0;
//The number of the pushbutton pin to manually activate shutter
const int shutterbuttonPin = 32; //Known as Trig on PCB, was 32
//Variable for reading the shutter pushbutton status
int shutterbuttonState = 0;

//Variable to store a running count of shots
int runCount = 0;
//Variable to store the number of shots to perform
int runLimit = 1;
//Variable to store delay time after motor movevement, wait for vibration
long delayTime = 1000;
//Variable to store delay time after shot, increase for longer exposures, or leave as to let the camera process
long shotdelayTime = 900;
//Variable to store number of motor steps per run
int numSteps = stepsPerRevolution;
//Constant to calculate linear distances, 4mm/2038
const float linearDistance = 0.00196270854;
//Variable to store time to completion
long timeLeft = 0;
//Variable to calc distance per run
float distancePershot = 0;
//Variable to calc total run distance
float distanceTotal = 0;
//Variable to calc distance left to run
float distanceLeft = 0;
//Variable to calc total delay time til completion
long delaytimeLeft = 0;
//Variable to calc total motor running time
float motortimeLeft = 0;

void setup() {
  //Initialize the serial ports:
  Serial.begin(115200);
  SerialBT.begin("Macro Proto"); //Bluetooth device name
  //Set up pins for camera trigger and led output
  pinMode(shutterPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  //Set pins for motor direction, step, and power enable to output
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  //Set pins as as button inputs
  pinMode(ccbuttonPin, INPUT);
  pinMode(ccwbuttonPin, INPUT);
  pinMode(shutterbuttonPin, INPUT);
  //Print the Menu
  Serial.println("Macro Motor Controller - David M. Helton 2023");
  Serial.println("Serial Bluetooth is enabled.");
  Serial.println("Commands: s - start, p - stop");
  Serial.println("l - number of shots to take(default=1)");
  Serial.println("y - delay time after shot (default=900ms)");
  Serial.println("t - delay time after moving motor (default=1000ms)");
  Serial.println("q - motor steps per shot (default=2038=1rev=4mm)");
  Serial.println("d - motor direction (default=1 cc, 0 ccw)");
  Serial.println("r - motor speed in RPM (default=8,max=15)");
  Serial.println("i - quick snap");
  Serial.println("u - upload bin file via wifi");
  Serial.println("x - reset");
  Serial.println("h - display this menu again");
  //blink as a simple POST
  digitalWrite(ledPin, HIGH);
  digitalWrite(directionPin, HIGH);
  digitalWrite(enablePin, HIGH);
  digitalWrite(stepPin, LOW);
  delay(333);
  digitalWrite(ledPin, LOW);
  digitalWrite(stepPin, HIGH);
  digitalWrite(directionPin, HIGH);
  digitalWrite(enablePin, LOW);
  delay(333);
  digitalWrite(ledPin, HIGH);
  digitalWrite(directionPin, LOW);
  digitalWrite(enablePin, HIGH);
  digitalWrite(stepPin, HIGH);
  delay(333);
  digitalWrite(ledPin, LOW);
  digitalWrite(directionPin, HIGH);
  digitalWrite(enablePin, HIGH);
  digitalWrite(stepPin, HIGH);
}

void loop() {
  //Check for BT serial input
  if (SerialBT.available() > 0) {
    char input = SerialBT.read();
    //Check input for commands
    if (input == 's') { // start command
      running = true;
      digitalWrite(enablePin, LOW);
      SerialBT.println("Started");
      Serial.println("Started");
    } else if (input == 'h') { // help command
      SerialBT.println("Macro Motor Controller - David M. Helton 2023");
      SerialBT.println("Bluetooth Version");
      SerialBT.println("Commands: s - start, p - stop");
      SerialBT.println("l - number of shots to take");
      SerialBT.println("y - delay time after shot (default=900ms)");
      SerialBT.println("t - delay time after moving motor (default=1000ms)");
      SerialBT.println("q - motor steps per shot (default=2038=1rev=4mm)");
      SerialBT.println("d - motor direction (default=1 cc, 0 ccw)");
      SerialBT.println("r - motor speed in RPM (default=8,max=15)");
      SerialBT.println("i - quick snap");
      SerialBT.println("u - upload bin file via wifi");
      SerialBT.println("x - reset");
      SerialBT.println("h - for help, but you knew that already");
    } else if (input == 'p') { // stop command
      running = false;
      SerialBT.println("Stopped");
      Serial.println("Stopped");
      runCount = 0;
    } else if (input == 'r') { // set RPM command
      motorRPM = SerialBT.parseFloat();
      SerialBT.print("RPM set to: ");
      SerialBT.println(motorRPM);
      Serial.print("RPM set to: ");
      Serial.println(motorRPM);
    } else if (input == 'l') { // set shots to run limit command
      runLimit = SerialBT.parseInt();
      SerialBT.print("Shot limit set to: ");
      SerialBT.println(runLimit);
      Serial.print("Shot limit set to: ");
      Serial.println(runLimit);
    } else if (input == 't') { // set delay after motor time command
      delayTime = SerialBT.parseInt();
      SerialBT.print("Delay after motor set to: ");
      SerialBT.println(delayTime);
      Serial.print("Delay after motor set to: ");
      Serial.println(delayTime);
    } else if (input == 'i') { // quick snap
      digitalWrite(shutterPin, HIGH);
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(shutterPin, LOW);
      digitalWrite(ledPin, LOW);
    } else if (input == 'y') { // set delay after shot time command
      shotdelayTime = SerialBT.parseInt();
      SerialBT.print("Delay after shot set to: ");
      SerialBT.println(shotdelayTime);
      Serial.print("Delay after shot set to: ");
      Serial.println(shotdelayTime);
    } else if (input == 'q') { // set number of steps command
      numSteps = SerialBT.parseInt();
      SerialBT.print("Number of motor steps per shot set to: ");
      SerialBT.println(numSteps);
      SerialBT.print("Linear distance per shot = ");
      SerialBT.print(linearDistance * numSteps, 4);
      SerialBT.println("mm");
      Serial.print("Number of motor steps per shot set to: ");
      Serial.println(numSteps);
      Serial.print("Linear distance per shot = ");
      Serial.print(linearDistance * numSteps, 4);
      Serial.println("mm");
    } else if (input == 'd') { // set direction command
      int direction = SerialBT.parseInt();
      if (direction == 1) {
        digitalWrite(directionPin, HIGH);
        SerialBT.println("Direction set to clockwise");
        Serial.println("Direction set to clockwise");
      } else if (direction == 0) {
        digitalWrite(directionPin, LOW);
        SerialBT.println("Direction set to counterclockwise");
        Serial.println("Direction set to counterclockwise");
      } else {
        SerialBT.println("Invalid direction input, use 'c' for clockwise or 'cc' for counterclockwise");
        Serial.println("Invalid direction input, use 'c' for clockwise or 'cc' for counterclockwise");
      }
    } else if (input == 'x') { // reset
      SerialBT.println("Restarting...");
      Serial.println("Restarting...");
      ESP.restart();
    } else if (input == 'u') { // connect to wifi and upload bin
      SerialBT.println("Please enter WIFI SSID ");
      while (!SerialBT.available()); // wait for input
      String inputid = SerialBT.readStringUntil('\n');
      inputid.trim();
      inputid.toCharArray(ssid, 32); // copy the input string to the ssid char array
      SerialBT.print("WIFI SSID Set to ");
      SerialBT.println(ssid);
      SerialBT.println("Please enter WIFI password ");
      SerialBT.read();
      while (!SerialBT.available()); // wait for input
      String inputpw = SerialBT.readStringUntil('\n');
      inputpw.trim();
      inputpw.toCharArray(password, 32);
      SerialBT.print("WIFI password Set to ");
      SerialBT.println(password);
              // Connect to WiFi network
         WiFi.begin(ssid, password);
         SerialBT.print("Connecting");

         // Wait for connection
         while (WiFi.status() != WL_CONNECTED) {
           delay(500);
           SerialBT.print(".");
         }
         SerialBT.println("");
         SerialBT.print("Connected to ");
         SerialBT.println(ssid);
         SerialBT.print("IP address: ");
         SerialBT.println(WiFi.localIP());

        /*use mdns for host name resolution*/
        if (!MDNS.begin(host)) { //http://macro.local
          SerialBT.println("Error setting up MDNS responder!");
          while (1) {
            delay(1000);
          }
        }
        SerialBT.println("mDNS responder started at ");
        /*return index page which is stored in serverIndex */
        server.on("/", HTTP_GET, []() {
          server.sendHeader("Connection", "close");
          server.send(200, "text/html", loginIndex);
        });
        server.on("/serverIndex", HTTP_GET, []() {
          server.sendHeader("Connection", "close");
          server.send(200, "text/html", serverIndex);
        });
        /*handling uploading firmware file */
        server.on("/update", HTTP_POST, []() {
          server.sendHeader("Connection", "close");
          server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
          ESP.restart();
        }, []() {
          HTTPUpload& upload = server.upload();
          if (upload.status == UPLOAD_FILE_START) {
            SerialBT.printf("Update: %s\n", upload.filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
              Update.printError(SerialBT);
            }
          } else if (upload.status == UPLOAD_FILE_WRITE) {
            /* flashing firmware to ESP*/
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
              Update.printError(SerialBT);
            }
          } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) { //true to set the size to the current progress
              SerialBT.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            } else {
              Update.printError(SerialBT);
            }
          }
        });
      server.begin();
      while (true) {server.handleClient();
        digitalWrite(ledPin, HIGH);
        delay(1);
      }
    }
  }
  //Check for serial input
  if (Serial.available() > 0) {
    char input = Serial.read();
    //Check input for commands
    if (input == 's') { // start command
      running = true;
      digitalWrite(enablePin, LOW);
      Serial.println("Started");
    } else if (input == 'p') { // stop command
      running = false;
      Serial.println("Stopped");
      runCount = 0;
    } else if (input == 'h') { // help command
      Serial.println("Commands: s - start, p - stop");
      Serial.println("l - number of shots to take");
      Serial.println("y - delay time after shot (default=900ms)");
      Serial.println("t - delay time after moving motor (default=1000ms)");
      Serial.println("q - motor steps per shot (2038=1rev=4mm)");
      Serial.println("d - motor direction (default=1 cc, 0 ccw)");
      Serial.println("r - motor speed in RPM (default=8,max=15)");
      Serial.println("i - quick snap");
      Serial.println("u - upload bin file via wifi");
      Serial.println("x - reset");
      Serial.println("h - display this menu again");
    } else if (input == 'r') { // set RPM command
      motorRPM = Serial.parseFloat();
      Serial.print("RPM set to: ");
      Serial.println(motorRPM);
    } else if (input == 'l') { // set shots to run limit command
      runLimit = Serial.parseInt();
      Serial.print("Shot limit set to: ");
      Serial.println(runLimit);
    } else if (input == 't') { // set delay after motor time command
      delayTime = Serial.parseInt();
      Serial.print("Delay after motor set to: ");
      Serial.println(delayTime);
    } else if (input == 'i') { // quick snap
      digitalWrite(shutterPin, HIGH);
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(shutterPin, LOW);
      digitalWrite(ledPin, LOW);
    } else if (input == 'y') { // set delay after shot time command
      shotdelayTime = Serial.parseInt();
      Serial.print("Delay after shot set to: ");
      Serial.println(shotdelayTime);
    } else if (input == 'q') { // set number of steps command
      numSteps = Serial.parseInt();
      Serial.print("Number of motor steps per shot set to: ");
      Serial.println(numSteps);
      Serial.print("Linear distance per shot = ");
      Serial.print(linearDistance * numSteps, 4);
      Serial.println("mm");
    } else if (input == 'd') { // set direction command
      int direction = Serial.parseInt();
      if (direction == 1) {
        digitalWrite(directionPin, HIGH);
        Serial.println("Direction set to clockwise");
      } else if (direction == 0) {
        digitalWrite(directionPin, LOW);
        Serial.println("Direction set to counterclockwise");
      } else {
        Serial.println("Invalid direction input, use 'c' for clockwise or 'cc' for counterclockwise");
      }
    } else if (input == 'x') { // reset
      Serial.println("Restarting...");
      ESP.restart();
    } else if (input == 'u') { // connect to wifi and upload bin
      Serial.println("Please enter WIFI SSID: ");
      Serial.read();
      while (!Serial.available()); // wait for input
      //Serial.setTimeout(30000);
      String input = Serial.readStringUntil('\n');
      input.trim();
      input.toCharArray(ssid, 32); // copy the input string to the ssid char array
      Serial.print("WIFI SSID Set to: ");
      Serial.println(ssid);
      Serial.println("Please enter WIFI password: ");
      Serial.read();
      while (!Serial.available()); // wait for input
      String inputpw = Serial.readStringUntil('\n');
      inputpw.trim();
      inputpw.toCharArray(password, 32);
      Serial.print("WIFI password Set to: ");
      Serial.println(password);
              // Connect to WiFi network
         WiFi.begin(ssid, password);
         Serial.print("Connecting");

         // Wait for connection
         while (WiFi.status() != WL_CONNECTED) {
           delay(500);
           Serial.print(".");
         }
         Serial.println("");
         Serial.print("Connected to ");
         Serial.println(ssid);
         Serial.print("IP address: ");
         Serial.println(WiFi.localIP());

        /*use mdns for host name resolution*/
        if (!MDNS.begin(host)) { //http://macro.local
          Serial.println("Error setting up MDNS responder!");
          while (1) {
            delay(1000);
          }
        }
        Serial.println("mDNS responder started at ");
        /*return index page which is stored in serverIndex */
        server.on("/", HTTP_GET, []() {
          server.sendHeader("Connection", "close");
          server.send(200, "text/html", loginIndex);
        });
        server.on("/serverIndex", HTTP_GET, []() {
          server.sendHeader("Connection", "close");
          server.send(200, "text/html", serverIndex);
        });
        /*handling uploading firmware file */
        server.on("/update", HTTP_POST, []() {
          server.sendHeader("Connection", "close");
          server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
          ESP.restart();
        }, []() {
          HTTPUpload& upload = server.upload();
          if (upload.status == UPLOAD_FILE_START) {
            Serial.printf("Update: %s\n", upload.filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
              Update.printError(Serial);
            }
          } else if (upload.status == UPLOAD_FILE_WRITE) {
            /* flashing firmware to ESP*/
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
              Update.printError(Serial);
            }
          } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) { //true to set the size to the current progress
              Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            } else {
              Update.printError(Serial);
            }
          }
        });
      server.begin();
      while (true) {server.handleClient();
        digitalWrite(ledPin, HIGH);
        delay(1);
      }
    }
  }
  //Read the state of the pushbutton values:
  ccbuttonState = digitalRead(ccbuttonPin);
  ccwbuttonState = digitalRead(ccwbuttonPin);
  shutterbuttonState = digitalRead(shutterbuttonPin);
  //Check if pushbuttons are pressed. If it is, move motor:
  if (ccbuttonState == HIGH) {
    digitalWrite(enablePin, LOW);
    digitalWrite(directionPin, LOW);
    singleStep();
    digitalWrite(enablePin, HIGH);
  }
    if (ccwbuttonState == HIGH) {
    digitalWrite(enablePin, LOW);
    digitalWrite(directionPin, HIGH);
    singleStep();
    digitalWrite(enablePin, HIGH);
  }
    if (shutterbuttonState == HIGH) {
    digitalWrite(shutterPin, HIGH);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(shutterPin, LOW);
    digitalWrite(ledPin, LOW);
  }
  //Check if things are running and activate shutter and onboard led
  if (running) {
    digitalWrite(shutterPin, HIGH);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(shutterPin, LOW);
    digitalWrite(ledPin, LOW);
    delay(shotdelayTime);
    //Calculate the shots and estimated time before finishing
    Serial.print("Shot Taken!, Shots left: ");    
    Serial.println(runLimit - runCount);
    SerialBT.print("Shot Taken!, Shots left: ");    
    SerialBT.println(runLimit - runCount);
    delaytimeLeft = (runLimit - runCount) * (delayTime + shotdelayTime +100);
    motortimeLeft = (runLimit - runCount) * (60000/motorRPM/stepsPerRevolution * numSteps);
    timeLeft = (delaytimeLeft + motortimeLeft) /1000;
    Serial.print("Estimated time left: ");
    SerialBT.print("Estimated time left: ");
    if (timeLeft < 60) {
      Serial.print(timeLeft);
      Serial.println(" (S)");
      SerialBT.print(timeLeft);
      SerialBT.println(" (S)");
    } else if (timeLeft < 3600) {
      int minutes = timeLeft / 60;
      int seconds = timeLeft % 60;
      Serial.print(minutes);
      Serial.print(":");
      SerialBT.print(minutes);
      SerialBT.print(":");
      if (seconds < 10) {
        Serial.print("0");
        SerialBT.print("0");
      }
      Serial.print(seconds);
      Serial.println(" (M:S)");
      SerialBT.print(seconds);
      SerialBT.println(" (M:S)");
    } else {
      int hours = timeLeft / 3600;
      int remaining = timeLeft % 3600;
      int minutes = remaining / 60;
      int seconds = remaining % 60;
      Serial.print(hours);
      Serial.print(":");
      SerialBT.print(hours);
      SerialBT.print(":");
      if (minutes < 10) {
        Serial.print("0");
        SerialBT.print("0");
      }
      Serial.print(minutes);
      Serial.print(":");
      SerialBT.print(minutes);
      SerialBT.print(":");
      if (seconds < 10) {
        Serial.print("0");
        SerialBT.print("0");
      }
      Serial.print(seconds);
      Serial.println(" (H:M:S)");
      SerialBT.print(seconds);
      SerialBT.println(" (H:M:S)");
    }
    //Step the specified number of steps and increment the run counter
    for (int i = 0; i < numSteps; i++) {
      singleStep();
    }
      runCount++;
      distancePershot = (linearDistance * numSteps);
      distanceTotal = (distancePershot * runLimit);
      distanceLeft = (distanceTotal) - (distancePershot * runCount);
      Serial.print("Motor moved!, total distance left: ");
      Serial.print(distanceLeft);
      Serial.println("mm");
      SerialBT.print("Motor moved!, total distance left: ");
      SerialBT.print(distanceLeft);
      SerialBT.println("mm");
      delay(delayTime);    
    //Keep track of how many shot runs are left and stop the motor
    if (runLimit > 0 && runCount >= runLimit) {
    digitalWrite(shutterPin, HIGH);
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(shutterPin, LOW);
    digitalWrite(ledPin, LOW);
    delay(shotdelayTime);
      running = false;
      Serial.println("Last shot taken, stopped");
      SerialBT.println("Last shot taken, stopped");
      runCount = 0;
    }
  }
  else {
    digitalWrite(enablePin, HIGH);
  }
}
void singleStep() {
        digitalWrite(stepPin, HIGH);
        digitalWrite(stepPin, LOW);
        delay(60000/motorRPM/stepsPerRevolution);
    }