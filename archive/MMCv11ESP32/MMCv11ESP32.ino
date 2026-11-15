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

#include "BluetoothSerial.h"

// Bluetooth setup
BluetoothSerial SerialBT;

//Change this to fit the number of steps per revolution for your motor
const int stepsPerRevolution = 2038; 
//Pin that controls the motor direction
const int directionPin = 27;  //25 on broken protoboard
//Pin that controls the motor steps
const int stepPin = 26;
//Pin that controls enables motor power
const int enablePin = 25; //27 on broken protoboard
//Variable to store the RPM of the motor
float motorRPM = 8;
//Variable to store whether the motor is running or not
bool running = false;
//Variable for storeing the motor direction 1= cw 0 =ccw
int direction = 0;

//The number of the shutter pin
const int shutterPin = 21;  //Was 18
//The led pin
const int ledPin = 2;
//The number of the pushbutton pin to move clockwise
const int ccbuttonPin = 19; //was 34
//Variable for reading the ccpushbutton status
int ccbuttonState = 0;  
//The number of the pushbutton pin to move counterclockwise  
const int ccwbuttonPin = 18; //was 35
//Variable for reading the ccwpushbutton status
int ccwbuttonState = 0;
//The number of the pushbutton pin to manually activate shutter
const int shutterbuttonPin = 16; //Known as Trig on PCB, was 17
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

// Non-blocking delay variables and state machine
unsigned long previousMillis = 0;
bool inDelay = false;
unsigned long delayStartTime = 0;
unsigned long delayDuration = 0;

// State machine for running sequence
enum RunState {
  STATE_IDLE,
  STATE_TAKE_SHOT,
  STATE_WAIT_AFTER_SHOT,
  STATE_MOVE_MOTOR,
  STATE_WAIT_AFTER_MOTOR,
  STATE_FINAL_SHOT
};

RunState currentState = STATE_IDLE;

// Global variable for WiFi upload serial port
Stream* uploadSerialPort = nullptr;

// Bluetooth connection tracking
bool btConnected = false;
bool btWasConnected = false;

void printMenu() {
  Serial.println("Macro Motor Controller - David M. Helton 2023");
  Serial.println("Serial Bluetooth is enabled.");
  Serial.println("Commands: s - start, p - stop");
  Serial.println("m - moves between shots (currently= " + String(runLimit) + ")");
  Serial.println("y - delay time after shot (currently= " + String(shotdelayTime) + "ms)");
  Serial.println("t - delay time after moving motor (currently= " + String(delayTime) + "ms)");
  Serial.println("q - motor steps per shot (currently= " + String(numSteps) + " = " + String(linearDistance * numSteps, 4) + "mm)");
  Serial.println("d - motor direction 1=cc 0=ccw (currently= " + String(direction) + (direction == 1 ? " cw)" : " ccw)"));
  Serial.println("r - motor speed in RPM (currently= " + String(motorRPM) + ", max=15)");
  Serial.println("i - quick snap");
  Serial.println("u - upload bin file via wifi");
  Serial.println("x - reset");
  Serial.println("h - display this menu again");
}

void printBluetoothMenu() {
  SerialBT.println("Macro Motor Controller - David M. Helton 2023");
  SerialBT.println("Commands: s - start, p - stop");
  SerialBT.println("m - moves between shots (currently= " + String(runLimit) + ")");
  SerialBT.println("y - delay time after shot (currently= " + String(shotdelayTime) + "ms)");
  SerialBT.println("t - delay time after moving motor (currently= " + String(delayTime) + "ms)");
  SerialBT.println("q - motor steps per shot (currently= " + String(numSteps) + " = " + String(linearDistance * numSteps, 4) + "mm)");
  SerialBT.println("d - motor direction 1=cc 0=ccw (currently= " + String(direction) + (direction == 1 ? " cw)" : " ccw)"));
  SerialBT.println("r - motor speed in RPM (currently= " + String(motorRPM) + ", max=15)");
  SerialBT.println("i - quick snap");
  SerialBT.println("u - upload bin file via wifi");
  SerialBT.println("x - reset");
  SerialBT.println("h - display this menu again");
}

void setup() {
  //Initialize the serial ports:
  Serial.begin(115200);
  SerialBT.begin("Macro ESP32TEST"); //Bluetooth device name
  //Set up pins for camera trigger and led output
  pinMode(shutterPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  //Set pins for motor direction, step, and power enable to output
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  //Set pins as as button inputs
  pinMode(ccbuttonPin, INPUT_PULLUP);
  pinMode(ccwbuttonPin, INPUT_PULLUP);
  pinMode(shutterbuttonPin, INPUT_PULLUP);
  
  // SET INITIAL DIRECTION - INVERTED LOGIC
  digitalWrite(directionPin, direction == 1 ? LOW : HIGH);
  
  //Print the Menu
  printMenu();
  //turn off LED if left on from OTA update
  digitalWrite(ledPin, LOW);
}

void loop() {
  // Check Bluetooth connection status
  checkBluetoothConnection();
  
  // Handle all serial input through a single function
  handleSerialInput(Serial);
  handleSerialInput(SerialBT);
  
  //Read the state of the pushbutton values:
  ccbuttonState = digitalRead(ccbuttonPin);
  ccwbuttonState = digitalRead(ccwbuttonPin);
  shutterbuttonState = digitalRead(shutterbuttonPin);
  
  //Check if pushbuttons are pressed. If it is, move motor:
  if (ccbuttonState == LOW) {
    digitalWrite(enablePin, LOW);
    digitalWrite(directionPin, LOW);
    singleStep();
    digitalWrite(enablePin, HIGH);
  }
  if (ccwbuttonState == LOW) {
    digitalWrite(enablePin, LOW);
    digitalWrite(directionPin, HIGH);
    singleStep();
    digitalWrite(enablePin, HIGH);
  }
  if (shutterbuttonState == LOW) {
    digitalWrite(shutterPin, HIGH);
    digitalWrite(ledPin, HIGH);
    delay(100); // This is a short blocking delay for button press
    digitalWrite(shutterPin, LOW);
    digitalWrite(ledPin, LOW);
  }
  
  // Handle non-blocking delays and state machine
  if (inDelay) {
    if (millis() - delayStartTime >= delayDuration) {
      inDelay = false;
      // After delay completes, move to next state
      advanceState();
    } else {
      return; // Still in delay, skip the rest of the loop
    }
  }
  
  // State machine execution
  if (running && !inDelay) {
    executeState();
  } else if (!running) {
    digitalWrite(enablePin, HIGH);
    currentState = STATE_IDLE;
  }
}

void checkBluetoothConnection() {
  // Check if Bluetooth client is connected
  bool currentlyConnected = SerialBT.hasClient();
  
  // If connection state changed
  if (currentlyConnected != btConnected) {
    btConnected = currentlyConnected;
    
    if (btConnected) {
      // New connection - print menu to Bluetooth
      Serial.println("Bluetooth client connected");
      printBluetoothMenu();
    } else {
      // Connection lost
      Serial.println("Bluetooth client disconnected");
    }
  }
}

void executeState() {
  int remainingShots; // Declare variable outside switch to avoid cross-initialization errors
  
  switch (currentState) {
    case STATE_IDLE:
      // Start the sequence
      currentState = STATE_TAKE_SHOT;
      executeState(); // Immediately execute the first state
      break;
      
    case STATE_TAKE_SHOT:
      digitalWrite(shutterPin, HIGH);
      digitalWrite(ledPin, HIGH);
      delay(100); // Short blocking delay for shutter trigger
      digitalWrite(shutterPin, LOW);
      digitalWrite(ledPin, LOW);
      
      // Calculate and display shot info
      Serial.print("Shot Taken!, Shots left: ");    
      Serial.println(runLimit - runCount);
      SerialBT.print("Shot Taken!, Shots left: ");    
      SerialBT.println(runLimit - runCount);
      
      // Calculate remaining time more accurately
      remainingShots = runLimit - runCount;
      if (remainingShots > 0) {
        // Calculate ACTUAL motor time per shot including the 1ms command check delay
        float stepTime = 60000.0 / motorRPM / stepsPerRevolution; // time per step in ms
        long actualMotorTimePerShot = (stepTime + 1) * numSteps; // ADD the 1ms command check delay
        
        // Total time for ALL remaining operations:
        // For EACH remaining shot: shot delay + ACTUAL motor time + motor delay
        timeLeft = remainingShots * (shotdelayTime + actualMotorTimePerShot + delayTime);
        
        // Convert to seconds
        timeLeft = timeLeft / 1000;
        
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
      }
      
      // Move to wait after shot state
      currentState = STATE_WAIT_AFTER_SHOT;
      startDelay(shotdelayTime);
      break;
      
    case STATE_MOVE_MOTOR:
      //Step the specified number of steps
      digitalWrite(enablePin, LOW); // Enable motor
      
      // SET THE DIRECTION BASED ON THE direction VARIABLE - INVERTED LOGIC
      digitalWrite(directionPin, direction == 1 ? LOW : HIGH);
      
      for (int i = 0; i < numSteps; i++) {
        singleStep();
        
        // Check for stop command during motor movement - MORE FREQUENT CHECKS
        if (!running) break;
        
        // Also check for serial input during motor movement
        handleSerialInput(Serial);
        handleSerialInput(SerialBT);
        
        // Small delay to allow serial processing
        delay(1);
      }
      digitalWrite(enablePin, HIGH); // Disable motor
      
      // Only continue if we weren't stopped during movement
      if (running) {
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
        
        // Check if this is the final shot
        if (runLimit > 0 && runCount >= runLimit) {
          currentState = STATE_FINAL_SHOT;
          executeState(); // Immediately execute final shot
        } else {
          currentState = STATE_WAIT_AFTER_MOTOR;
          startDelay(delayTime);
        }
      } else {
        // We were stopped during motor movement, go back to idle
        currentState = STATE_IDLE;
        digitalWrite(enablePin, HIGH);
      }
      break;
      
    case STATE_FINAL_SHOT:
      digitalWrite(shutterPin, HIGH);
      digitalWrite(ledPin, HIGH);
      delay(100); // Short blocking delay for final shot
      digitalWrite(shutterPin, LOW);
      digitalWrite(ledPin, LOW);
      
      // FINAL SHOT COMPLETED - STOP IMMEDIATELY
      running = false;
      Serial.println("Last shot taken, stopped");
      SerialBT.println("Last shot taken, stopped");
      runCount = 0;
      currentState = STATE_IDLE;
      break;
  }
}

void advanceState() {
  switch (currentState) {
    case STATE_WAIT_AFTER_SHOT:
      currentState = STATE_MOVE_MOTOR;
      executeState();
      break;
      
    case STATE_WAIT_AFTER_MOTOR:
      currentState = STATE_TAKE_SHOT;
      executeState();
      break;
  }
}

void singleStep() {
  digitalWrite(stepPin, HIGH);
  digitalWrite(stepPin, LOW);
  delay(60000/motorRPM/stepsPerRevolution); // This is a short step delay
}

void startDelay(unsigned long duration) {
  inDelay = true;
  delayStartTime = millis();
  delayDuration = duration;
}

void handleSerialInput(Stream &serialPort) {
  if (serialPort.available() > 0) {
    char input = serialPort.read();
    
    // If we're in a delay and user sends stop command, cancel the delay
    if (input == 'p' && (inDelay || running)) {
      inDelay = false;
      running = false;
      runCount = 0;
      currentState = STATE_IDLE;
      digitalWrite(enablePin, HIGH); // Ensure motor is disabled
      serialPort.println("Stopped");
      return;
    }
    
    //Check input for commands
    if (input == 's') { // start command
      running = true;
      digitalWrite(enablePin, LOW);
      runCount = 0;
      currentState = STATE_IDLE; // Reset state machine
      serialPort.println("Started");
    } else if (input == 'p') { // stop command
      running = false;
      serialPort.println("Stopped");
      runCount = 0;
      currentState = STATE_IDLE;
      digitalWrite(enablePin, HIGH);
    } else if (input == 'h') { // help command
      printMenu();
      printBluetoothMenu();
    } else if (input == 'r') { // set RPM command
      motorRPM = serialPort.parseFloat();
      serialPort.print("RPM set to: ");
      serialPort.println(motorRPM);
    } else if (input == 'm') { // set shots to run limit command
      runLimit = serialPort.parseInt();
      serialPort.print("Shot limit set to: ");
      serialPort.println(runLimit);
    } else if (input == 't') { // set delay after motor time command
      delayTime = serialPort.parseInt();
      serialPort.print("Delay after motor set to: ");
      serialPort.println(delayTime);
    } else if (input == 'i') { // quick snap
      digitalWrite(shutterPin, HIGH);
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(shutterPin, LOW);
      digitalWrite(ledPin, LOW);
    } else if (input == 'y') { // set delay after shot time command
      shotdelayTime = serialPort.parseInt();
      serialPort.print("Delay after shot set to: ");
      serialPort.println(shotdelayTime);
    } else if (input == 'q') { // set number of steps command
      numSteps = serialPort.parseInt();
      serialPort.print("Number of motor steps per shot set to: ");
      serialPort.println(numSteps);
      serialPort.print("Linear distance per shot = ");
      serialPort.print(linearDistance * numSteps, 4);
      Serial.println("mm");
      serialPort.println("mm");
    } else if (input == 'd') { // set direction command
      direction = serialPort.parseInt();
      if (direction == 1) {
        digitalWrite(directionPin, LOW);  // Changed to LOW for CW
        serialPort.println("Direction set to clockwise");
      } else if (direction == 0) {
        digitalWrite(directionPin, HIGH); // Changed to HIGH for CCW
        serialPort.println("Direction set to counterclockwise");
      } else {
        serialPort.println("Invalid direction input, use 'c' for clockwise or 'cc' for counterclockwise");
      }
    } else if (input == 'x') { // reset
      serialPort.println("Restarting...");
      ESP.restart();
    } else if (input == 'u') { // connect to wifi and upload bin
      handleWifiUpload(serialPort);
    }
  }
}

void handleWifiUpload(Stream &serialPort) {
  uploadSerialPort = &serialPort; // Store reference to serial port
  
  serialPort.println("Please enter WIFI SSID: ");
  while (!serialPort.available()); // wait for input
  String input = serialPort.readStringUntil('\n');
  input.trim();
  input.toCharArray(ssid, 32); // copy the input string to the ssid char array
  serialPort.print("WIFI SSID Set to: ");
  serialPort.println(ssid);
  serialPort.println("Please enter WIFI password: ");
  while (!serialPort.available()); // wait for input
  String inputpw = serialPort.readStringUntil('\n');
  inputpw.trim();
  inputpw.toCharArray(password, 32);
  serialPort.print("WIFI password Set to: ");
  serialPort.println(password);
  
  // Connect to WiFi network
  WiFi.begin(ssid, password);
  serialPort.print("Connecting");
  
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    serialPort.print(".");
  }
  serialPort.println("");
  serialPort.print("Connected to ");
  serialPort.println(ssid);
  serialPort.print("IP address: ");
  serialPort.println(WiFi.localIP());
  
  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://macro.local
    serialPort.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  serialPort.println("Connect to http://macro.local");
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
      if (uploadSerialPort) {
        uploadSerialPort->printf("Update: %s\n", upload.filename.c_str());
      }
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        if (uploadSerialPort) {
          Update.printError(*uploadSerialPort);
        }
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        if (uploadSerialPort) {
          Update.printError(*uploadSerialPort);
        }
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        if (uploadSerialPort) {
          uploadSerialPort->printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        }
      } else {
        if (uploadSerialPort) {
          Update.printError(*uploadSerialPort);
        }
      }
    }
  });
  server.begin();
  while (true) {
    server.handleClient();
    digitalWrite(ledPin, HIGH);
    delay(1);
  }
}