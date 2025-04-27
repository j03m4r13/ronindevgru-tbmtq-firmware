// VER 1.00080
#include <avr/wdt.h>
#include <RotaryEncoder.h>
#include <AccelStepper.h>
#include <TMCStepper.h>

#define AUTO_SELECT 1
#define MANUAL_SELECT 2

#define LEFT 1
#define RIGHT 2
#define OFF 3
#define SHIFT 4

#define MOVING 1
#define STOPPED 2
#define AUTO_PILOT 1
#define MANUAL_MODE 2
#define MANUAL_NO_SWITCH_MODE 0
#define TBM_TRIM_SWITCH_MODE 3

#define STALL_VALUE     100 // [0..255]


#define MSFS 1
#define XP 2
#define FS_LEFT 1
#define FS_RIGHT 2
#define FS_OFF 3

#define R_SWITCH 10
#define L_SWITCH 14
#define O_SWITCH 9
#define MICRO_SWITCH 2
#define SHIFT_SWITCH 3
#define EN_PIN           6 // Enable
#define DIR_PIN          7 // Direction
#define STEP_PIN         8 // Step
#define SW_RX            4 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            5 // TMC2208/TMC2224 SoftwareSerial transmit pin

#define DEBOUNCE_DELAY 200 // Debounce delay for button presses

#define SMOOTHING_READS 10 // Number of reads for smoothing all axes
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075


AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);



int simType = 0;

unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
bool isXP = false;
bool rebooted = true;
bool simDetected = false;
String inputString = "";  // String to hold incoming data
bool stringComplete = false;  // Flag to indicate message completion
bool preambleSent = false;

int LButtonState = 0;
int lastLButtonState = 0;
unsigned long lastLDebounceTime = 0;  // the last time the output pin was toggled
int RButtonState = 0;
int lastRButtonState = 0;
unsigned long lastRDebounceTime = 0;  // the last time the output pin was toggled

int OButtonState = 0;
int lastOButtonState = 0;
unsigned long lastODebounceTime = 0;  // the last time the output pin was toggled

int MButtonState = 0;
int lastMButtonState = 0;
unsigned long lastMDebounceTime = 0;  // the last time the output pin was toggled

int  SHIFTButtonState = 0;
int lastSHIFTButtonState = 0;
unsigned long lastSHIFTDebounceTime = 0;  // the last time the output pin was toggled



bool shaft = false;
double currSelectorPosition = 0.0;
double prevSelectorPosition = -1.0;
float midPoint = 0.07;
long fullRevolutionSteps = 3200;
long midStepper = fullRevolutionSteps / 2;
float pointer = 0.0;
String planeTrimData;

int state = 0;
bool manualSwitchOn = false;
bool hasSentAPOff = false;
int currSelectorDirection = 0.0;
int prevSelectorDirection = 0.0;
bool hasHomed = false;
bool masterTriggered = false;
int lastDirection = 0.0;
int currDirection = 0.0;
int autoSelectMode = MANUAL_SELECT;

long mapInputToSteps(double fuelSelector){
  if (fuelSelector >= 2.0) fuelSelector = 2.0;
  if (fuelSelector <= 0.0) fuelSelector = 0.0;
  fuelSelector = fuelSelector * (.7 / 2.0); // for remapping in case of falling short for knob travel
  long steps = fuelSelector * fullRevolutionSteps;
  return steps;
}

int smoothAxisValue(int pin, int smoothingReads) {
  int totalValue = 0;
  for (int i = 0; i < smoothingReads; i++) {
    totalValue += analogRead(pin);
  }
  return totalValue / smoothingReads;  // Return the averaged value
}
void softwareReset() {
  wdt_enable(WDTO_15MS); // Enable watchdog timer for a 15ms reset
  while (1) {}           // Wait for the watchdog to trigger the reset
}


void homeStepper(){
        
        int LButton;
        int RButton;
        int OButton;
        long stepsTaken = 0;
        long currSteps = 0;
        bool rightFound = false;
        int counter = 0;
        stepper.setCurrentPosition(0.0);
        stepper.moveTo(8000); // rotate full to search
        // Keep running the stepper
        while (stepper.distanceToGo() != 0) {
          

          RButton = digitalRead(R_SWITCH);

              if (RButton == LOW) { // found RIGHT position

                stepper.stop();
                stepper.setCurrentPosition(mapInputToSteps(1.0)*-1);
                return;
              }


          stepper.run();  // Run the motor to the target position

        }
        stepper.setCurrentPosition(mapInputToSteps(1.0)*-1); // JUST SET TO R FOR NOW
}

void setup() {

  pinMode(L_SWITCH, INPUT_PULLUP);
  pinMode(R_SWITCH, INPUT_PULLUP);
  pinMode(O_SWITCH, INPUT_PULLUP);
  pinMode(MICRO_SWITCH, INPUT);
  pinMode(SHIFT_SWITCH, INPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  driver.begin();
  driver.toff(4);          // Enable driver (off_time)
  driver.rms_current(800); // Set motor current (mA)
  driver.microsteps(16);   // Set microstepping (16 microsteps per step)
  stepper.setMaxSpeed(800);
  stepper.setAcceleration(5000);



  LButtonState = 0;
  lastLButtonState = 0;
  lastLDebounceTime = 0;  // the last time the output pin was toggled
  RButtonState = 0;
  lastRButtonState = 0;
  lastRDebounceTime = 0;  // the last time the output pin was toggled
  OButtonState = 0;
  lastOButtonState = 0;
  lastODebounceTime = 0;  // the last time the output pin was toggled
  MButtonState = -1;
  lastMButtonState = -1;
  lastMDebounceTime = 0;  // the last time the output pin was toggled
  SHIFTButtonState = 0;
  lastSHIFTButtonState = 0;
  lastSHIFTDebounceTime = 0;  // the last time the output pin was toggled

  autoSelectMode = MANUAL_SELECT;
  currSelectorPosition = 0.0;
  prevSelectorPosition = -1.0;
  hasHomed = false;
  masterTriggered = false;
 
  lastDirection = 0.0;
  currDirection = 0.0;
  simDetected = false;
  preambleSent = false;
  isXP = false;

  Serial.begin(250000);
}

void sendX(int value, int times){
  for(int i = 0; i < times; i++){
    Serial.println(value);
  }
}
bool isFloat(String str) {
    if (str.length() == 0) return false;
    int dotCount = 0;
    for (int i = 0; i < str.length(); i++) {
        if (i == 0 && (str[i] == '+' || str[i] == '-')) {
            continue; // Allow leading '+' or '-'
        }
        if (str[i] == '.') {
            dotCount++;
            if (dotCount > 1) return false; // Only one dot allowed
        } else if (!isDigit(str[i])) {
            return false; // Non-numeric character found
        }
    }
    return true;
}

void loop() {
  // DETECT SIMULATOR
  while(!simDetected){
    serialEvent();
    if (stringComplete) {
      if (inputString == "P\n"){
        simType = XP;
        preambleSent = true;
        Serial.println("K");      
      } else if (inputString == "Y\n"){
        simType = MSFS;
        preambleSent = true;

        Serial.println("K");       
      } else if (inputString == "Z\n"){
        digitalWrite(EN_PIN, HIGH);
        simDetected = true;
      } else if (inputString == "X\n"){
        simDetected = false;
        preambleSent = false;
        digitalWrite(EN_PIN, HIGH);
        softwareReset();
      }
      inputString = "";
      stringComplete = false;
    }    
    delay(40);
  }

  if (simType == MSFS && simDetected){
    unsigned long currentTime = millis(); 
    unsigned long analogReadInterval = 300; // Default slow analog read interval
    static bool button2PrevState, button3PrevState;
    static bool loHiSequence = false, hiLoSequence;
    static unsigned long loHiTimeElapsed = 0;
    static bool fromCutOfftoLoIdleSeq = false;
    static int fuelSelectorPos = -1;
    static int prevFuelSelectorPos = -2;
    static int prevThrottle = -1;
    static int prevRevThrottle = -1;
    static int prevMor = -1;
    static int prevCL = -1;
    static bool stopFSSend = false;
    static int sendCount = 0;
    static int dir = 0;
    static int manual_dir = 0;
    static bool masterPrevStatus = false;
    static int shiftState = -1;
    static int prevShiftState = -2;
    int LButton;
    int RButton;
    int OButton;
    int MButton;
    int SHIFTButton;

    int currManualDir = 0;
    MButton = digitalRead(MICRO_SWITCH);
    if (lastMButtonState != MButton){
      lastMDebounceTime = millis();
    }
    if ((millis() - lastMDebounceTime) > debounceDelay) {
      if(MButtonState != MButton) {
        if (MButton == LOW) { // microswitch triggered
          masterTriggered = true;
          if(autoSelectMode == AUTO_SELECT) {
            digitalWrite(EN_PIN, LOW); // turn on stepper when in auto select mode when activating micro switch
          }
          Serial.println("-12"); // At this point R is also active so just send 1 to plugin

        } else {
          masterTriggered = false;
          digitalWrite(EN_PIN, HIGH); // disable stepper when master is disengaged
        }
        MButtonState = MButton;
        masterPrevStatus = masterTriggered;
      }

    }
    serialEvent();
    if(stringComplete){
      if (inputString == "X\n") {
        simDetected = false;
        preambleSent = false;
        digitalWrite(EN_PIN, HIGH);
        softwareReset();
      } else if (inputString == "Y\n"){ // in case arduino is in the middle of the loop(), still try to detect simconnect restarts
        Serial.println("K");       
      } else if (inputString == "Z\n"){
        isXP = false;
        simDetected = true;
      } else if (inputString == "R\n"){
        hasSentAPOff = false;
        hasHomed = false;
        autoSelectMode = AUTO_SELECT;
        if (masterTriggered) {
          digitalWrite(EN_PIN, LOW);
          homeStepper();
          currSelectorPosition = 1.0; //RESET TO right;
          prevSelectorPosition = 1.0;
        }


      } else if (inputString == "T\n"){
        autoSelectMode = MANUAL_SELECT;
        digitalWrite(EN_PIN, HIGH);
      } else {
        inputString.trim();
        currSelectorPosition = inputString.toDouble();
      }
      inputString = "";
      stringComplete = false;

    }

    if (autoSelectMode == MANUAL_SELECT){
      int fuelLeft = digitalRead(L_SWITCH);
      int fuelRight = digitalRead(R_SWITCH);
      int fuelOff = digitalRead(O_SWITCH);
      unsigned long currentMillis = millis(); // Get the current time
      // Update the fuel selector position based on input
      if (fuelLeft == LOW) {
        fuelSelectorPos = FS_LEFT;
      } else if (fuelRight == LOW) {
        fuelSelectorPos = FS_RIGHT;
      } else if (fuelOff == LOW) {
        fuelSelectorPos = FS_OFF;
      }

      // Check if it's time to send data


      // Send only if the value has changed
      if (prevFuelSelectorPos != fuelSelectorPos){
        if (fuelSelectorPos == FS_LEFT) {
          Serial.println("-11");
        } else if (fuelSelectorPos == FS_RIGHT) {
          Serial.println("-12");
        } else if (fuelSelectorPos == FS_OFF) {
          Serial.println("-13");
        }
        prevFuelSelectorPos = fuelSelectorPos;
      }

      
    }

    if(autoSelectMode == AUTO_SELECT && masterTriggered){
      SHIFTButton = digitalRead(SHIFT_SWITCH);
      if (lastSHIFTButtonState != SHIFTButton){
        lastSHIFTDebounceTime = millis();
        currDirection = SHIFT;
        shiftState = 1;
      }

      if ((millis() - lastSHIFTDebounceTime) > debounceDelay) {
        if (SHIFTButton == LOW) {
          shiftState = 1;
        } else {
          shiftState = 0;
        }

        SHIFTButtonState = SHIFTButton;
      }

      if (prevShiftState != shiftState){
        if (shiftState == 1){
          Serial.println("-14");
        } else {
          Serial.println("-15");
        }
        prevShiftState = shiftState;
      }


      if (currSelectorPosition != prevSelectorPosition) {

          if (currSelectorPosition  <= 2.0 && currSelectorPosition >= 1.0){
            long steps = mapInputToSteps(currSelectorPosition);
            stepper.moveTo(steps*-1);
            // Keep running the stepper
            while (stepper.distanceToGo() != 0) {
              stepper.run();  // Run the motor to the target position
              serialEvent();
              if (stringComplete){
                stepper.stop();  // Decelerates to stop
                break;           // Exit the while loop after stopping
              }
            }
          } 
        prevSelectorPosition = currSelectorPosition;
      }
    }


    lastLButtonState = LButton;
    lastRButtonState = RButton;
    lastOButtonState = OButton;
    lastMButtonState = MButton;
    lastSHIFTButtonState = SHIFTButton;
    //delay(20);  // Small delay to avoid overwhelming USB communication
  } else if (simType == XP && simDetected) {  // XP ****************************************
      static int dir = 0;
      static int manual_dir = 0;
      static bool masterPrevStatus = false;
      int LButton;
      int RButton;
      int OButton;
      int MButton;
      int SHIFTButton;
      static int prevDir = -2;
      int currManualDir = 0;
      static int fsPrevPos = -3;
      static int fsCurrPos = -1;
      MButton = digitalRead(MICRO_SWITCH);
      if (lastMButtonState != MButton){
        lastMDebounceTime = millis();
      }
      if ((millis() - lastMDebounceTime) > debounceDelay) {
        if(MButtonState != MButton) {
          if (MButton == LOW) { // microswitch triggered
            masterTriggered = true;
            if(autoSelectMode == AUTO_SELECT) {
              digitalWrite(EN_PIN, LOW); // turn on stepper when in auto select mode when activating micro switch
            }
            Serial.println("1"); // At this point R is also active so just send 1 to plugin

          } else {
            masterTriggered = false;
            digitalWrite(EN_PIN, HIGH); // disable stepper when master is disengaged
          }
          MButtonState = MButton;
          masterPrevStatus = masterTriggered;
        }

      }

      if (autoSelectMode == MANUAL_SELECT && masterTriggered ){
        LButton = digitalRead(L_SWITCH);
        RButton = digitalRead(R_SWITCH);
        OButton = digitalRead(O_SWITCH);



        if (lastLButtonState != LButton){
          lastLDebounceTime = millis();
          currDirection = LEFT;
        }
        if (lastRButtonState != RButton){
          lastRDebounceTime = millis();
          currDirection = RIGHT;
        }
        if (lastOButtonState != OButton){
          lastODebounceTime = millis();
          currDirection = OFF;
        }

        if (currDirection != lastDirection){
          if ((millis() - lastLDebounceTime) > debounceDelay) {
              if (LButton == LOW) {
                
                lastDirection = currDirection;
                fsCurrPos = LEFT;
                Serial.println("-8");
              }
              LButtonState = LButton;
          }
          if ((millis() - lastRDebounceTime) > debounceDelay) {
              if (RButton == LOW) {
    
                lastDirection = currDirection;
                Serial.println("-7");

              }
              RButtonState = RButton;
          }
          prevDir = currDirection;
        }



      } else if (autoSelectMode == MANUAL_SELECT && !masterTriggered) {
        OButton = digitalRead(O_SWITCH);
        if (lastOButtonState != OButton && currDirection != lastDirection){
          lastODebounceTime = millis();
          currDirection = OFF;

        }
        if ((millis() - lastODebounceTime) > debounceDelay) {
            if (OButton == LOW) {
              Serial.println("-9"); // OFF
              digitalWrite(EN_PIN, HIGH);
              lastDirection = currDirection;
              lastRButtonState = 0; //since Right is already ON even when master is at off position
            }
            OButtonState = OButton; 
        }
      }

      if(autoSelectMode == AUTO_SELECT && masterTriggered){
        SHIFTButton = digitalRead(SHIFT_SWITCH);
        if (lastSHIFTButtonState != SHIFTButton){
          lastSHIFTDebounceTime = millis();
          currDirection = SHIFT;
        }

        if ((millis() - lastSHIFTDebounceTime) > debounceDelay) {
          if (SHIFTButton == LOW) {
            
            lastDirection = currDirection;
            //Serial.println("-6"); // SHIFT

          } else {
            Serial.println("-5"); // SHIFT
          }
          SHIFTButtonState = SHIFTButton;
        }
      }

        // Check if a message has been completed
      if (stringComplete) {
        if (inputString == "X\n") {
          simDetected = false;
          preambleSent = false;
          digitalWrite(EN_PIN, HIGH);
          softwareReset();
        } else if (inputString == "R\n"){
          hasSentAPOff = false;
          hasHomed = false;
          autoSelectMode = AUTO_SELECT;
          if (masterTriggered) {
            digitalWrite(EN_PIN, LOW);
            homeStepper();
            currSelectorPosition = 1.0; //RESET TO right;
            prevSelectorPosition = 1.0;
          }

          // long steps = mapInputToSteps(currSelectorPosition);
          // stepper.setCurrentPosition(steps); // no physical movements, just setting the position internally
        } else if (inputString == "T\n"){
          autoSelectMode = MANUAL_SELECT;
          digitalWrite(EN_PIN, HIGH);
        } else {
          inputString.trim();
          if(isFloat(inputString)){
            currSelectorPosition = inputString.toDouble();
          }
        }
        // Clear the string for the next input
        inputString = "";
        stringComplete = false;
      }
      serialEvent();
      if (currSelectorPosition != prevSelectorPosition) {

          if (autoSelectMode == AUTO_SELECT && currSelectorPosition  <= 2.0 && currSelectorPosition >= 1.0){
            long steps = mapInputToSteps(currSelectorPosition);
            stepper.moveTo(steps*-1);
            // Keep running the stepper
            while (stepper.distanceToGo() != 0) {
              stepper.run();  // Run the motor to the target position
              serialEvent();
              if (stringComplete){
                stepper.stop();  // Decelerates to stop
                break;           // Exit the while loop after stopping
              }
            }
          } else if (autoSelectMode == MANUAL_SELECT && currSelectorPosition  <= 2.0 && currSelectorPosition >= 1.0) {
            long steps = mapInputToSteps(currSelectorPosition);
            stepper.setCurrentPosition(steps*-1); // no physical movements, just setting the position internally
          }
        prevSelectorPosition = currSelectorPosition;
      }

      lastLButtonState = LButton;
      lastRButtonState = RButton;
      lastOButtonState = OButton;
      lastMButtonState = MButton;
      lastSHIFTButtonState = SHIFTButton;
  }

}


void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}