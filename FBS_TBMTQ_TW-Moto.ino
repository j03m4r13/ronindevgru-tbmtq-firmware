// VER 1.00080
#include <avr/wdt.h>
#include <RotaryEncoder.h>
#include <AccelStepper.h>
#include <TMCStepper.h>


#define MOVING 1
#define STOPPED 2
#define AUTO_PILOT 1
#define MANUAL_MODE 2
#define MANUAL_NO_SWITCH_MODE 0
#define TBM_TRIM_SWITCH_MODE 3

#define STALL_VALUE     100 // [0..255]

#define DOWN_SWITCH 14
#define UP_SWITCH 10
// MANUAL OVERRIDE RANGES
#define MOR_MIN 59
#define MOR_MAX 1016

#define MSFS 1
#define XP 2
#define FS_LEFT 1
#define FS_RIGHT 2
#define FS_OFF 3

#define EN_PIN           6 // Enable
#define DIR_PIN          7 // Direction
#define STEP_PIN         8 // Step
#define SW_RX            4 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            5 // TMC2208/TMC2224 SoftwareSerial transmit pin



#define PIN_IN1 3
#define PIN_IN2 2

#define MOR_PIN A3   // Analog input for Rx axis
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

RotaryEncoder *encoder = nullptr;
bool sequenceActive = false;         // Track if the sequence is running
bool sequenceHiLoActive = false;     // Track for Hi to Lo sequence
unsigned long sequenceStartTime = 0, sequenceHiLoStartTime = 0;; // Timer for the sequence
int sequenceStep = 0;                // Track the current step of the sequence
int sequenceHiLoStep = 0; 
bool idleLoTriggered = false;        // Track if Idle Low was triggered
unsigned long lastButton2PressTime = 0; // Track last Idle Low button press
unsigned long lastAnalogReadTime = 0;   // Track the last time we read A0, A1, A2, A3

bool button3State = false;  // Initial state of Button 3 (HI_IDLE) set to true
bool button4State = false; // Initial state of Button 4 (FLIGHT_IDLE)
bool flightIdleActive = false; // Keep track of Flight Idle state to maintain smooth reads
bool frmLtoR = false;
bool frmRtoL = false;
bool frmCutOff = false;
bool frmHIIdle = false;
bool reversing = false;

int simType = 0;
int revButtonState = 0;
int lastRevButtonState = 0;
unsigned long lastRevDebounceTime = 0;  // the last time the output pin was toggled

int aileronLeftButtonState = 0;
int lastAileronLeftButtonState = 0;
unsigned long lastAileronLeftDebounceTime = 0;  // the last time the output pin was toggled

int aileronRightButtonState = 0;
int lastAileronRightButtonState = 0;
unsigned long lastAileronRightDebounceTime = 0;  // the last time the output pin was toggled

unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
bool isXP = false;
bool rebooted = true;
bool simDetected = false;
String inputString = "";  // String to hold incoming data
bool stringComplete = false;  // Flag to indicate message completion
bool preambleSent = false;


// DATA TRANSMISSION LIMITING VALUES
unsigned long previousAileronMillis = 0;
unsigned long previousFlapsMillis = 0; // Stores the last time the data was sent
unsigned long previousCondLevMillis = 0; // Stores the last time the data was sent
unsigned long previousThrottleMillis = 0; // Stores the last time the data was sent
unsigned long previousMorMillis = 0; // Stores the last time the data was sent
unsigned long previousMillis = 0; // Stores the last time the data was sent
const unsigned long interval = 20; // Interval in milliseconds (10 ms)
long fullRevolutionSteps = 7200;
long midStepper = fullRevolutionSteps / 2;
int state = 0;
int autoPilotMode = MANUAL_MODE;
bool manualSwitchOn = false;
bool hasSentAPOff = false;
float currTrimF = -99.0;
float prevTrimF = 0.0; 
long lastEncoderPos = -10;
long encoderPos;
unsigned long lastMoveTime = 0;
bool isHoldingTrim = false;

long stepperCommandPos = 0;  // Commanded position

bool isUserMovingTrim = false;

int downButtonState = 0;
int lastDownButtonState = 0;
unsigned long lastDownDebounceTime = 0;  // the last time the output pin was toggled
int upButtonState = 0;
int lastUpButtonState = 0;
unsigned long lastUpDebounceTime = 0;  // the last time the output pin was toggled


// **** TW VARS ******


bool shaft = false;

float midPoint = 0.07;

float pointer = 0.0;
String planeTrimData;


//***********************

long mapInputToSteps(float trimValue){
  if (trimValue >= 1.0) trimValue = 1.0;
  if (trimValue <= -1.0) trimValue = -1.0;
  long steps = trimValue * fullRevolutionSteps * 0.7;
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

void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}




void setup() {




  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(DOWN_SWITCH, INPUT);
  digitalWrite(EN_PIN, HIGH);
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
  driver.begin();
  driver.toff(4);          // Enable driver (off_time)
  driver.rms_current(800); // Set motor current (mA)
  driver.microsteps(16);   // Set microstepping (16 microsteps per step)
  // driver.en_spreadCycle(false);
  // driver.pwm_autoscale(true);
  stepper.setMaxSpeed(10000);
  stepper.setAcceleration(8000);
  revButtonState = 0;
  lastRevButtonState = 0;
  lastRevDebounceTime = 0;  // the last time the output pin was toggled
  aileronLeftButtonState = 0;
  lastAileronLeftButtonState = 0;
  lastAileronLeftDebounceTime = 0;

  aileronRightButtonState = 0;
  lastAileronRightButtonState = 0;
  lastAileronRightDebounceTime = 0;
  simDetected = false;
  preambleSent = false;
  isXP = false;
  encoder->setPosition(0);
  Serial.begin(250000);
}

bool isTrimWheelHeld() {
    encoder->tick();  // Update encoder state
    encoderPos = encoder->getPosition();

    if (encoderPos == lastEncoderPos) {
        if (millis() - lastMoveTime > 300) {  // Held for 300ms
            return true;
        }
    } else {
        lastMoveTime = millis();
    }

    lastEncoderPos = encoderPos;
    return false;
}

bool isUserMovingTrimWheel() {
    encoder->tick();
    encoderPos = encoder->getPosition() * 40;
    //Serial.println(encoderPos);
    //Serial.println(stepperCommandPos);
    // If encoder position changes significantly while stepper is not commanding movement
    //Serial.println(encoderPos);
    int diff = abs(encoderPos - stepperCommandPos);
    //Serial.println(diff);
   // Serial.println(abs(encoderPos - stepperCommandPos));
    if (diff > 500) {  // Adjust threshold if needed
        return true;
    }
    return false;
}

void sendX(int value, int times){
  for(int i = 0; i < times; i++){
    Serial.println(value);
  }
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
        digitalWrite(EN_PIN, LOW);
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
    static int prevFuelSelectorPos = -1;
    static int prevThrottle = -1;
    static int prevRevThrottle = -1;
    static int prevMor = -1;
    static int prevCL = -1;
    static bool stopFSSend = false;
    static int sendCount = 0;
    static int dir = 0;
    int downButton;
    int upButton;
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
      } else if (inputString == "M\n"){
          digitalWrite(EN_PIN, HIGH);
          autoPilotMode = MANUAL_MODE;

      } else if (inputString == "A\n"){

          long steps = mapInputToSteps(currTrimF);
          stepper.setCurrentPosition(steps * 2 * -1); // no physical movements, just setting the position internally      

          digitalWrite(EN_PIN, LOW);
          autoPilotMode = AUTO_PILOT;
          

      } else {
          currTrimF = inputString.toFloat(); 

      }
      inputString = "";
      stringComplete = false;
    }


    if (autoPilotMode == MANUAL_MODE){
      encoder->tick(); // just call tick() to check the state.
      int newDir = (int)(encoder->getDirection());
      if (newDir != dir) {
        Serial.println(String(30000+newDir));
        dir = newDir;
      }
    }

    downButton = digitalRead(DOWN_SWITCH);
    upButton = digitalRead(UP_SWITCH);

    if (lastDownButtonState != downButton){
      lastDownDebounceTime = millis();
    }

    if (lastUpButtonState != upButton){
      lastUpDebounceTime = millis();
    }
    if ((millis() - lastDownDebounceTime) > debounceDelay && upButton != LOW &&  currTrimF > -1.0) {
        if (downButton == LOW) {
          manualSwitchOn = true;
          if (autoPilotMode == MANUAL_MODE) {
            digitalWrite(EN_PIN, LOW);
            stepper.setSpeed(1800);
            stepper.runSpeed();
            Serial.println("29999");
          }

        }
        downButtonState = downButton;
    }
    if ((millis() - lastUpDebounceTime) > debounceDelay && downButton != LOW && currTrimF < 1.0) {
        if (upButton == LOW) {
          
          manualSwitchOn = true;
          if (autoPilotMode == MANUAL_MODE) {
            digitalWrite(EN_PIN, LOW);
            stepper.setSpeed(-1800);
            stepper.runSpeed();
            Serial.println("30001");
          }
          

        }
        upButtonState = upButton;
    } 
  

  
    if (currTrimF != -99.0 && autoPilotMode == AUTO_PILOT){

      long steps = mapInputToSteps(currTrimF);
      stepper.moveTo(steps * 2 * -1);
      // Keep running the stepper
      while (stepper.distanceToGo() != 0) {

        //stepperCommandPos++;
        stepper.run();  // Run the motor to the target position
        // if (isUserMovingTrimWheel()) {
        //   if (!isUserMovingTrim) {
        //       //Serial.println("Trim Wheel Held - Reducing Torque or Stopping!");
        //       Serial.println("-999");
        //       stepper.stop();  // Decelerates to stop
        //       digitalWrite(EN_PIN, HIGH);  // Disable stepper
        //       autoPilotMode = MANUAL_MODE;
        //       isUserMovingTrim = true;
        //       break;
              
        //   }
        // }

        serialEvent();
        if (stringComplete){
          stepper.stop();  // Decelerates to stop
          break;           // Exit the while loop after stopping
        }
      }
    }


    //delay(20);  // Small delay to avoid overwhelming USB communication
  } else if (simType == XP && simDetected) {
 
  static int dir = 0;
  static int manual_dir = 0;
  int downButton;
  int upButton;
  
  if (autoPilotMode == MANUAL_MODE || autoPilotMode == MANUAL_NO_SWITCH_MODE) { 
    
    if (!manualSwitchOn) {
      encoder->tick(); // just call tick() to check the state.
      int newDir = (int)(encoder->getDirection());
      if (newDir != dir) {
        Serial.println(String(15000+newDir));
        dir = newDir;
      }
    }
  }

  int currManualDir = 0;
  if (autoPilotMode == MANUAL_MODE || autoPilotMode == AUTO_PILOT){
    downButton = digitalRead(DOWN_SWITCH);
    upButton = digitalRead(UP_SWITCH);

    if (lastDownButtonState != downButton){
      lastDownDebounceTime = millis();
    }

    if (lastUpButtonState != upButton){
      lastUpDebounceTime = millis();
    }
    if ((millis() - lastDownDebounceTime) > debounceDelay && upButton != LOW &&  currTrimF > -1.0) {
        if (downButton == LOW) {
          manualSwitchOn = true;
          if (autoPilotMode == MANUAL_MODE) {
            digitalWrite(EN_PIN, LOW);
            stepper.setSpeed(1800);
            stepper.runSpeed();
            Serial.println("-1");
          } else {
            //if(!hasSentAPOff){
              Serial.println("2");
              hasSentAPOff = true;
           // }
          }


        } else {
          manualSwitchOn = false;
          currManualDir = 0;
          if (autoPilotMode == MANUAL_MODE)
            digitalWrite(EN_PIN, HIGH);

        }
        downButtonState = downButton;
    }
    if ((millis() - lastUpDebounceTime) > debounceDelay && downButton != LOW && currTrimF < 1.0) {
        if (upButton == LOW) {
          
          manualSwitchOn = true;
          if (autoPilotMode == MANUAL_MODE) {
            digitalWrite(EN_PIN, LOW);
            stepper.setSpeed(-1800);
            stepper.runSpeed();
            Serial.println("1");
          } else {
            //if(!hasSentAPOff){
              Serial.println("2");  
              hasSentAPOff = true;
            //}
          }
          

        } else {
          manualSwitchOn = false;
          currManualDir = 0;
          if (autoPilotMode == MANUAL_MODE)
            digitalWrite(EN_PIN, HIGH);

        }
        upButtonState = upButton;
    } 
  }


    // Check if a message has been completed
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
      digitalWrite(EN_PIN, LOW);
      simDetected = true;
    } else if (inputString == "X\n"){
      simDetected = false;
      preambleSent = false;
      digitalWrite(EN_PIN, HIGH);
      softwareReset();
    } else if (inputString == "A\n"){
      hasSentAPOff = false;
      autoPilotMode = AUTO_PILOT;
      digitalWrite(EN_PIN, LOW);
      long steps = mapInputToSteps(currTrimF);
      stepper.setCurrentPosition(steps * 2 * -1); // no physical movements, just setting the position internally      
    } else if (inputString == "N\n"){
      autoPilotMode = MANUAL_NO_SWITCH_MODE;

      digitalWrite(EN_PIN, HIGH);
    } else if (inputString == "M\n"){
      autoPilotMode = MANUAL_MODE;

      digitalWrite(EN_PIN, HIGH);
    } else if (inputString == "O\n"){
      autoPilotMode = TBM_TRIM_SWITCH_MODE;

      digitalWrite(EN_PIN, LOW);
      long steps = mapInputToSteps(currTrimF);
      stepper.setCurrentPosition(steps * 2 * -1); // no physical movements, just setting the position internally   
    } else {
      currTrimF = inputString.toFloat();  
      
    }
    

    // Clear the string for the next input
    inputString = "";
    stringComplete = false;
  }
  
  serialEvent();

  if (currTrimF != prevTrimF) {
      

      if (autoPilotMode == AUTO_PILOT || autoPilotMode == TBM_TRIM_SWITCH_MODE){
        long steps = mapInputToSteps(currTrimF);
        stepper.moveTo(steps * 2 * -1);
        // Keep running the stepper
        while (stepper.distanceToGo() != 0) {
          stepper.run();  // Run the motor to the target position
          serialEvent();
          if (stringComplete){
            stepper.stop();  // Decelerates to stop
            break;           // Exit the while loop after stopping
          }
        }
      } else if (autoPilotMode == MANUAL_MODE || autoPilotMode == MANUAL_NO_SWITCH_MODE) {
        long steps = mapInputToSteps(currTrimF);
        stepper.setCurrentPosition(steps * 2 * -1); // no physical movements, just setting the position internally
      }

    prevTrimF = currTrimF;
      
  }
  
  lastDownButtonState = downButton;
  lastUpButtonState = upButton;

    //delay(40);  // Small delay to avoid overwhelming USB communication
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