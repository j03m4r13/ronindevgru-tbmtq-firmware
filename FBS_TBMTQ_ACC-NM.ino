// VER 1.00088
#include <avr/wdt.h>
#include <RotaryEncoder.h>
#include <AccelStepper.h>
#include <TMCStepper.h>

#define AUTO_SELECT 1
#define MANUAL_SELECT 2
// MANUAL OVERRIDE RANGES
#define MOR_MIN 59
#define MOR_MAX 1016

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
#define MOR_PIN A0   // Analog input for Rx axis
#define R_SWITCH 6
#define L_SWITCH 5
#define O_SWITCH 7
#define PIN_IN1 3
#define PIN_IN2 4


#define DEBOUNCE_DELAY 200 // Debounce delay for button presses

#define SMOOTHING_READS 10 // Number of reads for smoothing all axes
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075


RotaryEncoder *encoder = nullptr;




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


// DATA TRANSMISSION LIMITING VALUES
unsigned long previousAileronMillis = 0;
unsigned long previousFlapsMillis = 0; // Stores the last time the data was sent
unsigned long previousCondLevMillis = 0; // Stores the last time the data was sent
unsigned long previousThrottleMillis = 0; // Stores the last time the data was sent
unsigned long previousMorMillis = 0; // Stores the last time the data was sent
unsigned long previousMillis = 0; // Stores the last time the data was sent


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
void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}
void setup() {

  pinMode(L_SWITCH, INPUT_PULLUP);
  pinMode(R_SWITCH, INPUT_PULLUP);
  pinMode(O_SWITCH, INPUT_PULLUP);




  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
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

        simDetected = true;
      } else if (inputString == "X\n"){
        simDetected = false;
        preambleSent = false;

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

    serialEvent();
    if(stringComplete){
      if (inputString == "X\n") {
        simDetected = false;
        preambleSent = false;
        softwareReset();
      } else if (inputString == "Y\n"){ // in case arduino is in the middle of the loop(), still try to detect simconnect restarts
        Serial.println("K");       
      } else if (inputString == "Z\n"){
        isXP = false;
        simDetected = true;
      }
      inputString = "";
      stringComplete = false;
    }

    encoder->tick(); // just call tick() to check the state.
    int newDir = (int)(encoder->getDirection());
    if (newDir != dir) {
      Serial.println(String(30000+newDir));
      dir = newDir;
    }

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
      } else if (prevFuelSelectorPos == FS_OFF && fuelSelectorPos == FS_RIGHT){
        Serial.println("-12"); // resend -12 from OFF to Right position
      }
      prevFuelSelectorPos = fuelSelectorPos;
    }

  
    int mor = smoothAxisValue(MOR_PIN, 10);
    mor = constrain(mor, MOR_MIN, MOR_MAX);

    int morFinal = map(mor, MOR_MIN, MOR_MAX, 20000, 20100);
    unsigned long currentMorMillis = millis(); // Get the current time
    if (currentMorMillis - previousMorMillis >= 10) {
      previousMorMillis = currentMorMillis;
      if (prevMor != morFinal){
        Serial.println(morFinal);
        prevMor = morFinal;
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
      static int stickState = -2;
      static int prevStickState = -1;
      static int prevShiftState = 0;
      static int currentShiftState = -1;
      static int prevMor = -1;
      encoder->tick(); // just call tick() to check the state.
      int newDir = (int)(encoder->getDirection());
      if (newDir != dir) {
        Serial.println(String(15000+newDir));
        dir = newDir;
      }
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
        
        if ((millis() - lastODebounceTime) > debounceDelay) {
            if (OButton == LOW) {
              Serial.println("-9"); // OFF

              lastDirection = currDirection;
              lastRButtonState = 0; //since Right is already ON even when master is at off position
            }
            OButtonState = OButton; 
        }
        prevDir = currDirection;
      }




      

      int mor = smoothAxisValue(MOR_PIN, 10);
      mor = constrain(mor, MOR_MIN, MOR_MAX);

      int morFinal = map(mor, MOR_MIN, MOR_MAX, 20000, 20100);
      unsigned long currentMorMillis = millis(); // Get the current time
      if (currentMorMillis - previousMorMillis >= 10) {
        previousMorMillis = currentMorMillis;
        if (prevMor != morFinal){
          Serial.println(morFinal);
          prevMor = morFinal;
        }
      }

        // Check if a message has been completed
      if (stringComplete) {
        if (inputString == "X\n") {
          simDetected = false;
          preambleSent = false;
          softwareReset();
        }
        // Clear the string for the next input
        inputString = "";
        stringComplete = false;
      }
      serialEvent();


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