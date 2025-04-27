#include <avr/wdt.h>

// THROTTLE RANGES
#define THROTTLE_MIN_A 0
#define THROTTLE_MIN_B 510

#define THROTTLE_REV_A 802
#define THROTTLE_REV_B 569

// FLAPS RANGES
#define FLAPS_0_MIN 744
#define FLAPS_0_MAX 784

#define FLAPS_1_MIN 860 //adjust TO postion -8
#define FLAPS_1_MAX 920

#define FLAPS_2_MAX 960 // adjust LDG position -9

// MANUAL OVERRIDE RANGES
#define MOR_MIN 59
#define MOR_MAX 1016

#define MSFS 1
#define XP 2
#define FS_LEFT 1
#define FS_RIGHT 2
#define FS_OFF 3

#define AIL_R 1
#define AIL_L 2
#define AIL_N 3

#define CUTOFF 1
#define LO_IDLE 2
#define HI_IDLE 3
#define FL_IDLE 4
#define RV_THR 5

#define PIN_FUEL_CUTOFF 2  // Button 1: Cutoff
#define PIN_LO_IDLE 3      // Button 2: Idle Low
#define PIN_HI_IDLE 4      // Button 3: HI Idle
#define PIN_FLIGHT_IDLE 5  // Button 4: Flight Idle 

#define PIN_TOGA 7         // Button 6: TOGA
#define PIN_FUEL_SELECTOR_LEFT 8    // Button 7: Fuel Selector Left
#define PIN_FUEL_SELECTOR_RIGHT 9   // Button 8: Fuel Selector Right
#define PIN_FUEL_SELECTOR_OFF 10    // Button 9: Fuel Selector Off
#define PIN_AILERON_TRIM_LEFT 14    // Button 10: Aileron Trim Left
#define PIN_AILERON_TRIM_RIGHT 15   // Button 11: Aileron Trim Right

#define THROTTLE_PIN A0    // Analog input for X axis
#define FLAPS_PIN A1    // Analog input for Y axis
#define ANALOG_PIN_Z A2    // Analog input for Z axis
#define MOR_PIN A3   // Analog input for Rx axis
#define DEBOUNCE_DELAY 200 // Debounce delay for button presses

#define SMOOTHING_READS 10 // Number of reads for smoothing all axes


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
void setup() {
  pinMode(PIN_FUEL_CUTOFF, INPUT_PULLUP); // Cutoff button uses internal pull-up resistor
  pinMode(PIN_LO_IDLE, INPUT_PULLUP);     // Idle Low button uses internal pull-up resistor
  pinMode(PIN_HI_IDLE, INPUT_PULLUP);     // HI Idle (rocker switch)
  pinMode(PIN_FLIGHT_IDLE, INPUT_PULLUP); // Flight Idle (rocker switch)

  pinMode(PIN_TOGA, INPUT_PULLUP);        // TOGA
  pinMode(PIN_FUEL_SELECTOR_LEFT, INPUT_PULLUP);   // Fuel Selector Left
  pinMode(PIN_FUEL_SELECTOR_RIGHT, INPUT_PULLUP);  // Fuel Selector Right
  pinMode(PIN_FUEL_SELECTOR_OFF, INPUT_PULLUP);    // Fuel Selector Off
  pinMode(PIN_AILERON_TRIM_LEFT, INPUT);    // Aileron Trim Left
  pinMode(PIN_AILERON_TRIM_RIGHT, INPUT);   // Aileron Trim Right
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
  inputString = "";
  stringComplete = false;
  Serial.begin(250000);
}

void runMSFSLoop(){

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
    static int prevFuelSelectorPos = -1;
    static int prevThrottle = -1;
    static int prevRevThrottle = -1;
    static int prevMor = -1;
    static int prevCL = -1;
    static bool stopFSSend = false;
    static int sendCount = 0;
    int revButton;
    int cutOff = digitalRead(PIN_FUEL_CUTOFF);
    int highIdle = digitalRead(PIN_HI_IDLE);
    int flightIdle = digitalRead(PIN_FLIGHT_IDLE);
    int loIdle = digitalRead(PIN_LO_IDLE);
    static int stickState = -2;
    static int prevStickState = -1;

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

    if (cutOff == LOW){
      stickState = CUTOFF;
      
    }
    if (flightIdle == LOW){
      stickState = FL_IDLE;
      
    }

    if (highIdle == LOW){
      stickState = HI_IDLE;
      
    }
    if (loIdle == LOW){
      stickState = LO_IDLE;
      
    }


    if (prevStickState != stickState){
      if (stickState == CUTOFF){
        Serial.println("-1");
      } else if (stickState == LO_IDLE){
        Serial.println("-4");
      } else if (stickState == HI_IDLE){
        Serial.println("-3");
      } else if (stickState == FL_IDLE){
        Serial.println("-2");
      }
      prevStickState = stickState;
    }

    int aileronLeft = digitalRead(PIN_AILERON_TRIM_LEFT);
    static int aileronState = -1;
    if (lastAileronLeftButtonState != aileronLeft){
      lastAileronLeftDebounceTime = millis();
    }
    if ((millis() - lastAileronLeftDebounceTime) > 20){
      if (aileronLeft == LOW){
        //Serial.println("-5"); // LEFT AILERON signal
        aileronState = AIL_L;
      }
      aileronLeftButtonState = aileronLeft;
    }

    int aileronRight = digitalRead(PIN_AILERON_TRIM_RIGHT);
    if (lastAileronRightButtonState != aileronRight){
      lastAileronRightDebounceTime = millis();
    }
    if ((millis() - lastAileronRightDebounceTime) > 20){
      if (aileronRight == LOW){
        //Serial.println("-6"); // RIGHT AILERON signal
        aileronState = AIL_R;
      }
      aileronRightButtonState = aileronRight;
    }

     unsigned long currentAileronMillis = millis(); // Get the current time
    if (currentAileronMillis - previousAileronMillis >= 5) {
      previousAileronMillis = currentAileronMillis;
      if (aileronRight == HIGH && aileronLeft == HIGH){
        //Serial.println("-10"); // Rocker is in the middle
        aileronState = AIL_N;
      }
    }

    static int prevAileronState = -1;
    if (prevAileronState != aileronState){
      if (aileronState == AIL_L){
        Serial.println("-5");
      } else if (aileronState == AIL_R){
        Serial.println("-6");
      } else if (aileronState == AIL_N){
        Serial.println("-10");
      }
      prevAileronState = aileronState;
    }
     
    int fuelLeft = digitalRead(PIN_FUEL_SELECTOR_LEFT);
    int fuelRight = digitalRead(PIN_FUEL_SELECTOR_RIGHT);
    int fuelOff = digitalRead(PIN_FUEL_SELECTOR_OFF);
    unsigned long currentMillis = millis(); // Get the current time
    // Update the fuel selector position based on input
    if (fuelLeft == LOW) {
      fuelSelectorPos = FS_LEFT;
    } else if (fuelRight == LOW) {
      fuelSelectorPos = FS_RIGHT;
    } else if (fuelOff == LOW) {
      fuelSelectorPos = FS_OFF;
    }

  // // Check if it's time to send data
  // if (currentMillis - previousMillis >= 1) {
  //   previousMillis = currentMillis; // Update the last time
  //   // Send only if the value has changed
  //   if (prevFuelSelectorPos != fuelSelectorPos){
  //     if (fuelSelectorPos == FS_LEFT) {
  //       Serial.println("-11");
  //     } else if (fuelSelectorPos == FS_RIGHT) {
  //       Serial.println("-12");
  //     } else if (fuelSelectorPos == FS_OFF) {
  //       Serial.println("-13");
  //     }
  //     prevFuelSelectorPos = fuelSelectorPos;
  //   }

  // }

    if (stickState == FL_IDLE){
      int throttle = smoothAxisValue(THROTTLE_PIN, 50);
      //Serial.println(throttle);
      throttle = constrain(throttle, THROTTLE_REV_B, THROTTLE_MIN_B);
      unsigned long currentThrottleMillis = millis(); // Get the current time
      if (currentThrottleMillis - previousThrottleMillis >= 1) {
        previousThrottleMillis = currentThrottleMillis;

        if (throttle >= 0 && throttle <= 504) {
          static int prevThrottle = -1;
          int finalThrottle = map(throttle, 504, 0, 0, 1023);
          if (prevThrottle != finalThrottle){
            Serial.println(finalThrottle);
            prevThrottle = finalThrottle;
          }
        } else if (throttle <= -378 && throttle >= -650){ //-272 difference
          static int prevRevThrottle = -1;
          int finalThrottle = map(throttle, -378, -650, 10511, 10175);
          if (prevRevThrottle != finalThrottle){
            Serial.println(finalThrottle);  
            prevRevThrottle = finalThrottle;
          }
        
        } else if (throttle >= 510 && throttle <= 650){ //140 difference
          static int prevRevThrottle = -1;
          int finalThrottle = map(throttle, 650, 510, 10174, 10001);
          if (prevRevThrottle != finalThrottle){
            Serial.println(finalThrottle);  
            prevRevThrottle = finalThrottle;
          }
        }
      }
    }


    int flaps = smoothAxisValue(FLAPS_PIN, 10);
    //Serial.println(flaps);
    flaps = constrain(flaps, FLAPS_0_MIN, FLAPS_2_MAX);
    unsigned long currentFlapsMillis = millis(); // Get the current time
    static int prevFlaps = -1;
    if (currentFlapsMillis - previousFlapsMillis >= 10) {
      previousFlapsMillis = currentFlapsMillis;
      if (prevFlaps != flaps){
        if (flaps >= FLAPS_0_MIN && flaps <= FLAPS_0_MAX){
          Serial.println("-7");
        } else if (flaps >= FLAPS_1_MIN && flaps <= FLAPS_1_MAX) {
          Serial.println("-8");
        } else if (flaps >= FLAPS_2_MAX){
          Serial.println("-9");
        }
        prevFlaps = flaps;
      }
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

    lastAileronLeftButtonState = aileronLeft;
    lastAileronRightButtonState = aileronRight;
    //delay(20);  // Small delay to avoid overwhelming USB communication
  } else if (simType == XP && simDetected) {
    unsigned long currentTime = millis(); 
    unsigned long analogReadInterval = 300; // Default slow analog read interval
    static bool button2PrevState, button3PrevState;
    static bool loHiSequence = false, hiLoSequence;
    static unsigned long loHiTimeElapsed = 0;
    static bool fromCutOfftoLoIdleSeq = false;
    int revButton;
    static int stickState = -2;
    static int prevStickState = -1;
    static int prevThrottle = -1;
    static int prevRevThrottle = -1;
    int cutOff = digitalRead(PIN_FUEL_CUTOFF);
    int highIdle = digitalRead(PIN_HI_IDLE);
    int flightIdle = digitalRead(PIN_FLIGHT_IDLE);
    int loIdle = digitalRead(PIN_LO_IDLE);
    serialEvent();
    if(stringComplete){
      if (inputString == "X\n") {
        //simDetected = false;
        preambleSent = false;
        softwareReset();
      } else if (inputString == "P\n"){ // in case arduino is in the middle of the loop(), still try to detect simconnect restarts
        Serial.println("K");       
      } else if (inputString == "Z\n"){
        simType = XP;
        simDetected = true;
      }
      inputString = "";
      stringComplete = false;
    }
    
    if (cutOff == LOW){
      stickState = CUTOFF;
      
    }
    if (flightIdle == LOW){
      stickState = FL_IDLE;
      
    }

    if (highIdle == LOW){
      stickState = HI_IDLE;
      
    }
    if (loIdle == LOW){
      stickState = LO_IDLE;
      
    }

    if (prevStickState != stickState){
      if (stickState == CUTOFF){
        Serial.println("-1");
      } else if (stickState == LO_IDLE){
        Serial.println("-4");
      } else if (stickState == HI_IDLE){
        Serial.println("-3");
      } else if (stickState == FL_IDLE){
        Serial.println("-2");
      }
      prevStickState = stickState;
    }
    // if (cutOff == LOW){
    //   Serial.println("-1");
    // }
    // if (flightIdle == LOW && !reversing){
    //   Serial.println("-2");
    // }

    // if (highIdle == LOW){
    //   Serial.println("-3");
    // }
    // if (loIdle == LOW){
    //   Serial.println("-4");
    // }
    if (stickState == FL_IDLE){
      int throttle = smoothAxisValue(THROTTLE_PIN, 50);
    
      throttle = constrain(throttle, 406, 647);
      if (throttle >= 506 && throttle <= 647) {
        int finalThrottle = map(throttle, 506, 647, 1, 1023);
        if (prevThrottle != finalThrottle){
          Serial.println(finalThrottle);
          prevThrottle = finalThrottle;
        }
        //Serial.println(throttle);
        //Serial.println(finalThrottle);
      } else if (throttle <= 500 && throttle >= 406) {
        int finalThrottle = map(throttle, 406, 500, 1023,1);
        Serial.println(20000+finalThrottle);
      }
    }



    lastRevButtonState = revButton;
    delay(40);  // Small delay to avoid overwhelming USB communication
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