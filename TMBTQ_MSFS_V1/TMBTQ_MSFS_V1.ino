#include <Joystick.h>

#define PIN_FUEL_CUTOFF 2  // Button 1: Cutoff
#define PIN_LO_IDLE 3      // Button 2: Idle Low (sequence trigger)
#define PIN_HI_IDLE 4      // Button 3: HI Idle (rocker switch)
#define PIN_FLIGHT_IDLE 5  // Button 4: Flight Idle (rocker switch)
#define PIN_REVERSE 6      // Button 5: Reverse
#define PIN_TOGA 7         // Button 6: TOGA
#define PIN_FUEL_SELECTOR_LEFT 8    // Button 7: Fuel Selector Left
#define PIN_FUEL_SELECTOR_RIGHT 9   // Button 8: Fuel Selector Right
#define PIN_FUEL_SELECTOR_OFF 10    // Button 9: Fuel Selector Off
#define PIN_AILERON_TRIM_LEFT 14    // Button 10: Aileron Trim Left
#define PIN_AILERON_TRIM_RIGHT 15   // Button 11: Aileron Trim Right

#define ANALOG_PIN_X A0    // Analog input for X axis
#define ANALOG_PIN_Y A1    // Analog input for Y axis
//#define ANALOG_PIN_Z A2    // Analog input for Z axis
#define ANALOG_PIN_Rx A3   // Analog input for Rx axis
#define DEBOUNCE_DELAY 200 // Debounce delay for button presses

#define SMOOTHING_READS 10 // Number of reads for smoothing all axes

Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 
  11, // Number of buttons
  0,  // No hat switches
  true,  // X Axis
  false,  // Y Axis
  false,  // Z Axis
  true,  // Rx Axis
  false, // Ry Axis
  false, false, false, false, false, false
);

bool sequenceActive = false;         // Track if the sequence is running
unsigned long sequenceStartTime = 0; // Timer for the sequence
int sequenceStep = 0;                // Track the current step of the sequence
bool idleLoTriggered = false;        // Track if Idle Low was triggered
unsigned long lastButton2PressTime = 0; // Track last Idle Low button press
unsigned long lastAnalogReadTime = 0;   // Track the last time we read A0, A1, A2, A3

bool button3State = false;  // Initial state of Button 3 (HI_IDLE) set to true
bool button4State = false; // Initial state of Button 4 (FLIGHT_IDLE)
bool flightIdleActive = false; // Keep track of Flight Idle state to maintain smooth reads

int smoothAxisValue(int pin, int smoothingReads) {
  int totalValue = 0;
  for (int i = 0; i < smoothingReads; i++) {
    totalValue += analogRead(pin);
  }
  return totalValue / smoothingReads;  // Return the averaged value
}

void setup() {
  pinMode(PIN_FUEL_CUTOFF, INPUT_PULLUP); // Cutoff button uses internal pull-up resistor
  pinMode(PIN_LO_IDLE, INPUT_PULLUP);     // Idle Low button uses internal pull-up resistor
  pinMode(PIN_HI_IDLE, INPUT_PULLUP);     // HI Idle (rocker switch)
  pinMode(PIN_FLIGHT_IDLE, INPUT_PULLUP); // Flight Idle (rocker switch)
  pinMode(PIN_REVERSE, INPUT_PULLUP);     // Reverse
  pinMode(PIN_TOGA, INPUT_PULLUP);        // TOGA
  pinMode(PIN_FUEL_SELECTOR_LEFT, INPUT_PULLUP);   // Fuel Selector Left
  pinMode(PIN_FUEL_SELECTOR_RIGHT, INPUT_PULLUP);  // Fuel Selector Right
  pinMode(PIN_FUEL_SELECTOR_OFF, INPUT_PULLUP);    // Fuel Selector Off
  pinMode(PIN_AILERON_TRIM_LEFT, INPUT_PULLUP);    // Aileron Trim Left
  pinMode(PIN_AILERON_TRIM_RIGHT, INPUT_PULLUP);   // Aileron Trim Right

  // Set the initial state for HI_IDLE (Button 3) to ON
  //Joystick.setButton(2, true);

  Joystick.begin();
  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long analogReadInterval = 300; // Default slow analog read interval
  static bool button2PrevState, button3PrevState;
  static bool loHiSequence = false, hiLoSequence;
  static unsigned long loHiTimeElapsed = 0;
  static bool fromCutOfftoLoIdleSeq = false;

  
  if (!sequenceActive && !hiLoSequence && !loHiSequence) {
    if (!digitalRead(PIN_HI_IDLE)) { // Button 3 pressed (HI_IDLE)
      button3State = true;
      //button4State = false;
      flightIdleActive = false;   // Flight Idle becomes inactive
      analogReadInterval = 300;  // Slow analog reads when HI_IDLE is active
      Joystick.setRxAxis(0);    //FOR PROPELLER 1 AXIS
    } else {
      button3State = false;
    }

    if (!digitalRead(PIN_FLIGHT_IDLE) || flightIdleActive) { // Button 4 pressed or was previously pressed
      //button3State = false;
      button4State = true;
      flightIdleActive = true;    // Maintain Flight Idle active
      analogReadInterval = 50;    // Fast analog reads for smoother axis when FLIGHT_IDLE is active
      Joystick.setRxAxis(1024);   //FOR PROPELLER 1 AXIS
    } else {
      button4State = false;
    }
    if (button3State == true && button3State != button3PrevState) { // perform lo idle to hi idle sequence
      hiLoSequence = true;
      loHiTimeElapsed = millis();
    } else {
      fromCutOfftoLoIdleSeq = false;
      //Joystick.setButton(2, button3State); // Set Button 3 (HI_IDLE)
      
    }
    
    Joystick.setButton(3, button4State); // Set Button 4 (FLIGHT_IDLE)
  }
  button3PrevState = button3State;
  if (hiLoSequence && !sequenceActive) {
      if (millis() - loHiTimeElapsed >= 0) {
      // Step 1: Press bt1 (Cutoff) for 1 second
      Joystick.setButton(1, 1); // Press Button 2 
      if (millis() - loHiTimeElapsed >= 500) {  // After 1 second
        Joystick.setButton(1, 0); // Press Button 2 
       loHiTimeElapsed = millis(); // Reset the timer for Step 2
        hiLoSequence = false;
        loHiSequence = true;
      }
    }
  }
  if (loHiSequence) {
        if (millis() - loHiTimeElapsed >= 0) {
      // Step 1: Press bt1 (Cutoff) for 1 second
      Joystick.setButton(2, 1); // Press Button 3
      if (millis() - loHiTimeElapsed >= 150) {  // After 1 second
        Joystick.setButton(2, 0); // Release Button 3
        loHiTimeElapsed = millis(); // Reset the timer for Step 2
        hiLoSequence = false;
        loHiSequence = false;
      }
    }
  }

  // Handle Cutoff (Button 1) as normal, press/release directly
  if (digitalRead(PIN_FUEL_CUTOFF) == LOW && !sequenceActive) {

    Joystick.setButton(0, 1);  // Press Button 1 (bt1) as long as it's pressed
  } else if (!sequenceActive) {

    Joystick.setButton(0, 0);  // Release Button 1 (bt1) when not pressed
  }

  // Handle Idle Low (Button 2) with debounce logic
  bool button2CurrentState = digitalRead(PIN_LO_IDLE);

  // Debounce check for Idle Low button
  if (button2CurrentState == LOW && (currentTime - lastButton2PressTime > DEBOUNCE_DELAY)) {
    if (!sequenceActive && !idleLoTriggered) {
      sequenceActive = true;         // Activate the sequence
      idleLoTriggered = true;        // Mark that Idle Low was triggered
      sequenceStartTime = millis();  // Start the timer for the sequence
      sequenceStep = 1;              // Start with Step 1


    }
    lastButton2PressTime = currentTime;  // Update the last button press time
  }
  button2PrevState = button2CurrentState;

  // Run the sequence if it's active
  if (sequenceActive) {
    // Suspend analog reads during the sequence to avoid interference
    if (sequenceStep == 1 && millis() - sequenceStartTime >= 0) {
      // Step 1: Press bt1 (Cutoff) for 1 second
      Joystick.setButton(0, 1); // Press Button 1 (bt1) for the sequence
      if (millis() - sequenceStartTime >= 500) {  // After 1 second
        Joystick.setButton(0, 0); // Release Button 1 (bt1)
        sequenceStep = 2;         // Move to Step 2
        sequenceStartTime = millis(); // Reset the timer for Step 2
      }
    }

    if (sequenceStep == 2 && millis() - sequenceStartTime >= 0) {
      
      // Step 2: Press bt2 (Idle Low) for 1/2 second
      Joystick.setButton(1, 1); // Press Button 2 (bt2)
      if (millis() - sequenceStartTime >= 150) {  // After 1/2 second
        Joystick.setButton(1, 0);  // Release Button 2 (bt2)
        sequenceActive = false;    // End the sequence
      }
    }
  }

  // Reset the Idle Low trigger when the button is released
  if (button2CurrentState == HIGH && !sequenceActive) {
    idleLoTriggered = false;  // Allow Idle Low to trigger the sequence again
  }

  // Read A0, A1, A2, A3 based on throttle quadrant state with smoothing for smoother values
  if (flightIdleActive == true && currentTime - lastAnalogReadTime > analogReadInterval) {
  //if ( currentTime - lastAnalogReadTime > 50) {
    int xValue = smoothAxisValue(ANALOG_PIN_X, SMOOTHING_READS);  // Smooth read for X-axis
    //int yValue = smoothAxisValue(ANALOG_PIN_Y, SMOOTHING_READS);  // Smooth read for Y-axis
   // int zValue = smoothAxisValue(ANALOG_PIN_Z, SMOOTHING_READS);  // Smooth read for Z-axis
    //int rxValue = smoothAxisValue(ANALOG_PIN_X, SMOOTHING_READS); // Smooth read for Rx-axis
    //Serial.println(xValue);
    Joystick.setXAxis(xValue);


    lastAnalogReadTime = currentTime;  // Update the last analog read time
  }

  // Handle buttons 5 through 11 (Reverse to Aileron Trim Right)
  // Joystick.setButton(4, !digitalRead(PIN_REVERSE));
  // Joystick.setButton(5, !digitalRead(PIN_TOGA));
  // Joystick.setButton(6, !digitalRead(PIN_FUEL_SELECTOR_LEFT));
  // Joystick.setButton(7, !digitalRead(PIN_FUEL_SELECTOR_RIGHT));
  // Joystick.setButton(8, !digitalRead(PIN_FUEL_SELECTOR_OFF));
  // Joystick.setButton(9, !digitalRead(PIN_AILERON_TRIM_LEFT));
  // Joystick.setButton(10, !digitalRead(PIN_AILERON_TRIM_RIGHT));

  delay(20);  // Small delay to avoid overwhelming USB communication
}
