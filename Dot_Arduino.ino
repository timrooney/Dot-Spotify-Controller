

/* Tim Rooney
 * Dot [Spotify Controller]
 */

#define CLICKBTN_PULLUP HIGH


class ClickButton
{
public:
  ClickButton(uint8_t buttonPin);
  ClickButton(uint8_t buttonPin, boolean active);
  ClickButton(uint8_t buttonPin, boolean active, boolean internalPullup);
  void Update();
  int clicks;                   // button click counts to return
  boolean depressed;            // the currently debounced button (press) state (presumably it is not sad :)
  long debounceTime;
  long multiclickTime;
  long longClickTime;
private:
  uint8_t _pin;                 // Arduino pin connected to the button
  boolean _activeHigh;          // Type of button: Active-low = 0 or active-high = 1
  boolean _btnState;            // Current appearant button state
  boolean _lastState;           // previous button reading
  int _clickCount;              // Number of button clicks within multiclickTime milliseconds
  long _lastBounceTime;         // the last time the button input pin was toggled, due to noise or a press
};

// -------- end clickButton.h --------


// -------- clickButton.cpp --------

ClickButton::ClickButton(uint8_t buttonPin)
{
  _pin           = buttonPin;
  _activeHigh    = LOW;           // Assume active-low button
  _btnState      = !_activeHigh;  // initial button state in active-high logic
  _lastState     = _btnState;
  _clickCount    = 0;
  clicks         = 0;
  depressed      = false;
  _lastBounceTime= 0;
  debounceTime   = 20;            // Debounce timer in ms
  multiclickTime = 250;           // Time limit for multi clicks
  longClickTime  = 1000;          // time until long clicks register
  pinMode(_pin, INPUT);
}


ClickButton::ClickButton(uint8_t buttonPin, boolean activeType)
{
  _pin           = buttonPin;
  _activeHigh    = activeType;
  _btnState      = !_activeHigh;  // initial button state in active-high logic
  _lastState     = _btnState;
  _clickCount    = 0;
  clicks         = 0;
  depressed      = 0;
  _lastBounceTime= 0;
  debounceTime   = 20;            // Debounce timer in ms
  multiclickTime = 250;           // Time limit for multi clicks
  longClickTime  = 1000;          // time until long clicks register
  pinMode(_pin, INPUT);
}

ClickButton::ClickButton(uint8_t buttonPin, boolean activeType, boolean internalPullup)
{
  _pin           = buttonPin;
  _activeHigh    = activeType;
  _btnState      = !_activeHigh;  // initial button state in active-high logic
  _lastState     = _btnState;
  _clickCount    = 0;
  clicks         = 0;
  depressed      = 0;
  _lastBounceTime= 0;
  debounceTime   = 20;            // Debounce timer in ms
  multiclickTime = 250;           // Time limit for multi clicks
  longClickTime  = 1000;          // time until "long" click register

  // Turn on internal pullup resistor if applicable
  if (_activeHigh == LOW && internalPullup == CLICKBTN_PULLUP)
    pinMode(_pin, INPUT_PULLUP);
  else
    pinMode(_pin, INPUT);
}



void ClickButton::Update()
{
  long now = (long)millis();      // get current time
  _btnState = digitalRead(_pin);  // current appearant button state

  // Make the button logic active-high in code
  if (!_activeHigh) _btnState = !_btnState;

  // If the switch changed, due to noise or a button press, reset the debounce timer
  if (_btnState != _lastState) _lastBounceTime = now;


  // debounce the button (Check if a stable, changed state has occured)
  if (now - _lastBounceTime > debounceTime && _btnState != depressed)
  {
    depressed = _btnState;
    if (depressed) _clickCount++;
  }

  // If the button released state is stable, report nr of clicks and start new cycle
  if (!depressed && (now - _lastBounceTime) > multiclickTime)
  {
    // positive count for released buttons
    clicks = _clickCount;
    _clickCount = 0;
  }

  // Check for "long click"
  if (depressed && (now - _lastBounceTime > longClickTime))
  {
    // negative count for long clicks
    clicks = 0 - _clickCount;
    _clickCount = 0;
  }

  _lastState = _btnState;
}

// -------- end clickButton.cpp --------


/* ClickButton library demo
 
 OUtput a message on Serial according to different clicks on one button.
 
 Short clicks:
 
 Single click - 
 Double click - 
 Triple click - 
 
 Long clicks (hold button for one second or longer on last click):
 
 Single-click - 
 Double-click - 
 Triple-click - 
 
 2010, 2013 raron
 GNU GPLv3 license
 */


// the Button
const int buttonPin1 = 8;
ClickButton button1(buttonPin1, LOW, CLICKBTN_PULLUP);

// Button results 
int function = 0;

//int buttonState = 0;


//
#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(5, 6);
//   avoid using pins with LEDs attached
int threshold = 15;
int threshold2 = 7;


const int encoderSwitchPin = 8; //push button switch

//int buttonState = 0;
//int buttonStateOld = 0;


String stateDescription = "start";



void setup() {
  Serial.begin(1200);
  //Serial.println("0");


  pinMode(encoderSwitchPin, INPUT); 

  digitalWrite(encoderSwitchPin, HIGH); //turn pullup resistor on


  pinMode(buttonPin1, INPUT_PULLUP);

  // Setup button timers (all in milliseconds / ms)
  // (These are default if not set, but changeable for convenience)
  button1.debounceTime   = 20;   // Debounce timer in ms
  button1.multiclickTime = 250;  // Time limit for multi clicks
  button1.longClickTime  = 1000; // time until "held-down clicks" register



}

long oldPosition  = -999;

void loop() {

  button1.Update(); 

  // Save click codes in LEDfunction, as click codes are reset at next Update()
  if (button1.clicks != 0) function = button1.clicks;

  if(button1.clicks == 1) Serial.println("A")  ;//single click

  if(function == 2) Serial.println("N");//double click

  if(function == 3) Serial.println("P");//triple click

  if(function == 4) 
  {
    //Serial.println ("switching state");

    if (stateDescription == "start")
    {
      stateDescription = "next1";
      //Serial.println ("switched to");
      //Serial.println (stateDescription);

    }
    else if (stateDescription == "next1")
    {
      stateDescription = "next2";
      //Serial.println ("switched to");
      //Serial.println (stateDescription);

    }
    else if (stateDescription == "next2")
    {
      stateDescription = "start";
      //Serial.println ("switched to");
      //Serial.println (stateDescription);

    }

  }

  if(function == -1) Serial.println("F");//single long click

  if(function == -2) Serial.println("T");//double long click

  if(function == -3) Serial.println("R");//triple long click

   function = 0;
  delay(5);

  
  long newPosition = myEnc.read();
  //while (digitalRead(encoderSwitchPin) == HIGH) {
  //while(function == -1){
  if (stateDescription == "start" && newPosition > oldPosition + threshold){ 
    oldPosition = newPosition;
    //Serial.println(newPosition);
    Serial.println ("C");
  }
  else if
    (stateDescription == "start" && newPosition < oldPosition - threshold2) {
    oldPosition  = newPosition;
    //Serial.println(newPosition);
    Serial.println ("D");


    }
  if (stateDescription == "next1" && newPosition > oldPosition + threshold) {
    oldPosition = newPosition;
    //Serial.println(newPosition);
    Serial.println ("Z");
  }
  else if
    (stateDescription == "next1" && newPosition < oldPosition - threshold2) {
    oldPosition = newPosition;
    //Serial.println(newPosition);
    Serial.println ("V");



    function = 0;
    delay(5);


    }
  if (stateDescription == "next2" && newPosition > oldPosition + threshold) {
    oldPosition = newPosition;
    //Serial.println(newPosition);
    Serial.println ("J");
  }
  else if
    (stateDescription == "next2" && newPosition < oldPosition - threshold2) {
    oldPosition = newPosition;
    //Serial.println(newPosition);
    Serial.println ("K");



    function = 0;
    delay(5);

  }

}



