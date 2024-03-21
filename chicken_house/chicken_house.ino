//#include <LowPower.h>
#define LDR_PIN A5              // Gets input from LDR
#define LPWM_PIN 5              // Arduino PWM output pin 5; connect to IBT-2 pin 2 (LPWM)
#define RPWM_PIN 6              // Arduino PWM output pin 6; connect to IBT-2 pin 1 (RPWM)
#define LR_EN_PIN   7           // Arduino pin 7 connected to both L_EN and R_EN
#define BUTTON_PIN  2           // Arduino pin 2 for the button to open/close the door
#define SPEEDPWM 255            // Speed of the motor, should be a value between 0 and 255
#define MOTORTIME 12000         // time for the motor to finish opening/closing the door in ms
#define LDR_TIME_PERIOD  10000  // time period in ms to measure value from LDR sensor
#define BUTTON_TIME_PERIOD  1000 // threshold in ms to press button again - prevents erratic multiple presses
#define LDR_VOLTAGE 1000        // Threshold for the voltage drop across the LDR to distinguish night and day, value between 0 and 1023, high values: dark, low values: bright

#define DEBUG                   // Set this to get additional output via Serial
#define DEBUG_TIME_PERIOD 5000  // time period for sending debug info

//int i;
int dark_cnt = 0, bright_cnt = 0, ldrValue;
bool door_open = false, door_opening = false, door_closed = false, door_closing = false;
unsigned long time_motor, time_ldr, time_button = 0;

volatile bool button_pressed = false;

#ifdef DEBUG
  unsigned long time_debug = 0;
#endif

void open_door()
{
  door_opening = true;
  door_closing = false;
  door_closed = false;
  digitalWrite(LR_EN_PIN, HIGH);  // enable left and right rotation of motor
  analogWrite(LPWM_PIN, 0);
  analogWrite(RPWM_PIN, SPEEDPWM);

  #ifdef DEBUG  
    Serial.println("Opening door...");
  #endif

  time_motor = millis();  // set time when motor started moving
  bright_cnt = 0;
}

void close_door()
{
  door_closing = true;
  door_opening = false;
  door_open = false;
  digitalWrite(LR_EN_PIN, HIGH);  // enable left and right rotation of motor
  analogWrite(LPWM_PIN, SPEEDPWM);
  analogWrite(RPWM_PIN, 0);

  #ifdef DEBUG   
    Serial.println("Closing door...");
  #endif
  
  time_motor = millis();  // set time when motor started moving
  dark_cnt = 0;
}

void measure_ldr()
{
  time_ldr = millis();  // set time when last measurement happened
  ldrValue = analogRead(LDR_PIN);   // measure voltage drop over LDR, value is in the range 0 to 1023

  #ifdef DEBUG
    Serial.print("Measured ldrValue = ");
    Serial.println(ldrValue);
  #endif
  
  if (ldrValue < LDR_VOLTAGE)
  {
    // it is bright
    if (door_closed) {
      bright_cnt++;
      dark_cnt = 0;
    }
  }
  else
  {
    // it is dark
    if (door_open) {
      dark_cnt++;
      bright_cnt = 0;
    }
  }
}

void button_press() {
  button_pressed = true;
}

void setup()
{
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(LR_EN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  #ifdef DEBUG
    Serial.begin(9600);
    Serial.println("Starting system...");
  #endif
  open_door();    // initializing state of the door

  // take first measurement of LDR value immediately after system is initialized
  measure_ldr();

  attachInterrupt(digitalPinToInterrupt(2), button_press, RISING);
}

void loop()
{
  if (button_pressed) {
    if (millis() - time_button <= BUTTON_TIME_PERIOD)
      // Ignore button press
      button_pressed = false;
    else {
      time_button = millis();
      button_pressed=false;
      #ifdef DEBUG
        Serial.println("Button pressed.");
      #endif
      if (door_open || door_opening)
        close_door();
      else
        open_door();
    }
  }

  if (millis() - time_ldr > LDR_TIME_PERIOD) {
    // LDR_TIME_PERIOD has passed since last measurement
    measure_ldr();
  }

  if (millis() - time_motor > MOTORTIME) {
    // MOTORTIME has passed since motor started moving
    if (door_opening) {
      // door was opening
      digitalWrite(LR_EN_PIN, LOW);   // disable left and right rotation of motor
      door_opening = false;
      door_open = true;    
        
      #ifdef DEBUG 
        Serial.println("Door opened.");
      #endif  
    }
    else if (door_closing) {
      // door was closing
      digitalWrite(LR_EN_PIN, LOW);   // disable left and right rotation of motor
      door_closing = false;
      door_closed = true;

      #ifdef DEBUG
        Serial.println("Door closed.");
      #endif
    }
  }  
  
  #ifdef DEBUG
    if (millis() - time_debug > DEBUG_TIME_PERIOD) {
      //Send debug info
      time_debug = millis();
      Serial.print("bright_cnt = ");
      Serial.println(bright_cnt);
      Serial.print("dark_cnt = ");
      Serial.println(dark_cnt);
      Serial.print("door_open = ");
      Serial.println(door_open);
      Serial.print("door_opening = ");
      Serial.println(door_opening);
      Serial.print("door_closing = ");
      Serial.println(door_closing);
      Serial.print("door_closed = ");
      Serial.println(door_closed);
      Serial.println();
    }
  #endif

  if (bright_cnt == 3)
      open_door();
  else if (dark_cnt == 3)
      close_door();

  /*#ifdef DEBUG
    delay(200); // for the serial data to be sent before powerdown
  #endif
  
  for (i=0; i<=1; i++) {
    //do not power down if door is moving
    if (!door_opening and !door_closing)
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }*/
}
