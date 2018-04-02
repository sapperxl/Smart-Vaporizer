// Copyright 2018 Michael Simmons, released under the GNU General Public License. Please send all feedback to sapperxl@yahoo.com
// If you like this and you see me feel free to buy me a beer!
#include <EEPROM.h>
#include <PID_v1.h>
#include <Adafruit_MAX31855.h>
#include <LiquidCrystal_I2C.h>
#include <MenuBackend.h>    //MenuBackend library, copyright by Alexander Brevig
#include <SPI.h>

//Define all the hardware pins
const int heaterPin = 6;
const int buttonPinLeft = 10;
const int buttonPinRight = 11;
const int buttonPinEsc = 12;
const int buttonPinEnter = 13;
const int MAXDO   = 3;
const int MAXCS   = 4;
const int MAXCLK  = 5;
const int buzzerPin = 2;

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
const int numReadings = 10;      // number of readings in array. Increase if slower more stable readings are reqd, reduce if faster readings are reqd
double readings[numReadings];      // the readings from thermocouple
int index;                  // the index of the current reading
int total;                  // the running total
int averageF;                // the average      

double Setpoint, Input, Output;    //Define PID variables we'll be connecting to
float Kp = 1; 
float Ki = 1;    //integral
float Kd = 1;  //derivative
PID myPID(&Input, &Output, &Setpoint,Kp, Ki, Kd, DIRECT);  //Specify the links and initial tuning parameters

int preheatSetpoint = 250; //Default preheat setpoint
int treatSetpoint = 350; //Default treat setpoint

unsigned long treatTime = 150000; //Default timer duration, 2.5 minutes, set to 5 seconds for testing
unsigned long now;
bool treating = false; // boolean which is true when running treat function
bool settingTreatTemp = false;
bool settingStandbyTemp = false;
bool settingTimer = false;
bool heating = false;

char lastButtonPushed = 0;
bool lastButtonEnterState = LOW;   // the previous reading from the Enter input pin
bool lastButtonEscState = LOW;   // the previous reading from the Esc input pin
bool lastButtonLeftState = LOW;   // the previous reading from the Left input pin
bool lastButtonRightState = LOW;   // the previous reading from the Right input pin

long lastEnterDebounceTime = 0;  // the last time the output pin was toggled
long lastEscDebounceTime = 0;  // the last time the output pin was toggled
long lastLeftDebounceTime = 0;  // the last time the output pin was toggled
long lastRightDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 300;    // the debounce time

bool right;    // boolean for right button
bool left;      // boolean for left button
bool enter;    // boolean for enter button
bool escape;    // boolean for esc button

static void menuChangeEvent(MenuChangeEvent changed);
static void menuUseEvent(MenuUseEvent used);

//Menu variables
MenuBackend menu = MenuBackend(menuUseEvent,menuChangeEvent);
//initialize menuitems
MenuItem preheatStandby = MenuItem("Preheat/Standby");
MenuItem startTreating = MenuItem("Start Treating");
MenuItem turnHeaterOff = MenuItem("Turn Heater Off");
MenuItem settings = MenuItem("Settings");
  MenuItem setTreatTemp = MenuItem("Set Treat Temp");
    MenuItem incTreatTemp = MenuItem("Inc Treat Temp");
    MenuItem decTreatTemp = MenuItem("Dec Treat Temp");
  MenuItem setStandbyTemp = MenuItem("Set Standby Temp");
    MenuItem incStandbyTemp = MenuItem("Inc Standby Temp");
    MenuItem decStandbyTemp = MenuItem("Dec Standby Temp");
  MenuItem setTreatTime = MenuItem("Set Treat Time");
    MenuItem incTreatTime = MenuItem("Inc Treat Time");
    MenuItem decTreatTime = MenuItem("Dec Treat Time");
MenuItem tunePID = MenuItem("Tune PID");  
  MenuItem tuneKp = MenuItem("Tune Kp");
    MenuItem incTuneKp = MenuItem("Inc Kp");
    MenuItem decTuneKp = MenuItem("Dec Kp");
  MenuItem tuneKi = MenuItem("Tune Ki");
    MenuItem incTuneKi = MenuItem("Inc Ki");
    MenuItem decTuneKi = MenuItem("Dec Ki");
  MenuItem tuneKd = MenuItem("Tune Kd");
    MenuItem incTuneKd = MenuItem("Inc Kd");
    MenuItem decTuneKd = MenuItem("Dec Kd");
MenuItem eeprom = MenuItem("EEPROM");
  MenuItem storeSettings = MenuItem("Store Settings");
  MenuItem loadDefaultSettings = MenuItem("Load Default Settings");


void setup()
{
EEPROM.get(0, Kp);    //Default PID settings for proportional
EEPROM.get(4, Ki);
EEPROM.get(8, Kd);
EEPROM.get(12, preheatSetpoint);
EEPROM.get(14, treatSetpoint);
EEPROM.get(16, treatTime);

pinMode(buttonPinLeft, INPUT);
pinMode(buttonPinRight, INPUT);
pinMode(buttonPinEnter, INPUT);
pinMode(buttonPinEsc, INPUT);
pinMode(buzzerPin, OUTPUT);

Setpoint = 0; //default start with heater off, you could change this to the standby temp if you want it to start heating as soon as its plugged in
 
//turn the PID on
myPID.SetMode(AUTOMATIC);

lcd.begin();
lcd.backlight();
lcd.setCursor(0, 0);
lcd.print(F("Steadfast"));
lcd.setCursor(0, 1);
lcd.print(F("Apiaries"));
delay(2000);  //Read the splash screen and let the thermocouple stabilize
lcd.clear();

/*
Menu Structure
Preheat/Standby -- Start Treating -- Turn Heater off -- Settings 
                                                      -- Set Treat Temp
                                                        -- ++ Treat Temp
                                                        -- -- Treat Temp
                                                      -- Set Standby Temp
                                                        -- ++ Standby Temp
                                                        -- -- Standby Temp 
                                                      -- Set Treat Time 
                                                        -- ++ Treat Time
                                                        -- -- Treat Time
                                                      -- Tune Kp
                                                        -- ++ Kp
                                                        -- -- Kp
                                                      -- Tune Ki
                                                        -- ++ Ki
                                                        -- -- Ki
                                                      -- Tune Kd
                                                        -- ++ Kd
                                                        -- -- Kd
                                                      -- Store Settings
                                                      -- Load Default Settings
 */

//configure menu
menu.getRoot().add(preheatStandby);
preheatStandby.addRight(startTreating).addRight(turnHeaterOff).addRight(settings).addRight(tunePID).addRight(eeprom);
settings.add(setTreatTemp).addRight(setStandbyTemp).addRight(setTreatTime);
setTreatTemp.add(incTreatTemp).addRight(decTreatTemp);
setStandbyTemp.add(incStandbyTemp).addRight(decStandbyTemp);
setTreatTime.add(incTreatTime).addRight(decTreatTime);
tunePID.add(tuneKp).addRight(tuneKi).addRight(tuneKd);
tuneKp.add(incTuneKp).addRight(decTuneKp);
tuneKi.add(incTuneKi).addRight(decTuneKi);
tuneKd.add(incTuneKd).addRight(decTuneKd);
eeprom.add(storeSettings).addRight(loadDefaultSettings);

menu.toRoot();    // go to root of menu

}  // setup()

void loop(){
temps();
heaterControl();
readButtons();  //I splitted button reading and navigation in two procedures because
navigateMenus();  //in some situations I want to use the button for other purpose (eg. to change some settings)
if (treating == true){
  treat();
  }
} //loop()

void menuChangeEvent(MenuChangeEvent changed){

MenuItem newMenuItem=changed.to; //get the destination menu

lcd.setCursor(0,1); //set the start position for lcd printing to the second row

if(newMenuItem.getName()==menu.getRoot()){
  lcd.print(F("Main Menu       "));
}else if(newMenuItem.getName()=="Preheat/Standby"){
  lcd.print(F("Preheat/Standby  "));
}else if(newMenuItem.getName()=="Start Treating"){
  lcd.print(F("Start Treating  "));
}else if(newMenuItem.getName()=="Turn Heater Off"){
  lcd.print(F("Turn Heater Off "));
}else if(newMenuItem.getName()=="Settings"){
  lcd.print(F("Settings        "));
}else if(newMenuItem.getName()=="Set Treat Temp"){
  lcd.print(F("Set Treat Temp  "));
}else if(newMenuItem.getName()=="Inc Treat Temp"){
  lcd.print(F("Inc Treat Temp  "));
}else if(newMenuItem.getName()=="Dec Treat Temp"){
  lcd.print(F("Dec Treat Temp  "));
}else if(newMenuItem.getName()=="Set Standby Temp"){
  lcd.print(F("Set Standby Temp"));
}else if(newMenuItem.getName()=="Inc Standby Temp"){
  lcd.print(F("Inc Standby Temp"));
}else if(newMenuItem.getName()=="Dec Standby Temp"){
  lcd.print(F("Dec Standby Temp"));
}else if(newMenuItem.getName()=="Set Treat Time"){
  lcd.print(F("Set Treat Time  "));
}else if(newMenuItem.getName()=="Inc Treat Time"){
  lcd.print(F("Inc Treat Time  "));
}else if(newMenuItem.getName()=="Dec Treat Time"){
  lcd.print(F("Dec Treat Time  "));
}else if(newMenuItem.getName()=="Tune PID"){
  lcd.print(F("Tune PID        "));
}else if(newMenuItem.getName()=="Tune Kp"){
  lcd.print(F("Tune Kp         "));
}else if(newMenuItem.getName()=="Inc Kp"){
  lcd.print(F("Inc Tune Kp     "));
}else if(newMenuItem.getName()=="Dec Kp"){
  lcd.print(F("Dec Tune Kp     "));
}else if(newMenuItem.getName()=="Tune Ki"){
  lcd.print(F("Tune Ki         "));
}else if(newMenuItem.getName()=="Inc Ki"){
  lcd.print(F("Inc Tune Ki     "));
}else if(newMenuItem.getName()=="Dec Ki"){
  lcd.print(F("Dec Tune Ki     "));
}else if(newMenuItem.getName()=="Tune Kd"){
  lcd.print(F("Tune Kd         "));
}else if(newMenuItem.getName()=="Inc Kd"){
  lcd.print(F("Inc Tune Kd     "));
}else if(newMenuItem.getName()=="Dec Kd"){
  lcd.print(F("Dec Tune Kd     "));
}else if(newMenuItem.getName()=="EEPROM"){
  lcd.print(F("EEPROM          "));
}else if(newMenuItem.getName()=="Store Settings"){
  lcd.print(F("Store Settings  "));
}else if(newMenuItem.getName()=="Load Default Settings"){
  lcd.print(F("Load Default Set"));
}
}

void menuUseEvent(MenuUseEvent used){
if (used.item.getName() == "Preheat/Standby"){
  Setpoint = preheatSetpoint;
}else if (used.item.getName() == "Start Treating"){
  treating = true;
  now = millis();
}else if (used.item.getName() == "Turn Heater Off"){
  Setpoint = 0;
  menu.toRoot();
}else if (used.item.getName() == "Inc Treat Temp"){
  treatSetpoint++;
  lcd.setCursor(9, 0);
  lcd.print(F("SP:"));
  lcd.print(treatSetpoint);
  lcd.print(F("   "));
  delay(500);
  lcd.setCursor(9, 0);
  lcd.print(F(" "));
}else if (used.item.getName() == "Dec Treat Temp"){
  treatSetpoint--;
  lcd.setCursor(9, 0);
  lcd.print(F("SP:"));
  lcd.print(treatSetpoint);
  lcd.print(F("   "));
  delay(500);
  lcd.setCursor(9, 0);
  lcd.print(F(" "));
}else if (used.item.getName() == "Inc Standby Temp"){
  preheatSetpoint++;
  lcd.setCursor(9, 0);
  lcd.print(F("SP:"));
  lcd.print(preheatSetpoint);
  lcd.print(F("   "));
  delay(500);
  lcd.setCursor(9, 0);
  lcd.print(F(" "));
}else if (used.item.getName() == "Dec Standby Temp"){
  preheatSetpoint--;
  lcd.setCursor(9, 0);
  lcd.print(F("SP:"));
  lcd.print(preheatSetpoint);
  lcd.print(F("   "));
  delay(500);
  lcd.setCursor(9, 0);
  lcd.print(F(" "));
}else if (used.item.getName() == "Inc Treat Time"){
  treatTime = treatTime + 1000;
  lcd.setCursor(9, 0);
  lcd.print(F("T:"));
  lcd.print(treatTime / 1000);
  lcd.print(F("s   "));
  delay(500);
  lcd.setCursor(9, 0);
  lcd.print(F(" "));
}else if (used.item.getName() == "Dec Treat Time"){
  treatTime = treatTime - 1000;
  lcd.setCursor(9, 0);
  lcd.print(F("T:"));
  lcd.print(treatTime / 1000);
  lcd.print(F("s   "));
  delay(500);
  lcd.setCursor(9, 0);
  lcd.print(F(" "));
}else if (used.item.getName() == "Inc Kp"){
  Kp = Kp + .01;
  lcd.setCursor(8, 0);
  lcd.print(F("Kp:"));
  lcd.print(Kp);
  lcd.print(F("   "));
  delay(500);
  lcd.setCursor(8, 0);
  lcd.print(F("    "));
}else if (used.item.getName() == "Dec Kp"){
  Kp = Kp - .01;
  lcd.setCursor(8, 0);
  lcd.print(F("Kp:"));
  lcd.print(Kp);
  lcd.print(F("   "));
  delay(500);
  lcd.setCursor(8, 0);
  lcd.print(F("    "));
}else if (used.item.getName() == "Inc Ki"){
  Ki = Ki + .01;
  lcd.setCursor(8, 0);
  lcd.print(F("Ki:"));
  lcd.print(Ki);
  lcd.print(F("   "));
  delay(500);
  lcd.setCursor(8, 0);
  lcd.print(F("    "));
}else if (used.item.getName() == "Dec Ki"){
  Ki = Ki - .01;  
  lcd.setCursor(8, 0);
  lcd.print(F("Ki:"));
  lcd.print(Ki);
  lcd.print(F("   "));
  delay(500);
  lcd.setCursor(8, 0);
  lcd.print(F("    "));
}else if (used.item.getName() == "Inc Kd"){
  Kd = Kd + .01;
  lcd.setCursor(8, 0);
  lcd.print(F("Kd:"));
  lcd.print(Kd);
  lcd.print(F("   "));
  delay(500);
  lcd.setCursor(8, 0);
  lcd.print(F("    "));
}else if (used.item.getName() == "Dec Kd"){
  Kd = Kd - .01;
  lcd.setCursor(8, 0);
  lcd.print(F("Kd:"));
  lcd.print(Kd);
  lcd.print(F("   "));
  delay(500);
  lcd.setCursor(8, 0);
  lcd.print(F("    "));
}else if (used.item.getName() == "Store Settings"){
  EEPROM.put(0, Kp);
  EEPROM.put(4, Ki);
  EEPROM.put(8, Kd);
  EEPROM.put(12, preheatSetpoint);
  EEPROM.put(14, treatSetpoint);
  EEPROM.put(16, treatTime);
  lcd.setCursor(0, 1);
  lcd.print(F("Stored          "));
  delay(500);
  lcd.setCursor(0, 1);
  lcd.print(F("Store Settings"));
}else if (used.item.getName() == "Load Default Settings"){
  Kp = 16.16;
  Ki = 0.14;
  Kd = 480.10;
  preheatSetpoint = 250;
  treatSetpoint = 350;
  treatTime = 150000;
  lcd.setCursor(0, 1);
  lcd.print(F("Loaded          "));
  delay(500);
  lcd.setCursor(0, 1);
  lcd.print(F("Load Default Settings"));
}
}

void readButtons(){ //read buttons status
int reading;
int buttonEnterState=LOW; // the current reading from the Enter input pin
int buttonEscState=LOW; // the current reading from the input pin
int buttonLeftState=LOW; // the current reading from the input pin
int buttonRightState=LOW; // the current reading from the input pin

//Enter button
// read the state of the switch into a local variable:
reading = digitalRead(buttonPinEnter);

// check to see if you just pressed the enter button 
// (i.e. the input went from LOW to HIGH), and you've waited 
// long enough since the last press to ignore any noise: 

// If the switch changed, due to noise or pressing:
if (reading != lastButtonEnterState) {
// reset the debouncing timer
lastEnterDebounceTime = millis();
} 

if ((millis() - lastEnterDebounceTime) > debounceDelay) {
// whatever the reading is at, it's been there for longer
// than the debounce delay, so take it as the actual current state:
buttonEnterState=reading;
lastEnterDebounceTime=millis();
}

// save the reading. Next time through the loop,
// it'll be the lastButtonState:
lastButtonEnterState = reading;


//Esc button 
// read the state of the switch into a local variable:
reading = digitalRead(buttonPinEsc);

// check to see if you just pressed the Down button 
// (i.e. the input went from LOW to HIGH), and you've waited 
// long enough since the last press to ignore any noise: 

// If the switch changed, due to noise or pressing:
if (reading != lastButtonEscState) {
// reset the debouncing timer
lastEscDebounceTime = millis();
} 

if ((millis() - lastEscDebounceTime) > debounceDelay) {
// whatever the reading is at, it's been there for longer
// than the debounce delay, so take it as the actual current state:
buttonEscState = reading;
lastEscDebounceTime=millis();
}

// save the reading. Next time through the loop,
// it'll be the lastButtonState:
lastButtonEscState = reading; 


//Down button 
// read the state of the switch into a local variable:
reading = digitalRead(buttonPinRight);

// check to see if you just pressed the Down button 
// (i.e. the input went from LOW to HIGH), and you've waited 
// long enough since the last press to ignore any noise: 

// If the switch changed, due to noise or pressing:
if (reading != lastButtonRightState) {
// reset the debouncing timer
lastRightDebounceTime = millis();
} 

if ((millis() - lastRightDebounceTime) > debounceDelay) {
// whatever the reading is at, it's been there for longer
// than the debounce delay, so take it as the actual current state:
buttonRightState = reading;
lastRightDebounceTime =millis();
}

// save the reading. Next time through the loop,
// it'll be the lastButtonState:
lastButtonRightState = reading; 


//Up button 
// read the state of the switch into a local variable:
reading = digitalRead(buttonPinLeft);

// check to see if you just pressed the Down button 
// (i.e. the input went from LOW to HIGH), and you've waited 
// long enough since the last press to ignore any noise: 

// If the switch changed, due to noise or pressing:
if (reading != lastButtonLeftState) {
// reset the debouncing timer
lastLeftDebounceTime = millis();
} 

if ((millis() - lastLeftDebounceTime) > debounceDelay) {
// whatever the reading is at, it's been there for longer
// than the debounce delay, so take it as the actual current state:
buttonLeftState = reading;
lastLeftDebounceTime=millis();;
}

// save the reading. Next time through the loop,
// it'll be the lastButtonState:
lastButtonLeftState = reading; 

//records which button has been pressed
if (buttonEnterState==HIGH){
lastButtonPushed=buttonPinEnter;
enter = true;
right = false;
left = false;

}else if(buttonEscState==HIGH){
lastButtonPushed=buttonPinEsc;
escape = true;
enter = false;
right = false;
left = false;

}else if(buttonRightState==HIGH){
lastButtonPushed=buttonPinRight;
right = true;
left = false;
enter = false;

}else if(buttonLeftState==HIGH){
lastButtonPushed=buttonPinLeft;
left = true;
right = false;
enter = false;

}else{
lastButtonPushed=0;
enter = false;
right = false;
left = false;
} 
}

void navigateMenus() {
MenuItem currentMenu=menu.getCurrent();

switch (lastButtonPushed){
case buttonPinEnter:
if(!(currentMenu.moveDown())){ //if the current menu has a child and has been pressed enter then menu navigate to item below
menu.use();
}else{ //otherwise, if menu has no child and has been pressed enter the current menu is used
menu.moveDown();
} 
break;
case buttonPinEsc:
menu.toRoot(); //go to root
break;
case buttonPinRight:
menu.moveRight();
break; 
case buttonPinLeft:
menu.moveLeft();
break; 
}

lastButtonPushed=0; //reset the lastButtonPushed variable
}

void temps(){
   int f = thermocouple.readFarenheit();
   if (isnan(f)) {    //  if f is not a number
     lcd.setCursor(0,0);
     lcd.print(F("TC Err"));
   } else {
   total = total - readings[index];         
   readings[index] = f;         // read from the sensor: 
   total = total + readings[index];     // add the reading to the total:    
   index = index + 1;                 // advance to the next position in the array:         
   if (index >= numReadings)          // if we're at the end of the array...
   index = 0;                         // ...wrap around to the beginning:                          
   averageF = total / numReadings;    // calculate the average:
    lcd.setCursor(0, 0);
     lcd.print(F("TEMP:")); 
     lcd.print(f);
     lcd.setCursor(10, 0);
     lcd.print(F("SP:"));
     lcd.print(Setpoint);
}}

void heaterControl(){
  Input = analogRead(averageF);
  myPID.Compute();
  analogWrite(heaterPin, Output);
}

/* commented out for testing without heater hooked up
void heatingUp(){
  Setpoint = treatSetpoint;
  lcd.setCursor(0, 1);
  lcd.print(F("Heating         ");
  heating = true; 
  if (averageF > treatSetpoint - .5) {
    heating = false;
  }
}
*/

void treat(){
//  if (averageF < treatSetpoint - .5){    Commented out for testing without heater
//    heatingUp();   Commented out for testing without heater
if (millis() - now <= treatTime){ // Replace with } else if (millis() - now <= treatTime && heating == false){ for heating function
    Setpoint = treatSetpoint;
    lcd.setCursor(0, 1);
    lcd.print(F("Treating "));
    lcd.print((treatTime - (millis() - now)) / 1000);
    lcd.print(F("      "));
    }else{ 
    Setpoint = preheatSetpoint;
    treating = false;
    digitalWrite(buzzerPin, HIGH);
    delay(500);
    digitalWrite(buzzerPin, LOW);
    menu.toRoot();
    }
}
