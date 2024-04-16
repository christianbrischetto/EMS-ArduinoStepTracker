#include <LiquidCrystal_I2C.h>
#include <PL_ADXL355.h>
#include <Stream.h>

LiquidCrystal_I2C lcd(0x27,16,2); //set lcd address and size

//init variables
const int calibrationPin_in = 5; //pullup
const int actionPin_in = 6; //pullup 
const int menuPin_in = 7; //pullup 
int actionPin_out = 2; //CHANGING output pin
int menuState = 0; //variable for menu cycle
int numAverages = 50; //for accel value filtering
bool calibrationComplete = false;
int stepCount;
int eventCount;
unsigned long timeLast = 0;
float pace;
double gx;
double gy;
double gz;
double ref_x;
double ref_y;
double ref_z;

// _____________________________________________________________________
//
//  MAKE THIS BASED OFF OF THE SELF TEST SEQUENCE FOLLOWING CALIBRATION 
// ______________________________________________________________________
double gain = 1; // 1.2 for PCB
// 2V / 3.6V power supply = 195mV/g / 360mV/g output sensitivity
// 3.3V/3.6V * 0.360mV 
//                    = 0.33V/g
double outputSensitivity = 0.33/5 * gain * 1023;

void homeScreenDisplay(){
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0,0);   //Set cursor to character 2 on line 0
  lcd.print("<-Action");
  lcd.setCursor(10,1);
  lcd.print("Menu->");
}

void selfTestDisplay(){
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(3,0);  
  lcd.print("Self Test");

  // trigger SelfTest routine
  if ((menuState == 1 || menuState == 4) && (digitalRead(actionPin_in) == LOW)){
    digitalWrite(actionPin_out, HIGH);
    digitalWrite(3, HIGH);
  }
  else{
    digitalWrite(actionPin_out, LOW);
    digitalWrite(3, LOW);
  }
}

void steptrackerDisplay(int steps){
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(3,0);  
  lcd.print("Steps:  ");
  lcd.print(steps);
  
  if (menuState == 2 && (digitalRead(actionPin_in) == LOW)){
    stepCount = 0;
  }
}

void walkingpaceDisplay(float pace){
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(3,0);  
  lcd.print("Pace:  ");
  lcd.print(pace);

  lcd.setCursor(5,1);
  lcd.print("steps/s");
}

int CountSteps(){ //count steps 
  double magnitude = sqrt(gx*gx + gy*gy + gz*gz);
  if(magnitude > 1.1){
    stepCount++;
    eventCount++;
  }
  pace = TrackPace();

  return (stepCount);
}

float TrackPace(){ //walking pace calculation
  unsigned long timeCurrent = millis();
  if((timeCurrent - timeLast) >= 5000){
    pace = (float)eventCount / ((timeCurrent - timeLast)/5000);

    eventCount = 0;
    timeLast = timeCurrent;
  }
  return pace;
}

void calibrate(){
  // sets the current ADXL355 reading as your current states
  ref_x = analogRead(A2); 
  ref_y = analogRead(A1); 
  ref_z = analogRead(A0); 
}

void getXYZg(double& gx, double& gy, double& gz){
  double currentx = 0;
  for(int i = 0; i < numAverages; i++){
    double x = analogRead(A2);
    currentx += x;
  }
  currentx = currentx/numAverages;

  double currenty = 0;
  for(int i = 0; i < numAverages; i++){
    double y = analogRead(A1);
    currenty += y;
  }
  currenty = currenty/numAverages;

  double currentz = 0;
  for(int i = 0; i < numAverages; i++){
    double z = analogRead(A0);
    currentz += z;
  }
  currentz = currentz/numAverages;
  
  // get difference in current to reference voltage
  // scale as a percentage of the output sensitivity
  gx = (currentx - ref_x)/outputSensitivity; 
  gy = (currenty - ref_y)/outputSensitivity;  
  gz = (currentz - ref_z)/outputSensitivity + 1;  
  
}

void printXYZg(double gx, double gy, double gz){
  delay(10); //to stop the screen resetting constantly
  lcd.clear();
  lcd.backlight();
  
  if(gx >= 0){
    lcd.setCursor(1,0);
  }
  else{
    lcd.setCursor(0,0);
  }
  lcd.print(gx);
  lcd.print("g");

  if(gy >= 0){
    lcd.setCursor(9,0);
  }
  else{
    lcd.setCursor(8,0);
  }
  lcd.print(gy);
  lcd.print("g");

  if(gz >= 0){
    lcd.setCursor(1,1);
  }
  else{
    lcd.setCursor(0,1);
  }
  lcd.print(gz);
  lcd.print("g");
}

void setup() {
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
  lcd.print("initializing");
  delay(1000);
}

void loop() {
  delay(10); //to stop the screen resetting constantly

  //read in button press as HIGH (normally OPEN switch)
  pinMode(menuPin_in, INPUT_PULLUP);
  pinMode(actionPin_in, INPUT_PULLUP);
  pinMode(calibrationPin_in, INPUT_PULLUP);
  
  // CALIBRATION BUTTON (CAN ACTIVATE INDEPENDENT OF MENU STATE)
  if (digitalRead(calibrationPin_in) == LOW){
    calibrate();
    calibrationComplete = true;
  }

  //ACTION BUTTON
  if (digitalRead(actionPin_in) == LOW){
    digitalWrite(actionPin_out, HIGH);
  }
  else digitalWrite(actionPin_out, LOW);

  //MENU BUTTON
  if (digitalRead(menuPin_in) == LOW){  
    menuState += 1;
    if (menuState > 4){
      menuState = 0;}
    delay(500);
  }
  
  if(calibrationComplete){
    int stepCount = CountSteps();
  }
  
  getXYZg(gx,gy,gz);

  //switch states on button press
  switch(menuState){
    case 0:
      actionPin_out = 2;
      homeScreenDisplay(); break;
    case 1:
      actionPin_out = 8;
      selfTestDisplay(); break;
    case 2:
      actionPin_out = 2;
      steptrackerDisplay(stepCount); break;
    case 3:
      actionPin_out = 2;
      walkingpaceDisplay(pace); break;
    case 4: // ************** FOR TESTING **************
      actionPin_out = 2;
      printXYZg(gx,gy,gz); break;
  }
}


