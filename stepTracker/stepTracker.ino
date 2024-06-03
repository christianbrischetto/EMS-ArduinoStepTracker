#include <LiquidCrystal_I2C.h>
#include <PL_ADXL355.h>
#include <Stream.h>

LiquidCrystal_I2C lcd(0x27,16,2); //set lcd address and size

//init variables
const int calibrationPin_in = 5;    //pullup
const int actionPin_in = 6;         //pullup 
const int menuPin_in = 7;           //pullup 
int actionPin_out = 2;              //CHANGING output pin, 2 is a no action state
int menuState = 0;                  //variable for menu cycle
int numAverages = 50;               //for accel value filtering
bool calibrationComplete = false;   //global bool to trigger step counting
int stepCount;                      //current count of steps
int eventCount;                     //number of steps taken within 5 seconds, used in pace tracking function
unsigned long timeLast = 0;         //timer used in pace calculation
float pace;                         //walking pace
double gx;                          //adxl output in terms of gravity
double gy;                          //""
double gz;                          //""
double ref_x;                       //initial adxl value as an analog input (0-1023)
double ref_y;                       //""
double ref_z;                       //""
int mV_x;                           //accelerometer value in mV for self test
int mV_y;                           //""
int mV_z;                           //""
double adxlVec;                     //total magnitude of the accelerometer output
bool activeStep;                    //bool used to enable an active step


//*************************************************************************************************************
double gain = 1; // 1.2 for PCB
// 2V / 3.6V power supply = 195mV/g / 360mV/g output sensitivity
// 3.3V/3.6V * 0.360mV 
//                    = 0.33V/g
double outputSensitivity = 0.33/5 * gain * 1023;
//*************************************************************************************************************


void homeScreenDisplay(){
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(4,0);   //Set cursor to character 2 on line 0
  lcd.print("Tuesday");
  lcd.setCursor(3,1);
  lcd.print("Group One");
}

void selfTestDisplay(){
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(1,0);  
  lcd.print("ST (mV)");

  lcd.setCursor(9,0);
  lcd.print("x");
  if(mV_x >= 0){
    lcd.setCursor(11,0);
  }
  else{
    lcd.setCursor(10,0);
  }
  lcd.print(mV_x);

  lcd.setCursor(0,1);
  lcd.print("y");
  if(mV_y >= 0){
    lcd.setCursor(2,1);
  }
  else{
    lcd.setCursor(1,1);
  }
  lcd.print(mV_y);

  lcd.setCursor(9,1);
  lcd.print("z");
  if(mV_z >= 0){
    lcd.setCursor(11,1);
  }
  else{
    lcd.setCursor(10,1);
  }
  lcd.print(mV_z);
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
  adxlVec = sqrt(gx*gx + gy*gy + gz*gz);
  if(!activeStep){
    if(adxlVec > 1.2){
      stepCount++;
      eventCount++;
      activeStep = true;
    }
  }

  if(activeStep){
    if(adxlVec < 1){
      activeStep = false;
    }
  }

  pace = TrackPace();
  return (stepCount);
}

float TrackPace() { 
  unsigned long timeCurrent = millis();
  unsigned long timeElapsed = timeCurrent - timeLast;

  if (timeElapsed >= 5000) {
    pace = (float)eventCount * 5000 / timeElapsed;

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

void calibrationDisplay(){
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0,0);  
  lcd.print("Calib");

  lcd.setCursor(9,0);
  lcd.print("x");
  if(gx >= 0){
    lcd.setCursor(11,0);
  }
  else{
    lcd.setCursor(10,0);
  }
  lcd.print(gx);
  lcd.print("g");

  lcd.setCursor(0,1);
  lcd.print("y");
  if(gy >= 0){
    lcd.setCursor(2,1);
  }
  else{
    lcd.setCursor(1,1);
  }
  lcd.print(gy);
  lcd.print("g");

  lcd.setCursor(9,1);
  lcd.print("z");
  if(gz >= 0){
    lcd.setCursor(11,1);
  }
  else{
    lcd.setCursor(10,1);
  }
  lcd.print(gz);
  lcd.print("g");
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

  mV_x = ((currentx - ref_x)/1023)*5000;
  mV_y = ((currenty - ref_x)/1023)*5000;
  mV_z = ((currentz - ref_x)/1023)*5000;
}

void setup() {
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
  lcd.print("initializing");
  delay(1000);

  Serial.begin(115200);
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
  
  if(calibrationComplete){
    int stepCount = CountSteps();
  }

  // trigger SelfTest routine
  if ((menuState == 1) && (digitalRead(actionPin_in) == LOW)){
    digitalWrite(4, LOW);
  }
  else{
    digitalWrite(4, HIGH);
  }
  
  getXYZg(gx,gy,gz);

  // // for testing purposes only
  // Serial.println(adxlVec);

  //MENU BUTTON
    if (digitalRead(menuPin_in) == LOW){  
      menuState += 1;
      if (menuState > 4){
        menuState = 0;}
      delay(500);
    }

  //switch states on button press
  switch(menuState){
    case 0:
      actionPin_out = 2;
      homeScreenDisplay(); break;
    case 1:
      // actionPin_out = 4;
      selfTestDisplay(); break;
    case 2:
      actionPin_out = 2;
      steptrackerDisplay(stepCount); break;
    case 3:
      actionPin_out = 2;
      walkingpaceDisplay(pace); break;
    case 4:
      actionPin_out = 2;
      calibrationDisplay(); break;
  }
}


