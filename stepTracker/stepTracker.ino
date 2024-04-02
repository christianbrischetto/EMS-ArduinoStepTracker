#include <LiquidCrystal_I2C.h>
#include <PL_ADXL355.h>
#include <Stream.h>

LiquidCrystal_I2C lcd(0x27,16,2); //set lcd address and size

//init variables
int menuPin_in = 7; //pullup 
int actionPin_in = 2; //pullup 
int actionPin_out = 4; //CHANGING output pin

int menuState = 0; //variable for menu cycle
int stepCount;
float pace;
double gx;
double gy;
double gz;
double currentx;
double currenty;
double currentz;
double ref_x;
double ref_y;
double ref_z;

// 2V / 3.6V power supply = 195mV/g / 360mV/g output sensitivity
// 3.3V/3.6V * 0.360mV 
//                    = 0.33V/g
double outputSensitivity = 0.33/5 * 1023;


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
  lcd.print("Calibrate");
}

bool step(){
  // check if step is taken
  //return(true);
}

int CountSteps(){ //count steps 
  if (step()){
    stepCount += 1;
  }

  if (menuState == 2 && (digitalRead(actionPin_in) == LOW)){
    stepCount = 0;
  }
  return (stepCount);
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

int TrackPace(){ //walking pace calculation
  //based on steps over time??
  //action button cycles types of pace (steps per second? per minute? per hour?)
}

void walkingpaceDisplay(float pace){
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(3,0);  
  lcd.print("Pace:  ");
  lcd.print(pace);
}

void calibrate(){
  // self test
  // take self test values as xy 0g, z 1g
  ref_x = analogRead(A1); 
  ref_y = analogRead(A0); 
  ref_z = analogRead(A3); 

  // set ref value as g value
  printXYZg(0,0,1);
  
}

void getXYZg(double& gx, double& gy, double& gz){
  currentx = analogRead(A1);
  currenty = analogRead(A0);
  currentz = analogRead(A3);
  
  // 2V / 3.6V power supply = 195mV/g / 360mV/g output sensitivity
  // 3.3V/3.6V * 360mV 
  //                    = 330mV/g

  // get difference in current to reference voltage
  // scale as a percentage of the output sensitivity
  gx = (currentx - ref_x)/outputSensitivity; 
  gy = (currenty - ref_y)/outputSensitivity;  
  gz = (currentz - ref_z)/outputSensitivity;  
  
}

void printXYZg(double gx, double gy, double gz){
  lcd.clear();
  lcd.backlight();
  delay(100); //to stop the screen resetting constantly

  lcd.setCursor(0,0);  
  lcd.print(gx);
  lcd.print("g");

  lcd.setCursor(8,0);  
  lcd.print(gy);
  lcd.print("g");

  lcd.setCursor(0,1);  
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
  delay(100); //to stop the screen resetting constantly

  //read in button press as HIGH (normally OPEN switch)
  pinMode(menuPin_in, INPUT_PULLUP);
  pinMode(actionPin_in, INPUT_PULLUP);
  
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
  
  int stepCount = CountSteps();
  float pace = TrackPace();

  //switch states on button press
  switch(menuState){
    case 0:
      actionPin_out = 4;
      homeScreenDisplay(); break;
    case 1:
      actionPin_out = 4;
      selfTestDisplay(); break;
    case 2:
      actionPin_out = 5;
      steptrackerDisplay(stepCount); break;
    case 3:
      actionPin_out = 4;
      walkingpaceDisplay(pace); break;
    case 4:
      getXYZg(gx,gy,gz);
      printXYZg(gx,gy,gz); break;
  }
}


