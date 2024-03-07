#include <LiquidCrystal_I2C.h>
#include <PL_ADXL355.h>

LiquidCrystal_I2C lcd(0x27,16,2); //set lcd address and size

//init variables
int menuPin_in = 7; //pullup 
int actionPin_in = 2; //pullup 
int actionPin_out = 4; //CHANGING output pin

int menuState = 0; //variable for menu cycle
int stepCount;
float pace;

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
}

int CountSteps(){ //count steps 
  //if accelerometer tick, step += 1
}

void steptrackerDisplay(int steps){
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(3,0);  
  lcd.print("Steps:  ");
  lcd.print(steps);
}

int TrackPace(){ //walking pace calculation
  //based on steps over time??
}

void walkingpaceDisplay(float pace){
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(3,0);  
  lcd.print("Pace:  ");
  lcd.print(pace);
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
    if (menuState > 3){
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
  }
}


