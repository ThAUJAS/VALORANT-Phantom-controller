//color definition
int red[3] = {255,0,0};
int green[3] = {0,255,0};
int blue[3] = {0,0,255};
int purple[3] = {255,0,255};
int sky[3] = {0,255,255};
int white[3] = {255,255,255};
int black[3] = {0,0,0};

//--------------------------------Variable definition---------------------------------//
const int sizeIn = 14;
int countLED = 0;
int anval = 0;
int dival = 0;
int skill_1 = 1;
int skill_2 = 1;
int ult = 1;
int skill_3 = 1;

//-------------------------------------Setup------------------------------------------//
void setup() {
  for(int j = 2; j <= 12; j++){
    pinMode(j, INPUT_PULLUP);
  }
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, OUTPUT);
  pinMode(A6, INPUT);
  pinMode(A3, OUTPUT);
  Serial.begin(9600);
}

//-------------------------------------Loop------------------------------------------//
void loop() {
  anval = analogRead(A6);
  dival = digitalRead(A0);
  //depending on the potentiometer value, when the button is pressed, only one skill will be activitated
  if(dival == 0){
    if(anval>=113 && anval<= 200 && skill_2 != 0 && ult != 0 && skill_3 != 0){
      skill_1 = 0;
    }else if(anval>200 && anval<=500 && skill_1 != 0 && ult != 0 && skill_3 != 0){
      skill_2 = 0;
    }else if(anval>500 && anval<=800 && skill_1 != 0 && ult != 0 && skill_2 != 0){
      skill_3 = 0;
    }else if(anval>800 && anval<=900 && skill_1 != 0 && skill_2 != 0 && skill_3 != 0){
      ult = 0;
    }
  }else{
    skill_1 = 1; skill_2 = 1; skill_3 = 1; ult = 1;
  }
  //LED coloring as function of the skill
  if(anval>=113 && anval<= 200){LedColor(sky);}
  else if(anval>200 && anval<=500){LedColor(purple);}
  else if(anval>500 && anval<=800){LedColor(red);}
  else if(anval>800 && anval<=900){LedColor(blue);}

  //sending the data by serial comm that will be taken by the python code
  for(int i = 2; i < 11; i++){
    Serial.print(digitalRead(i)); Serial.print(" ");
  }  
    Serial.print(analogRead(A6)); Serial.print(" ");Serial.print(digitalRead(A1)); Serial.print(" ");Serial.print(skill_1); Serial.print(" ");Serial.print(skill_2); Serial.print(" ");Serial.print(skill_3); Serial.print(" ");Serial.println(ult);
}

//function that changes the color of the LED according to the RGB input
void LedColor(int colors[]){
  analogWrite(A2, colors[0]);
  analogWrite(A6, colors[1]);
  analogWrite(A3, colors[2]);
}
