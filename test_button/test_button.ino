//buttons 
const byte up = 2;
const byte shoot = 3;
const int sizeIn = 2;
int input[sizeIn];
//-----------------------------------------Setup-------------------------------------------//
void setup() {
  pinMode(up, INPUT_PULLUP);
  pinMode(shoot, INPUT_PULLUP);
  Serial.begin(9600);
}

//---------------------------------------Loop------------------------------------------//
void loop() {
  input[0] = digitalRead(up);
  input[1] = digitalRead(shoot);
  for(int i = 0; i < sizeIn-1; i++)
  {
    Serial.print(input[i]);Serial.print(" ");
  }
  Serial.println(input[sizeIn-1]);
}
