//buttons 
const byte up = 2;
const int sizeIn = 1;
int input[sizeIn];
//-----------------------------------------Setup-------------------------------------------//
void setup() {
  pinMode(up, INPUT_PULLUP);
 
  Serial.begin(4800);
}

//---------------------------------------Loop------------------------------------------//
void loop() {
  input[0] = digitalRead(up);
  input[1] = digitalRead(up);
  for(int i = 0; i < sizeIn-1; i++)
  {
    Serial.print(input[i]);Serial.print(" ");
  }
  Serial.println(input[sizeIn-1]);
}
