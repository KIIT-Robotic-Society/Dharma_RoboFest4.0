void setup() {

  Serial.begin(9600);
  pinMode(D10,INPUT_PULLUP);
  pinMode(D9,INPUT_PULLUP);
  pinMode(D8,INPUT_PULLUP);
}

void loop() {
  Serial.print("Button 1: ");
  Serial.println(digitalRead(D10));
  
  Serial.print("Button 2: ");
  Serial.println(digitalRead(D9));

  Serial.print("Button 3: ");
  Serial.println(digitalRead(D8));

  delay(500);



}
