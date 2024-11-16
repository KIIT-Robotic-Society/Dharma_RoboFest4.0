#define ADC D1  // Define ADC pin

void setup() {
  Serial.begin(115200);  // Corrected: Use Serial.begin to initialize serial communication
  pinMode(ADC, INPUT);   // Set ADC pin as input
}

void loop() {
  int adcValue = analogRead(ADC);  // Read analog value from ADC pin
  Serial.println(adcValue);          // Print the analog value to the Serial Monitor
  delay(1000);                      // Add a delay for better readability
}
