int M1 = 1;
int IR1 = 1;
int Detect = 1;  //ต้องจูน
void setup() {
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(IR1, INPUT);
}
void Belt() {
  while (1) {
    digitalWrite(M1, HIGH);
    if (analogRead(IR1) >= Detect) {
      break;
    }
  }
  digitalWrite(M1, LOW);
}
void loop() {
}
