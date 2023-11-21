
#define OutputA =10;
#define OutputB =3;
#define Sw =2;

int StateA, LastStateA;

void setup() {
  pinMode(2, INPUT_PULLUP);
  Serial.begin(9600);

}

void loop() {
  StateA = digitalRead();
  Serial.println(digitalRead(2));
  delay(100);

}
