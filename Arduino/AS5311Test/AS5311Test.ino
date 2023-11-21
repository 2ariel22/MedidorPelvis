// 5mm 255 
void setup(){
  //start serial connection
  Serial.begin(9600);
  
  pinMode(18, INPUT);
  pinMode(6, INPUT);
  pinMode(6, INPUT);
  attachInterrupt(18, APulse, CHANGE);
  attachInterrupt(6 , BPulse, CHANGE);
}

// Linear distance in mm
volatile float distance = 0;
int CountPulsos = 0;


void APulse(){
  if (digitalRead(18) == digitalRead(6)){
    distance += 0.015;
    CountPulsos +=1;}
  else{
    distance -= 0.015; 
    CountPulsos -=1;
}}

void BPulse(){
  if (digitalRead(18) == digitalRead(6)){
    distance -= 0.015;
    CountPulsos -=1;}
  else{
    distance += 0.015; 
    CountPulsos +=1;
}}

void loop(){
  // Print out the distance value
  Serial.println("distancia: " + String(distance,2));
  Serial.println("pulsos: " + String(CountPulsos));
  delay(100);
}