#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET -1 // Cambiar a -1 si no se utiliza el pin de reset

#define outputA 5
#define outputB 3

int counter = 0;
int aState;
int aLastState;

bool auxHorario=false, auxAntihorario=false;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int flecha = 16;

void setup() {
  Serial.begin(115200);

    pinMode (outputA,INPUT);
       pinMode (outputB,INPUT);
       aLastState = digitalRead(outputA);   //Leemos el valor incial

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("No se ha encontrado la pantalla OLED"));
    for (;;);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Hola"); 

  display.display(); // Mostrar en la pantalla
  delay(2000); // Espera 2 segundos
}

void loop() {

  
aState = digitalRead(outputA);
if (aState != aLastState)
   {
      if (digitalRead(outputB) != aState) {
           
           if(!auxHorario){
              
              Serial.println("Horario");
              flecha+=16;
              auxHorario = true;
           }
           else{
            auxHorario=false;
            
           
           }
            delay(400);
           
           }
      else{
        
           if(!auxAntihorario){
              Serial.println("antioHorario");
              flecha-=16;
              auxAntihorario = true;
           }
           else{
            auxAntihorario=false;
            
           }
           delay(400);
           
           }

    if(flecha>48){
      flecha=48;
    }
    else if(flecha<16){
      flecha=16;
    }
     Mostrar(flecha);
     
   }
aLastState = aState; // Guardamos el ultimo valor
 delay(400);
 
}


void Mostrar(int y){

  display.clearDisplay();
  display.setCursor(0, 16);
  display.println("Opcion 1");
  display.setCursor(0, 32);
  display.println("Opcion 2");
  display.setCursor(0, 48);
  display.println("Opcion 3");

  display.setCursor(50, y);
  display.println("<=");

  display.display();
}
void Enter(){
  display.clearDisplay();
  display.setCursor(10, 32);
  display.println("wen dia :3");

  display.display();

}
