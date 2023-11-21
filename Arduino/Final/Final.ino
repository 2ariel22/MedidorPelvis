#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//definir tamanio de la Pantalla oled
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

//Creamos un objeto pantalla Oled
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//pines del Encoder Selector
#define outputA 10
#define outputB 3
#define Sw 2

//Control de Encoder Selector
int aState;
int aLastState;
bool BotonEncoder = false;
bool auxHorario = false, auxAntihorario = false;

//Posicion de la flecha menu
int flechaY = 0, flechaX = 45;


//creamos un onjeto MPU6050
MPU6050 mpu;

// OFFSETS obtenidos de la calibracion previa del mpu6050
// OFFSETS -1474, 28, 436, 315, -103, 16

//variables para el filtro kalman
float gyroX_prev = 0;  // Variable para almacenar la estimación previa del giroscopio X
float gyroY_prev = 0;  // Variable para almacenar la estimación previa del giroscopio Y
float Q = 0.01;        // Proceso de covarianza
float R = 0.1;         // Medida de covarianza

// las variables X y Y corresponde a la posicion de la Flecha
void Menu(int y, int x) {
  display.clearDisplay();
  display.setTextSize(0.5);
  display.setCursor(0, 0);
  display.println("* Medir");
  display.setCursor(0, 10);
  display.println("* Datos");
  display.setCursor(0, 20);
  display.println("* Gif:0");
  display.setCursor(x, y);
  display.println("<=");
  display.display();
  
}

float kalmanFilter(float measurement, float prevEstimate, float covariances) {
  // Predicción
  float predictedEstimate = prevEstimate;
  float predictedCovariance = covariances + Q;

  // Actualización (corrección)
  float kalmanGain = predictedCovariance / (predictedCovariance + R);
  float updatedEstimate = predictedEstimate + kalmanGain * (measurement - predictedEstimate);
  float updatedCovariance = (1 - kalmanGain) * predictedCovariance;

  return updatedEstimate;
}


void drawCartesianLines() {
  // Dibujar líneas horizontales y verticales para crear un plano cartesiano
  display.drawFastHLine(0, SCREEN_HEIGHT / 2, SCREEN_WIDTH, SSD1306_WHITE);  // Línea horizontal central
  display.drawFastVLine(SCREEN_WIDTH / 2, 0, SCREEN_HEIGHT, SSD1306_WHITE);  // Línea vertical central
  display.display();
}

void Escribir(){
  
}

void MostrarGiroscopio(){
        //Serial.println("Giroscopio");
        int16_t accelX, accelY, accelZ;
        //int16_t gyroX, gyroY, gyroZ;

        //mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
        mpu.getAcceleration(&accelX, &accelY, &accelZ);
        
        float anguloX = (atan(accelX/sqrt(pow(accelY,2) + (pow(accelZ,2)))) * (180.0/3.14)) + 5;
        float anguloY = (atan(accelY/sqrt(pow(accelX,2) + (pow(accelZ,2)))) * (180.0/3.14));
        
        Serial.print("Gyro: ");
        Serial.print(anguloX);
        Serial.print(", ");
        Serial.print(anguloY);
        Serial.println();

        display.clearDisplay();

        drawCartesianLines();  // Dibujar el plano cartesiano
      /*
        float filteredGyroX = kalmanFilter(gyroX, gyroX_prev, 0.01);
        float filteredGyroY = kalmanFilter(gyroY, gyroY_prev, 0.01);
         Serial.print("Filtado: ");
        Serial.print(filteredGyroX);
        Serial.print(", ");
        Serial.print(filteredGyroY);
        Serial.println();
        gyroX_prev = filteredGyroX;
        gyroY_prev = filteredGyroY;
      */
        int ballSize = 4;  // Tamaño de la bola
        int ballX = map(anguloX, -25, 25, 0, SCREEN_WIDTH - ballSize);
        int ballY = map(anguloY, -25, 25, 0, SCREEN_HEIGHT - ballSize);

        if (ballX < 0) {
          ballX = 0;
        } else if (ballX > SCREEN_WIDTH - ballSize) {
          ballX = SCREEN_WIDTH - ballSize;
        }
        if (ballY < 0) {
          ballY = 0;
        } else if (ballY > SCREEN_HEIGHT - ballSize) {
          ballY = SCREEN_HEIGHT - ballSize;
        }

        display.fillCircle(ballX, ballY, ballSize, SSD1306_WHITE);
        display.setTextSize(0.3);
        display.setCursor(0, 0);
        display.println("x: "+ String(anguloX,2));
        display.setCursor(0, 8);
        display.println("y: "+ String(anguloY,2));
        display.display();
        delay(100);
      
}

void MostrarDatos(){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("*Le Muestra Datos*");
  display.display();
}

void ControlMenu(){
        
        aState = digitalRead(outputA);
        if (aState != aLastState) {
          int bState = digitalRead(outputB);
          if (bState != aState) {
            if (!auxHorario) {
              //Serial.println("Horario");

              flechaY += 10;
              if (flechaY > 20) {
                flechaY = 20;
              } else if (flechaY < 0) {
                flechaY = 0;
              }
              auxHorario = true;
              Menu(flechaY,flechaX);
            } else {
              auxHorario = false;
            }
          } else {
            if (!auxAntihorario) {
              Serial.println("antioHorario");
              flechaY -= 10;
              if (flechaY > 20) {
                flechaY = 20;
              } else if (flechaY < 0) {
                flechaY = 0;
              }
              auxAntihorario = true;
              Menu(flechaY,flechaX);
            } else {
              auxAntihorario = false;
            }
          }

          aLastState = aState;  // Guardamos el último valor del pin A
        }
}

void dibujarCajaDeRegalo() {
  // Limpia la pantalla
  display.clearDisplay();

  // Dibuja la base de la caja
  display.drawRect(10, 10, 108, 20, WHITE);

  // Dibuja las líneas que forman la tapa de la caja
  display.drawLine(10, 10, 64, 0, WHITE);
  display.drawLine(118, 10, 64, 0, WHITE);
  display.drawLine(10, 30, 64, 50, WHITE);
  display.drawLine(118, 30, 64, 50, WHITE);

  // Actualiza la pantalla
  display.display();
}

void setup() {
  Serial.begin(9600);
  pinMode(Sw, INPUT_PULLUP);

  aLastState = digitalRead(outputA);  // Leemos el valor inicial del pin A

  Wire.begin();
  mpu.initialize();
 
  //#define OFFSETS  -1608,      86,     330,     315,    -106,      14

  mpu.setXAccelOffset(-1608);
  mpu.setYAccelOffset(86);
  mpu.setZAccelOffset(330);
  mpu.setXGyroOffset(315);
  mpu.setYGyroOffset( -106);
  mpu.setZGyroOffset( 14);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(0.5);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Hola");
  display.display();  // Mostrar en la pantalla
  delay(1000);        // Espera 1 segundo
}

void loop() {
 if (BotonEncoder) {
    while (true) {
      bool Salida = digitalRead(Sw);
      if (Salida == 0) {
        delay(200);
        BotonEncoder = false;
        ControlMenu();
        break;
      }
      if(flechaY == 0){
        MostrarGiroscopio();
      }
      else if(flechaY == 10){
        MostrarDatos();
      }
      else if(flechaY == 20){
        dibujarCajaDeRegalo();
      }
      

    }

  }

  else {
    while (true) {
      bool Salida = digitalRead(Sw);
      if (Salida == 0) {
        delay(200);
        BotonEncoder = true;
        break;
      } else {
        ControlMenu();
      }
    }
  }
}