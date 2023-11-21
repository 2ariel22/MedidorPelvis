
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

MPU6050 mpu;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define OFFSETS  -1474, 28, 436, 315, -103, 16

float gyroX_prev = 0; // Variable para almacenar la estimación previa del giroscopio X
float gyroY_prev = 0; // Variable para almacenar la estimación previa del giroscopio Y
float Q = 0.01;       // Proceso de covarianza
float R = 0.1;        // Medida de covarianza



#define outputA 10
#define outputB 3
#define interruptPin 2 // Pin del botón



int flecha = 0;

int counter = 0;
int aState;
int aLastState;

bool auxHorario = false, auxAntihorario = false;
bool botonPresionado = false;

bool aux20 = false;

void setup() {
  pinMode(outputA, INPUT);
  pinMode(outputB, INPUT);
  pinMode(interruptPin, INPUT_PULLUP); // Configurar el pin del botón como entrada con resistencia pull-up

  attachInterrupt(digitalPinToInterrupt(interruptPin), funcionBotonPresionado, FALLING); // Asociar la función de interrupción al pin 3 por cualquier cambio
  
  
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  
  mpu.setXAccelOffset(4294965820);
  mpu.setYAccelOffset(58);
  mpu.setZAccelOffset(436);
  mpu.setXGyroOffset(315);
  mpu.setYGyroOffset(4294967190);
  mpu.setZGyroOffset(16);


  Serial.begin(9600);
  aLastState = digitalRead(outputA); // Leemos el valor inicial del pin A

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(0.5);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Hola");
  display.display(); // Mostrar en la pantalla
  delay(1000); // Espera 1 segundo
}

void loop() {
  if(aux20){
    
  aState = digitalRead(outputA);
  if (aState != aLastState) {
    // Verificar el estado del botón si ha cambiado
    if (digitalRead(interruptPin) == LOW) {
      // Si se detecta que el botón está presionado
      if (!botonPresionado) {
        // Aquí llamas a la función que debe ejecutarse al presionar el botón
        funcionBotonPresionado();
        botonPresionado = true;
        aux20 = !aux20;
      }
    } else {
      botonPresionado = false;
    }

    // Lógica de movimiento de flecha
    int bState = digitalRead(outputB);
    if (bState != aState) {
      if (!auxHorario) {
        Serial.println("Horario");
        flecha += 10;
        if (flecha > 20) {
          flecha = 20;
        } else if (flecha < 0) {
          flecha = 0;
        }
        auxHorario = true;
        Mostrar(flecha);
      } else {
        auxHorario = false;
      }
    } else {
      if (!auxAntihorario) {
        Serial.println("antioHorario");
        flecha -= 10;
        if (flecha > 20) {
          flecha = 20;
        } else if (flecha < 0) {
          flecha = 0;
        }
        auxAntihorario = true;
        Mostrar(flecha);
      } else {
        auxAntihorario = false;
      }
    }
    
    aLastState = aState; // Guardamos el último valor del pin A
  }
}

else{

int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;

  mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
  Serial.print("Gyro: ");
  Serial.print(gyroX);
  Serial.print(", ");
  Serial.print(gyroY);
  Serial.println();

  display.clearDisplay();

  drawCartesianLines(); // Dibujar el plano cartesiano

  float filteredGyroX = kalmanFilter(gyroX, gyroX_prev, 0.01);
  float filteredGyroY = kalmanFilter(gyroY, gyroY_prev, 0.01);

  gyroX_prev = filteredGyroX;
  gyroY_prev = filteredGyroY;

  int ballSize = 5; // Tamaño de la bola
  int ballX = map(filteredGyroX, -500, 500, 0, SCREEN_WIDTH - ballSize);
  int ballY = map(filteredGyroY, -500, 500, 0, SCREEN_HEIGHT - ballSize);

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
  display.display();

  delay(100); // Ajusta el retardo según sea necesario para controlar la frecuencia de salida de datos


}


}

void funcionBotonPresionado() {
  aux20=!aux20;
}

void Mostrar(int y) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Opcion 1");
  display.setCursor(0, 10);
  display.println("Opcion 2");
  display.setCursor(0, 20);
  display.println("Opcion 3");
  display.setCursor(50, y);
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
  display.drawFastHLine(0, SCREEN_HEIGHT / 2, SCREEN_WIDTH, SSD1306_WHITE); // Línea horizontal central
  display.drawFastVLine(SCREEN_WIDTH / 2, 0, SCREEN_HEIGHT, SSD1306_WHITE); // Línea vertical central
  display.display();
}
