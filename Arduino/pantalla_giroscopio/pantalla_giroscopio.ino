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


void setup() {
  Wire.begin();
  Serial.begin(115200);

  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  
  mpu.setXAccelOffset(4294965820);
  mpu.setYAccelOffset(58);
  mpu.setZAccelOffset(436);
  mpu.setXGyroOffset(315);
  mpu.setYGyroOffset(4294967190);
  mpu.setZGyroOffset(16);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Inicializa la pantalla OLED
  display.display(); // Borra la pantalla
  delay(2000); // Espera 2 segundos
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();

  Serial.println("MPU6050 inicializado para lecturas de sensores...");
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

void loop() {
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
