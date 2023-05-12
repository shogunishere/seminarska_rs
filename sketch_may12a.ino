#include<Wire.h>
#include<Ticker.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// Naslov MPU9250 na I2C vodilu
#define MPU_ADD 104
// Naslov registra za pospesek 
#define ACC_MEAS_REG 59
// luci
#define I2C_ADD_IO1 32

#define RATE 10

#define BLYNK_TEMPLATE_ID "TMPL4mr4Ly93g"
#define BLYNK_TEMPLATE_NAME "Seminarska RS"
#define BLYNK_AUTH_TOKEN "LkKa02a7HaZQ1m5ZUjLaiNwq6TXqaLhV"
#define STEP_THRESHOLD 1.2 // You may need to adjust this based on your testing

// WiFi credentials
char ssid[] = "tocka";
char pass[] = "koda12345";

// <-------------------- I2C  funkciji zacetek -------------------------->
void I2CWriteRegister(uint8_t I2CDevice, uint8_t RegAdress, uint8_t Value){
  // I2CDevice - Naslov I2C naprave
  // RegAddress - Naslov registra 
  // Value - Vrednost za vpisati v register
  
  Wire.beginTransmission(I2CDevice);
  // Napravi sporočimo naslov registra, s katerega želimo brati:
  Wire.write(RegAdress);
  // Posljemo vrednost
  Wire.write(Value);
  Wire.endTransmission();
}

void I2CReadRegister(uint8_t I2CDevice, uint8_t RegAdress, uint8_t NBytes, uint8_t *Value){
  Wire.beginTransmission(I2CDevice);
  // Napravi sporočimo naslov registra, s katerega želimo brati:
  Wire.write(RegAdress);
  // Končamo prenos:
  Wire.endTransmission();
  
  // Napravi sporočimo, da želimo prebrati določeno število 8-bitnih registrov:
  Wire.requestFrom(I2CDevice, NBytes);
  for (int q = 0; q < NBytes; q++) {
    // Preberemo naslednji 8-bitni register oz. naslednji bajt:
    *Value = (uint8_t)Wire.read();
    Value++;
    //uint32_t vrednost = Wire.read();
  }

}

// <-------------------- I2C  Funkcije konec -------------------------->


// Meritve sa senzorja 
uint8_t accMeas[] = {0,0,0,0,0,0};

Ticker readSensor, leds;

// Meritve 
float accX = 0; 
float accY = 0; 
float accZ = 0; 

uint32_t steps = 0; // This will hold the number of steps

float previousAccMag = 0.0; 
bool isStep = false; 

void MPU9250_init(){
  // Resetiraj MPU9250 senzora => Register PWR_MGMT_1 (107)
  I2CWriteRegister(MPU_ADD,107,128); // 128 = 1000 0000
  // Pocakaj
  delay(500);
  // Preveri ID od senzora => Register WHO_AM_I (117) 
  uint8_t ID;
  I2CReadRegister(MPU_ADD,117,1,&ID);
  Serial.println("ID:");
  Serial.println(ID, HEX);
  // Gyroscope Conf => Register GYRO_CONFIG (27) 
  // 4 in 3 bit dolocata obseg 
  I2CWriteRegister(MPU_ADD,27,0); // 
  delay(100);
  // Accelerator Conf => Register ACCEL_CONFIG (28)
  // 4 in 3 bit dolocata obseg 
  // Opciono => Register ACCEL_CONFIG_2 (29)
  I2CWriteRegister(MPU_ADD,28,0); // 
  delay(100);
}

void readAcc(){
  static uint32_t count = 0;
  int32_t tmp;
  // TODO
  I2CReadRegister(MPU_ADD,ACC_MEAS_REG,6,accMeas);

  tmp = (((int8_t)accMeas[0] << 8) + (uint8_t)accMeas[1]);
  accX = tmp*1.0;

  tmp = (((int8_t)accMeas[2] << 8) + (uint8_t)accMeas[3]);
  accY = tmp*1.0;

  tmp = (((int8_t)accMeas[4] << 8) + (uint8_t)accMeas[5]);
  accZ = tmp*1.0;


  accX /= pow(2,14);

  accY /= pow(2,14);

  accZ /= pow(2,14);


  // Calculate acceleration magnitude (Euclidean distance)
  float accMag = sqrt(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2));
  
  // Check if a step has been made
  if (accMag > STEP_THRESHOLD && previousAccMag <= STEP_THRESHOLD) {
    if (!isStep) {
      isStep = true;
      steps++;

    }
  } else if (accMag < STEP_THRESHOLD) {
    isStep = false;
  }
  
  previousAccMag = accMag;

  // Print the results when the count is divisible by RATE
  if (count % RATE == 0)
  {
    Serial.print("ACC_X: X= ");
    Serial.print(accX);
    Serial.print(", ACC_Y= ");
    Serial.print(accY);
    Serial.print(", ACC_Z= ");
    Serial.println(accZ);

    // Send the acceleration data to Blynk app
    Blynk.virtualWrite(V0, accX);
    Blynk.virtualWrite(V1, accY);
    Blynk.virtualWrite(V2, accZ);
    
    // Send the step count to Blynk app
    Blynk.virtualWrite(V3, steps);
  }

  // Increment the counter
  count = count + 1;
}


void setup() {
  // put your setup code here, to run once:
  // Serijska komunikacija
  Serial.begin(115200);
    // I2C
  Wire.begin(12,14);
  // SDA - 12 pin
  // SCL - 14 pin
  Wire.setClock(100000);
  // Podesavanje senzorja 
  // https://github.com/bolderflight/mpu9250/blob/main/src/mpu9250.cpp
  MPU9250_init();
  
  // Kalibracija ?
  // calibrateGyro();
  
  // Branje senzorja
  readSensor.attach(0.1, readAcc);

  // Connect to Wi-Fi
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected to Wi-Fi");

  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();
}
