#include<Wire.h>
#include<Ticker.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <PubSubClient.h>

// Naslov MPU9250 na I2C vodilu
#define MPU_ADD 104
// Naslov registra za pospesek 
#define ACC_MEAS_REG 59
// luci
#define I2C_ADD_IO1 32

#define RATE 40
#define WINDOW_SIZE 50
#define STEP_THRESHOLD 0.1 // 0.07
#define CAL 100.0f

#define BLYNK_TEMPLATE_ID "TMPL4mr4Ly93g"
#define BLYNK_TEMPLATE_NAME "Seminarska RS"
#define BLYNK_AUTH_TOKEN "LkKa02a7HaZQ1m5ZUjLaiNwq6TXqaLhV"

// WiFi credentials
char ssid[] = "tocka";
char password[] = "koda12345";


// MQQT
const char* mqtt_server = "broker.mqtt-dashboard.com";
const char* topic = "acceleration";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(61)
char msg[MSG_BUFFER_SIZE];
int value = 0;

// Meritve sa senzorja 
uint8_t accMeas[] = {0,0,0,0,0,0};

Ticker readSensor, leds;

// Mere in cilji
int32_t steps_goal = 6000;
int32_t calorie_goal = 2000;
int32_t calorie_counter = 0;
int32_t weight = 75;
int32_t height = 180;
float totaldistance = 0;
uint32_t totalsteps = 0;
int32_t totalcalories = 0;

// Meritve 
float accX = 0; 
float accY = 0; 
float accZ = 0; 

// Kalibracija
float accX_cal = 0;
float accY_cal = 0;
float accZ_cal = 0;

// Bufferji 
float accXBuffer[WINDOW_SIZE] = {0};
float accYBuffer[WINDOW_SIZE] = {0};
float accZBuffer[WINDOW_SIZE] = {0};

// Zadnje meritve
float lastAccX = 0;
float lastAccY = 0;
float lastAccZ = 0;

// Minimumi
float minAccX, minAccY, minAccZ, maxAccX, maxAccY, maxAccZ;

// Dynamic thresholdi
float dynamicThresholdX, dynamicThresholdY, dynamicThresholdZ;

int bufferIndex = 0;
uint32_t steps = 0; // This will hold the number of steps

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

void calibration() {

  int32_t table_x = 0;
  int32_t table_y = 0;
  int32_t table_z = 0;

  for (int i = 0; i < 50; i++) {

    Wire.beginTransmission(MPU_ADD);
    Wire.write(ACC_MEAS_REG);
    Wire.endTransmission();

    Wire.requestFrom(MPU_ADD, 6);

    table_x = (uint8_t) Wire.read();
    table_x = table_x << 8;
    table_x += (uint8_t) Wire.read();
    accX_cal += table_x / (128*128);
    table_y = (uint8_t) Wire.read();
    table_y = table_y << 8;
    table_y += (uint8_t) Wire.read();
    accY_cal += table_y / (128*128);
    table_z = (uint8_t) Wire.read();
    table_z = table_z << 8;
    table_z += (uint8_t) Wire.read();
    accZ_cal += table_z / (128*128);

    //delay(1000 / RATE);
  }

  accX_cal /= CAL;
  accY_cal /= CAL;
  accZ_cal /= CAL;

  Serial.println("Skalibriran!");
}

void readAcc(){
  static uint32_t count = 0;
  int32_t tmp;

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

  // Dodajanje novih meritev v buffer
  accXBuffer[bufferIndex] = accX;
  accYBuffer[bufferIndex] = accY;
  accZBuffer[bufferIndex] = accZ;

  // Izracun razlik
  float diffX = abs(accX - lastAccX);
  float diffY = abs(accY - lastAccY);
  float diffZ = abs(accZ - lastAccZ);

  // Determine which axis had the biggest change
  if (diffX > diffY && diffX > diffZ) {
    // X axis had the biggest change
    dynamicThresholdX = (maxAccX + minAccX) / 2;

    // Preckamo dynamic threshold
    if (accX < dynamicThresholdX && lastAccX > dynamicThresholdX && diffX > STEP_THRESHOLD) {
      steps++;
    }

    // Zapomni zadnjo meritev
    lastAccX = accX;
    maxAccX = -99999;
    minAccX = 99999;
  } else if (diffY > diffX && diffY > diffZ) {
    // Y axis had the biggest change
    dynamicThresholdY = (maxAccY + minAccY) / 2;

    // Preckamo dynamic threshold
    if (accY < dynamicThresholdY && lastAccY > dynamicThresholdY && diffY > STEP_THRESHOLD) {
      steps++;
    }

    // Zapomni zadnjo meritev
    lastAccY = accY;
    maxAccY = -99999;
    minAccY = 99999;

  } else {
    // Z axis had the biggest change
    dynamicThresholdZ = (maxAccZ + minAccZ) / 2;

    // Preckamo dynamic threshold
    if (accZ < dynamicThresholdZ && lastAccZ > dynamicThresholdZ && diffZ > STEP_THRESHOLD) {
      steps++;
    }

    // Zapomni zadnjo meritev
    lastAccZ = accZ;
    maxAccZ = -99999;
    minAccZ = 99999;
  }

  // Increment the buffer index, wrapping around to the start if necessary
  bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;

  // Print the results when the count is divisible by RATE
  if (count % RATE == 0)
  {
    // Računanje hitrosti

    float speed = get_stride(steps) * steps / 2;

    // Računanje razdalje

    float distance = get_stride(steps) * steps;

    // Računanje kalorij

    float calories = speed * weight / 400;

    // Hoja ali tek?

    int activity;

    if (steps > 5) {
      activity = 2;
    }
    else if (steps > 0) {
      activity = 1;
    }
    else {
      activity = 0;
    }

    totalsteps += steps;
    totalcalories += calories;
    totaldistance += distance;
    steps = 0;

    Serial.print("ACC_X: X= ");
    Serial.print(accX);
    Serial.print(", ACC_Y= ");
    Serial.print(accY);
    Serial.print(", ACC_Z= ");
    Serial.println(accZ);
    Serial.print("Total steps: ");
    Serial.println(totalsteps);
    Serial.print("Current speed: ");
    Serial.println(speed);
    Serial.print("Distance: ");
    Serial.println(totaldistance/100);
    Serial.print("Calories: ");
    Serial.println(totalcalories/100);
    Serial.print("Current activity: ");
    Serial.println(activity);

    // Send the acceleration data to Blynk app
    Blynk.virtualWrite(V0, accX);
    Blynk.virtualWrite(V1, accY);
    Blynk.virtualWrite(V2, accZ);
    
    // Send the step count to Blynk app
    Blynk.virtualWrite(V3, totalsteps);
    Blynk.virtualWrite(V5, totalcalories/100);
    Blynk.virtualWrite(V4, totaldistance/100);

    // MQTT
    // sends acceleration data as string stored in msg variable
    client.loop();

    snprintf (msg, MSG_BUFFER_SIZE, "%lu,%4.2f,%4.2f,%4.2f", millis() / 1000 , accX, accY, accZ);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish(topic, msg);

  }

  // Increment the counter
  count = count + 1;
}

float get_stride(int32_t n_steps) {
  if (n_steps == 1) {
    return height / 5;
  }
  else if (n_steps == 2) {
    return height / 4;
  }
  else if (n_steps == 3) {
    return height / 3;
  }
  else if (n_steps == 4) {
    return height / 2;
  }
  else if (n_steps == 5) {
    return height / 1.2;
  }
  else if (n_steps == 6 || n_steps == 7) {
    return height;
  }
  else if (n_steps > 7) {
    return height * 1.2;
  }
  return 1;
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  // Serijska komunikacija
  Serial.begin(115200);
  
  setup_wifi();

  client.setServer(mqtt_server, 1883);
  
  // I2C
  Wire.begin(12,14);

  // SDA - 12 pin
  // SCL - 14 pin
  Wire.setClock(100000);

  // Podesavanje senzorja 
  // https://github.com/bolderflight/mpu9250/blob/main/src/mpu9250.cpp
  MPU9250_init();

  // Kalibracija
  Serial.println("Kalibracija...");
  delay(5000);
  calibration();
  
  // Branje senzorja
  readSensor.attach(0.1, readAcc);
    
  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();

  if (!client.connected()) {
    reconnect();
  }
}