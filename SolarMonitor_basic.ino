#include <DHT_U.h>
#include <SimpleTimer.h>
#include <BlynkSimpleEsp8266.h>

#define DHTPIN 2
#define DHTTYPE    DHT11     // DHT 21 (AM2301)

DHT_Unified dht(DHTPIN, DHTTYPE);

// inverter on voltage and cutoff voltage
float on_voltage = 12.0;
float off_voltage = 11.5;

// refresh interval
uint32_t delayMS = 1000;

int pinValue1; // inverter virtual pin

int Vsensor = A0; // voltage sensor 
int inverter = D2; // relay

// voltage sensor
float correctionfactor = 0; // adjust this for calibration
float vout = 0.0; 
float vin = 0.0; 

// two resistors 30k and 7.5k ohms
float R1 = 30000;  //   
float R2 = 7500; //  
int value = 0; 

char data[80];

SimpleTimer timer;

// Blynk token you received via email
char auth[] = "YOUR_BLYNK_AUTH_TOKEN";

// Your WiFi credentials
char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";


void setup() {
  Serial.begin(9600);
  pinMode(Vsensor, INPUT); 
  pinMode(inverter, OUTPUT);
  digitalWrite(inverter, HIGH);
  
  dht.begin();
  
  Serial.println(F("Solar Power Monitor"));
  
  Blynk.begin(auth, ssid, pass);
  Blynk.virtualWrite(V5, 0);

  
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  
  // Set delay between sensor readings
  timer.setInterval(delayMS, timerEvent);

  timerEvent();
}

void loop() {
  Blynk.run();
  timer.run();
}

void timerEvent()
{
  
  // Get temperature event and print its value.
  sensors_event_t event;

  // Virtual pin 1 (V1) has temperature value
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
    Blynk.virtualWrite(V1, "ERR");
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    
    sprintf(data, "%.f", event.temperature);
    Blynk.virtualWrite(V1, data); // send data to blynk
  }
  
  // Get humidity event and print its value.
  // Virtual pin 2 (V2) has humidity value
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
    Blynk.virtualWrite(V2, "ERR");
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));

    sprintf(data, "%.f", event.relative_humidity);
    Blynk.virtualWrite(V2, data); // send data to blynk
  }

  float vtot = 0.0;
  int loops = 10; // number of samples

  // loop multiple times and get average reading
  for (int i=0; i < loops; i++) {
    vtot = vtot + analogRead(Vsensor);
  }
  value = vtot/loops;

  // voltage calculation
  vout = (value * 3.3) / 1024.0; // 3.3V
  vin = vout / (R2/(R1+R2));
  
  vin = vin - correctionfactor; 
 
  Serial.print("Voltage: "); 
  Serial.print(vin, 4);
  Serial.println("V");

  // Virtual pin 0 (V0) has voltage value
  sprintf(data, "%.1f", vin);
  Blynk.virtualWrite(V0, data); // send voltage value to blynk


  if (vin < off_voltage && digitalRead(inverter) == LOW) {
    digitalWrite(inverter, HIGH); 
    Serial.println("TURNING RELAY OFF");
  }

  if (vin > on_voltage && digitalRead(inverter) == HIGH) {
    digitalWrite(inverter, LOW); 
    Serial.println("TURNING RELAY ON");
  }
 


  if (digitalRead(inverter) == LOW) {
    Blynk.virtualWrite(V5, 255);
    Serial.println("Inverter is ON");
  } else {
    Blynk.virtualWrite(V5, 0);
    Serial.println("Inverter is OFF");
  }

  Serial.println("------------------");
  Serial.println("");
}
