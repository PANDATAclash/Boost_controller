#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <PID_v1.h>

// Pin Definitions  
#define SOLENOID_PIN 5  
#define MAP_SENSOR_PIN A0  
#define POTENTIOMETER_PIN A1  

// TFT Display  
Adafruit_ST7735 tft = Adafruit_ST7735(10, 9, 8); // CS, DC, RESET pins

// PID Variables  
double setpoint, input, output;
double Kp = 2.5, Ki = 4.0, Kd = 1.2;  // Fine-tuned PID values
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Pressure & Safety Settings  
const float MAP_SENSOR_MIN_VOLTAGE = 0.2;  
const float MAP_SENSOR_MAX_VOLTAGE = 4.9;  
const float MAP_SENSOR_MIN_PRESSURE = 20;    
const float MAP_SENSOR_MAX_PRESSURE = 250;   
float overboostThreshold = 240;  // Dynamically set (Setpoint + 0.2 bar)
const float MAX_PRESSURE_LIMIT = 250;        
bool solenoidActive = false;
bool overboostShutdown = false;

// Faster Screen Update Timing  
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 100; 

void setup() {
    pinMode(SOLENOID_PIN, OUTPUT);
    Serial.begin(115200);  // High-speed Serial Communication ✅ FIXED

    // Initialize TFT  
    tft.initR(INITR_BLACKTAB);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);

    myPID.SetMode(AUTOMATIC);
    myPID.SetTunings(Kp, Ki, Kd);

    // ✅ Initialize display with default values
    displayStatus(0.0, 0.0, 0, 0.0, 0.0);
}

void loop() {
    static float lastPressure = -1;
    static float lastOutput = -1;
    static float lastSetpoint = -1;

    // Read MAP Sensor  
    int sensorValue = analogRead(MAP_SENSOR_PIN);
    float voltage = sensorValue * (5.0 / 1023.0);
    
    // 🚨 MAP Sensor Failsafe (If voltage is below 0.2V, disable boost control)
    if (voltage < MAP_SENSOR_MIN_VOLTAGE) {
        Serial.println("⚠ WARNING: MAP SENSOR FAILURE! DISABLING BOOST CONTROL.");
        digitalWrite(SOLENOID_PIN, LOW);
        displaySensorError();
        return;
    }
    
    input = ((voltage - MAP_SENSOR_MIN_VOLTAGE) / (MAP_SENSOR_MAX_VOLTAGE - MAP_SENSOR_MIN_VOLTAGE)) * 
            (MAP_SENSOR_MAX_PRESSURE - MAP_SENSOR_MIN_PRESSURE) + MAP_SENSOR_MIN_PRESSURE;
    float pressureBar = input / 100.0;  

    // Read Potentiometer for Desired Pressure  
    int potValue = analogRead(POTENTIOMETER_PIN);
    setpoint = map(potValue, 0, 1023, MAP_SENSOR_MIN_PRESSURE, MAX_PRESSURE_LIMIT);
    float setpointBar = setpoint / 100.0;  

    // 🚨 Set Overboost Limit Dynamically (Setpoint + 0.2 bar)
    overboostThreshold = setpoint + 20;  // 20 kPa = 0.2 bar
    float overboostBar = overboostThreshold / 100.0;

    // 🚨 Overboost Protection with Soft Boost Cut  
    if (input > overboostThreshold) {
        overboostShutdown = true;  
        solenoidActive = false;  
        analogWrite(SOLENOID_PIN, 127);  // Set solenoid to 50% instead of shutting off completely
        displayOverboostWarning();  
        printSerialData(pressureBar, setpointBar, 50, voltage, overboostBar);
        return; 
    }

    // Reset overboost protection  
    if (overboostShutdown && input < (overboostThreshold - 10)) {
        overboostShutdown = false;
    }

    // 🚀 Normal PID-Based Boost Control  
    if (!overboostShutdown) {
        myPID.Compute();
        output = constrain(output, 0, 255); 
        
        if (input < setpoint) {
            analogWrite(SOLENOID_PIN, (int)output);
            solenoidActive = (output > 0);
        } else {
            digitalWrite(SOLENOID_PIN, LOW);  
            solenoidActive = false;
        }
    }

    // Convert duty cycle to %  
    int dutyCyclePercent = (output / 255.0) * 100;

    // Faster Display Update  
    if (millis() - lastDisplayUpdate >= displayUpdateInterval || 
        input != lastPressure || output != lastOutput || setpoint != lastSetpoint) {
        
        displayStatus(pressureBar, setpointBar, dutyCyclePercent, voltage, overboostBar);
        printSerialData(pressureBar, setpointBar, dutyCyclePercent, voltage, overboostBar);
        lastPressure = input;
        lastOutput = output;
        lastSetpoint = setpoint;
        lastDisplayUpdate = millis();
    }
}

// 📢 **Display on TFT**  
void displayStatus(float pressureBar, float setpointBar, int dutyCyclePercent, float sensorVoltage, float obLimit) {
    tft.fillRect(0, 10, 128, 100, ST77XX_BLACK); 

    tft.setCursor(10, 10);
    tft.print("Setpoint: ");
    tft.print(setpointBar, 2);
    tft.print(" bar");

    tft.setCursor(10, 30);
    tft.print("Pressure: ");
    tft.print(pressureBar, 2);
    tft.print(" bar");

    tft.setCursor(10, 50);
    tft.print("Duty: ");
    tft.print(dutyCyclePercent);
    tft.print("%");

    tft.setCursor(10, 70);
    tft.print("OB Limit: ");
    tft.print(obLimit, 2);
    tft.print(" bar");

    tft.setCursor(10, 90);
    tft.print("Sensor V: ");
    tft.print(sensorVoltage, 2);
    tft.print("V");
}

// 🚨 **Overboost Warning**  
void displayOverboostWarning() {
    tft.fillScreen(ST77XX_RED);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(10, 20);
    tft.setTextSize(2);
    tft.print("OVERBOOST!");
    tft.setCursor(10, 50);
    tft.setTextSize(1);
    tft.print("Boost Cut Active!");
}

// 🚨 **MAP Sensor Error**  
void displaySensorError() {
    tft.fillScreen(ST77XX_RED);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(10, 20);
    tft.setTextSize(2);
    tft.print("SENSOR FAIL!");
}

// 📡 **Serial Monitor Output (Readable & Clear)**  
void printSerialData(float pressureBar, float setpointBar, int dutyCyclePercent, float sensorVoltage, float obLimit) {
    Serial.println("==================================");
    Serial.print("Setpoint  : "); Serial.print(setpointBar, 2); Serial.println(" bar");
    Serial.print("Pressure  : "); Serial.print(pressureBar, 2); Serial.println(" bar");
    Serial.print("Duty Cycle: "); Serial.print(dutyCyclePercent); Serial.println("%");
    Serial.print("OB Limit  : "); Serial.print(obLimit, 2); Serial.println(" bar");
    Serial.print("Sensor V  : "); Serial.print(sensorVoltage, 2); Serial.println("V");
    Serial.println("==================================\n");
}
