#include <Wire.h>                  
#include <Adafruit_GFX.h>           
#include <Adafruit_SSD1306.h>       
#include <PID_v1.h>

// Pin Definitions  
#define SOLENOID_PIN 5  
#define MAP_SENSOR_PIN A0  
#define POTENTIOMETER_PIN A1  

// OLED Display (I2C, address 0x3C for 128x32 OLED)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); 

// PID Variables  
double setpoint, input, output;
double Kp = 2.5, Ki = 4.0, Kd = 1.2;  // Tuned PID values
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Pressure & Safety Settings  
const float MAP_SENSOR_MIN_VOLTAGE = 0.2;  
const float MAP_SENSOR_MAX_VOLTAGE = 4.9;  
const float MAP_SENSOR_MIN_PRESSURE = 20;    
const float MAP_SENSOR_MAX_PRESSURE = 250;   
float overboostThreshold = 230;  // Setpoint + 0.3 bar (30 kPa)
const float MAX_PRESSURE_LIMIT = 250;        
bool solenoidActive = false;
bool overboostShutdown = false;

// Hysteresis to prevent rapid solenoid toggling  
const float hysteresis = 10.0;  // 10 kPa (0.1 bar)  

// Low-pass filter settings  
const float filterAlpha = 0.1;  
float filteredPressure = 0;

// Faster Screen Update Timing  
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 100; 

void setup() {
    pinMode(SOLENOID_PIN, OUTPUT);
    Serial.begin(115200);  

    // Initialize OLED  
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    display.display();   
    delay(2000);         

    display.clearDisplay();  

    myPID.SetMode(AUTOMATIC);
    myPID.SetTunings(Kp, Ki, Kd);

    // Initialize display with default values
    displayStatus(0.0, 0.0, 0);
}

void loop() {
    static float lastPressure = -1;
    static float lastOutput = -1;
    static float lastSetpoint = -1;

    // Read MAP Sensor  
    int sensorValue = analogRead(MAP_SENSOR_PIN);
    float voltage = sensorValue * (5.0 / 1023.0);
    
    // MAP Sensor Failsafe  
    if (voltage < MAP_SENSOR_MIN_VOLTAGE) {
        Serial.println("⚠ WARNING: MAP SENSOR FAILURE! DISABLING BOOST CONTROL.");
        digitalWrite(SOLENOID_PIN, LOW);
        displaySensorError();
        return;
    }
    
    // Convert sensor voltage to pressure  
    float rawPressure = ((voltage - MAP_SENSOR_MIN_VOLTAGE) / (MAP_SENSOR_MAX_VOLTAGE - MAP_SENSOR_MIN_VOLTAGE)) * 
                         (MAP_SENSOR_MAX_PRESSURE - MAP_SENSOR_MIN_PRESSURE) + MAP_SENSOR_MIN_PRESSURE;

    // Apply Low-Pass Filter to smooth fluctuations  
    filteredPressure = (filterAlpha * rawPressure) + ((1 - filterAlpha) * filteredPressure);
    float pressureBar = filteredPressure / 100.0;  

    // Read Potentiometer for Desired Pressure  
    int potValue = analogRead(POTENTIOMETER_PIN);
    setpoint = map(potValue, 0, 1023, MAP_SENSOR_MIN_PRESSURE, MAX_PRESSURE_LIMIT);
    float setpointBar = setpoint / 100.0;  

    // Set Overboost Limit (Setpoint + 0.3 bar)  
    overboostThreshold = setpoint + 30;  
    float overboostBar = overboostThreshold / 100.0;

    // Overboost Protection  
    if (filteredPressure > overboostThreshold) {
        overboostShutdown = true;  
        solenoidActive = false;  
        analogWrite(SOLENOID_PIN, 127);  // 50% open instead of shutting off completely
        displayOverboostWarning();  
        printSerialData(pressureBar, setpointBar, 50, voltage, overboostBar);
        return; 
    }

    // Reset overboost protection  
    if (overboostShutdown && filteredPressure < (overboostThreshold - hysteresis)) {
        overboostShutdown = false;
    }

    // Normal PID-Based Boost Control  
    if (!overboostShutdown) {
        myPID.Compute();
        output = constrain(output, 0, 255); 

        if (filteredPressure < (setpoint - hysteresis)) {  
            analogWrite(SOLENOID_PIN, (int)output);  
            solenoidActive = (output > 0);
        } else if (filteredPressure > (setpoint + hysteresis)) {  
            digitalWrite(SOLENOID_PIN, LOW);  
            solenoidActive = false;
        }
    }

    // Convert duty cycle to %  
    int dutyCyclePercent = (output / 255.0) * 100;

    // Update Display & Serial Output  
    if (millis() - lastDisplayUpdate >= displayUpdateInterval || 
        filteredPressure != lastPressure || output != lastOutput || setpoint != lastSetpoint) {
        
        displayStatus(pressureBar, setpointBar, dutyCyclePercent);
        printSerialData(pressureBar, setpointBar, dutyCyclePercent, voltage, overboostBar);
        lastPressure = filteredPressure;
        lastOutput = output;
        lastSetpoint = setpoint;
        lastDisplayUpdate = millis();
    }
}

//  **Display on OLED**  
void displayStatus(float pressureBar, float setpointBar, int dutyCyclePercent) {
    display.clearDisplay();  
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);  
    display.setCursor(0, 0);

    display.print("Setpoint: ");
    display.print(setpointBar, 2);
    display.print(" bar");

    display.setCursor(0, 10);
    display.print("Pressure: ");
    display.print(pressureBar, 2);
    display.print(" bar");

    display.setCursor(0, 20);
    display.print("Duty: ");
    display.print(dutyCyclePercent);
    display.print("%");

    display.display();  
}

//  **Overboost Warning**  
void displayOverboostWarning() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("OVERBOOST!");
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print("Boost Cut Active!");
    display.display();
}

//  **MAP Sensor Error**  
void displaySensorError() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("SENSOR FAIL!");
    display.display();
}

//  **Serial Monitor Output**  
void printSerialData(float pressureBar, float setpointBar, int dutyCyclePercent, float sensorVoltage, float obLimit) {
    Serial.println("==================================");
    Serial.print("Setpoint  : "); Serial.print(setpointBar, 2); Serial.println(" bar");
    Serial.print("Pressure  : "); Serial.print(pressureBar, 2); Serial.println(" bar");
    Serial.print("Duty Cycle: "); Serial.print(dutyCyclePercent); Serial.println("%");
    Serial.println("==================================\n");
}
