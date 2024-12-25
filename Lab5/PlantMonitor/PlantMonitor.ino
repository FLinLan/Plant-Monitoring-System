/** Uses chatGPT for generating the Doxygen comments
 * @file Lab5.ino
 * @brief Multi-Sensor Monitoring System using FreeRTOS on Arduino
 * @author Max Lan, Yuanzu Chen
 * @date 8/15/2024
 *
 * This program implements a multi-sensor monitoring system using FreeRTOS on an Arduino platform. 
 * It integrates various sensors including temperature, humidity, water level, light, distance, 
 * and motion detection. The system features two main categories (Security and Sensors) with 
 * multiple modes in each category. Sensor data is collected through separate tasks and 
 * communicated via FreeRTOS queues. The current readings and modes are displayed on an I2C LCD.
 */

// ==================== INCLUDES ==================
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "soc/timer_group_reg.h"
#include "esp_timer.h"

/**
 * @defgroup PinDefinitions Pin Definitions
 * @{
 */
#define DHT_PIN 4                 /**< Pin for DHT sensor */
#define WATER_LEVEL_PIN 5         /**< Pin for water level sensor */
#define LIGHT_SENSOR_PIN A0       /**< Pin for light sensor */
#define MODE_BUTTON_PIN 3         /**< Pin for mode selection button */
#define TASK_BUTTON_PIN 2         /**< Pin for task selection button */
#define ULTRASONIC_TRIG_PIN 7     /**< Trigger pin for ultrasonic sensor */
#define ULTRASONIC_ECHO_PIN 8     /**< Echo pin for ultrasonic sensor */
#define MOTION_SENSOR_PIN 9       /**< Pin for motion sensor */
#define DHT_TYPE DHT11            /**< Type of DHT sensor */
/** @} */

// ==================== MACROS ==================
esp_timer_handle_t lightTimer;    /**< Timer handle for light sensor readings */
hw_timer_t* timer = NULL;         /**< Hardware timer */
unsigned long lastToggleTime = 0; /**< Last time the timer was toggled */
const unsigned long interval = 1000000; /**< Interval for sensor readings (1 second in microseconds) */

/**
 * @defgroup TaskHandles Task Handles
 * @{
 */
TaskHandle_t dhtTaskHandle = NULL;
TaskHandle_t waterLevelTaskHandle = NULL;
TaskHandle_t lightTaskHandle = NULL;
TaskHandle_t ultrasonicTaskHandle = NULL;
TaskHandle_t motionTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t userInputTaskHandle = NULL;
/** @} */

/**
 * @defgroup QueueHandles Queue Handles
 * @{
 */
QueueHandle_t sensorDataQueue;    /**< Queue for sensor data */
QueueHandle_t securityDataQueue;  /**< Queue for security-related data */
/** @} */

// ==================== LIBRARY FUNCTIONS ==================
LiquidCrystal_I2C lcd(0x27, 16, 2);  /**< LCD object for I2C communication */

// ==================== STRUCT ==================

DHT dht(DHT_PIN, DHT_TYPE);  /**< DHT sensor object */

/**
 * @struct SensorData
 * @brief Structure to hold sensor data
 */
struct SensorData {
    float temperature;  /**< Temperature reading */
    float humidity;     /**< Humidity reading */
    int waterLevel;     /**< Water level reading */
    int light;          /**< Light intensity reading */
    int sensorType;     /**< Type of sensor */
};

/**
 * @struct SecurityData
 * @brief Structure to hold security-related data
 */
struct SecurityData {
    float distance;      /**< Distance reading from ultrasonic sensor */
    bool motionDetected; /**< Motion detection status */
    int sensorType;      /**< Type of security sensor */
};

/**
 * @enum DisplayMode
 * @brief Enum for display modes
 */
enum DisplayMode {
    SECURITY,       /**< Security mode */
    SENSORS,        /**< Sensors mode */
    NUM_CATEGORIES  /**< Number of categories */
};

/**
 * @enum SecurityMode
 * @brief Enum for security modes
 */
enum SecurityMode {
    MOTION,             /**< Motion detection mode */
    DISTANCE,           /**< Distance measurement mode */
    NUM_SECURITY_MODES  /**< Number of security modes */
};

/**
 * @enum SensorMode
 * @brief Enum for sensor modes
 */
enum SensorMode {
    TEMP_HUMID,     /**< Temperature and humidity mode */
    WATER_LEVEL,    /**< Water level mode */
    PHOTODIODE,     /**< Light intensity mode */
    NUM_SENSOR_MODES /**< Number of sensor modes */
};

// Global variables for tracking current modes and display state
volatile DisplayMode currentCategory = SENSORS;  /**< Current display category */
volatile int currentSecurityMode = MOTION;       /**< Current security mode */
volatile int currentSensorMode = TEMP_HUMID;     /**< Current sensor mode */
volatile uint32_t lastModeDebounceTime = 0;      /**< Last debounce time for mode button */
volatile uint32_t lastTaskDebounceTime = 0;      /**< Last debounce time for task button */
volatile bool displayNeedsUpdate = true;         /**< Flag to indicate if display needs updating */

const uint32_t debounceDelay = 50;               /**< Debounce delay in milliseconds */

/**
 * @brief Task to read DHT sensor
 * @param pvParameters Pointer to task parameters
 */
void dhtTask(void *pvParameters) {
    SensorData data;
    data.sensorType = TEMP_HUMID;

    while (1) {
        // Update the timer
        volatile uint32_t* timer_update = (volatile uint32_t*)TIMG_T0UPDATE_REG(0);
        *timer_update = 1;
        // Read current timer value
        volatile uint32_t* timer_value = (volatile uint32_t*)TIMG_T0LO_REG(0);
        unsigned long currentTime = *timer_value;
        
        // Check if it's time to read the sensor (every 1 second)
        if (currentTime - lastToggleTime >= interval) {
          data.temperature = dht.readTemperature();
          data.humidity = dht.readHumidity();
          xQueueSend(sensorDataQueue, &data, portMAX_DELAY);
        }
    }
}

/**
 * @brief Callback function for light sensor timer
 * @param arg Pointer to arguments
 */
void lightTimerCallback(void* arg) {
    SensorData data;
    data.sensorType = PHOTODIODE;

    // Read the light sensor
    data.light = analogRead(LIGHT_SENSOR_PIN);

    // Send the data to the queue
    xQueueSend(sensorDataQueue, &data, portMAX_DELAY);
}

/**
 * @brief Task for photo resistor to read light
 * @param pvParameters Pointer to task parameters
 */
void lightTask(void *pvParameters) {
    // Create an esp_timer to replace the vTaskDelay mechanism
    const esp_timer_create_args_t lightTimerArgs = {
        .callback = &lightTimerCallback,
        .name = "light_timer"
    };

    // Create the timer
    esp_timer_create(&lightTimerArgs, &lightTimer);

    // Start the timer with a period of 2000 ms (2 seconds)
    esp_timer_start_periodic(lightTimer, 2000000); // 2000 ms in microseconds

    // Light task loop can still run, but the work is done in the timer callback
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Use a delay to avoid a tight loop
    }
}

/**
 * @brief Task to read water level sensor
 * @param pvParameters Pointer to task parameters
 */
void waterLevelTask(void *pvParameters) {
    SensorData data;
    data.sensorType = WATER_LEVEL;
    while (1) {
        data.waterLevel = analogRead(WATER_LEVEL_PIN);
        xQueueSend(sensorDataQueue, &data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Read every 1 second
    }
}

/**
 * @brief Task to read ultrasonic sensor
 * @param pvParameters Pointer to task parameters
 */
void ultrasonicTask(void *pvParameters) {
    SecurityData data;
    data.sensorType = DISTANCE;
    while (1) {
        // Trigger the ultrasonic sensor
        digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
        
        // Measure the response
        long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
        data.distance = duration * 0.034 / 2; // Convert to cm
        
        xQueueSend(securityDataQueue, &data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Read every 1 second
    }
}

/**
 * @brief Task to read motion sensor
 * @param pvParameters Pointer to task parameters
 */
void motionTask(void *pvParameters) {
    SecurityData data;
    data.sensorType = MOTION;
    while (1) {
        data.motionDetected = digitalRead(MOTION_SENSOR_PIN) == HIGH;
        xQueueSend(securityDataQueue, &data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(31));  // Read approximately 32 times per second
    }
}

/**
 * @brief Task to update display
 * @param pvParameters Pointer to task parameters
 */
void displayTask(void *pvParameters) {
    SensorData data1;
    SecurityData data2;

    char buffer[50];
    while (1) {
        // Update display category and mode if needed
        if (displayNeedsUpdate) {
            lcd.clear();
            lcd.setCursor(0, 0);
            if (currentCategory == SECURITY) {
                lcd.print("Security - ");
                lcd.print(currentSecurityMode == MOTION ? "Motion" : "Distance");
            } else {
                lcd.print("Sensors - ");
                switch (currentSensorMode) {
                    case TEMP_HUMID:
                        lcd.print("Temp/Humid");
                        break;
                    case WATER_LEVEL:
                        lcd.print("Water Level");
                        break;
                    case PHOTODIODE:
                        lcd.print("Light");
                        break;
                }
            }
            displayNeedsUpdate = false;
        }

        // Check for new data from queues
        if (xQueueReceive(securityDataQueue, &data2, portMAX_DELAY) == pdTRUE |
            xQueueReceive(sensorDataQueue, &data1, portMAX_DELAY) == pdTRUE) {
            
            // Update display based on current category and mode
            if (currentCategory == SECURITY) {
                switch (currentSecurityMode) {
                    case MOTION:
                        if (data2.sensorType == MOTION) {
                            lcd.setCursor(0, 1);
                            lcd.print("Motion: ");
                            lcd.print(data2.motionDetected ? "Detected" : "None    ");
                        }
                        break;
                    case DISTANCE:
                        if (data2.sensorType == DISTANCE) {
                            snprintf(buffer, sizeof(buffer), "Dist: %.1fcm    ", data2.distance);
                            lcd.setCursor(0, 1);
                            lcd.print(buffer);
                        }
                        break;
                }
            } else { // SENSORS category
                switch (currentSensorMode) {
                    case TEMP_HUMID:
                        if (data1.sensorType == TEMP_HUMID) {
                            snprintf(buffer, sizeof(buffer), "T:%.1fC H:%.1f%%", data1.temperature, data1.humidity);
                            lcd.setCursor(0, 1);
                            lcd.print(buffer);
                        }
                        break;
                    case WATER_LEVEL:
                        if (data1.sensorType == WATER_LEVEL) {
                            snprintf(buffer, sizeof(buffer), "Water: %d    ", data1.waterLevel);
                            lcd.setCursor(0, 1);
                            lcd.print(buffer);
                        }
                        break;
                    case PHOTODIODE:
                        if (data1.sensorType == PHOTODIODE) {
                            snprintf(buffer, sizeof(buffer), "Light: %d    ", data1.light);
                            lcd.setCursor(0, 1);
                            lcd.print(buffer);
                        }
                        break;
                }
            }
        }
    }
}

/**
 * @brief Task to handle user input
 * @param pvParameters Pointer to task parameters
 */
void userInputTask(void *pvParameters) {
    while (1) {
        uint32_t currentTime = millis();

        // Mode button handling
        if (currentTime - lastModeDebounceTime > debounceDelay) {
            if (digitalRead(MODE_BUTTON_PIN) == LOW) {
                lastModeDebounceTime = currentTime;
                currentCategory = (currentCategory == SECURITY) ? SENSORS : SECURITY;
                displayNeedsUpdate = true;
            }
        }

        // Task button handling
        if (currentTime - lastTaskDebounceTime > debounceDelay) {
            if (digitalRead(TASK_BUTTON_PIN) == LOW) {
                lastTaskDebounceTime = currentTime;
                if (currentCategory == SECURITY) {
                    currentSecurityMode = (currentSecurityMode + 1) % NUM_SECURITY_MODES;
                } else {
                    currentSensorMode = (currentSensorMode + 1) % NUM_SENSOR_MODES;
                }
                displayNeedsUpdate = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz task frequency
    }
}

/**
 * @brief Setup function
 * 
 * Initializes hardware, creates queues, and starts FreeRTOS tasks
 */
void setup() {
    Serial.begin(115200);
    
    // Initialize I2C LCD
    Wire.begin();
    lcd.init();
    lcd.backlight();
    
    // Initialize DHT sensor
    dht.begin();
    
    // Configure timer
    volatile uint32_t* timer_config = (volatile uint32_t*)TIMG_T0CONFIG_REG(0);
    *timer_config = 0; // Clear undesired information from timer configuration
    *timer_config |= (79 << 13); // Setting the prescaler (divider) at 80 (number shows 79 due to zero-based indexing)
    *timer_config |= (1 << 30); // Enable increment mode
    *timer_config |= (1 << 29); // Enable auto-reload
    *timer_config |= (1 << 31); // Start the timer
    
    // Initialize pins
    pinMode(WATER_LEVEL_PIN, INPUT);
    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(TASK_BUTTON_PIN, INPUT_PULLUP);
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    pinMode(MOTION_SENSOR_PIN, INPUT);
    
    // Create queues for inter-task communication
    sensorDataQueue = xQueueCreate(10, sizeof(SensorData));
    securityDataQueue = xQueueCreate(10, sizeof(SecurityData));

    // Create tasks and assign them to the dual core 
    xTaskCreatePinnedToCore(dhtTask, "DHTTask", 2048, NULL, 2, &dhtTaskHandle, 0);
    xTaskCreatePinnedToCore(waterLevelTask, "WaterLevelTask", 2048, NULL, 2, &waterLevelTaskHandle, 0);
    xTaskCreatePinnedToCore(lightTask, "LightDetection", 2048, NULL, 2, &lightTaskHandle, 0);
    xTaskCreatePinnedToCore(ultrasonicTask, "UltrasonicTask", 2048, NULL, 2, &ultrasonicTaskHandle, 0);
    xTaskCreatePinnedToCore(motionTask, "MotionTask", 2048, NULL, 2, &motionTaskHandle, 0);
    xTaskCreatePinnedToCore(displayTask, "DisplayTask", 2048, NULL, 1, &displayTaskHandle, 1);
    xTaskCreatePinnedToCore(userInputTask, "UserInputTask", 2048, NULL, 1, &userInputTaskHandle, 1);
}

/**
 * @brief Main loop function
 * 
 * Empty as all functionality is handled by FreeRTOS tasks
 */
void loop() {
}
