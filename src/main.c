#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "dht.h"

/* =====================================================
 * DHT22 CONFIGURATION
 * ===================================================== */

#define DHT_GPIO GPIO_NUM_4

/* =====================================================
 * SENSOR DATA
 * ===================================================== */

typedef struct
{
    float temperature;
    float humidity;
    int lightLevel;
    bool motionDetected;
} SensorData;

/* =====================================================
 * FREE RTOS QUEUES
 * ===================================================== */

QueueHandle_t displayQueue;
QueueHandle_t alarmQueue;

/* =====================================================
 * SENSOR READING FUNCTION
 * ===================================================== */

void readSensors(SensorData *data)
{
    float temperature = 0.0;
    float humidity = 0.0;

    /*
     * Read REAL values from DHT22
     */
    esp_err_t result = dht_read_float_data(
        DHT_TYPE_AM2301,
        DHT_GPIO,
        &humidity,
        &temperature
    );

    if (result == ESP_OK)
    {
        data->temperature = temperature;
        data->humidity = humidity;
    }
    else
    {
        printf("ERROR: Failed to read DHT22!\n");

        /*
         * Keep previous values if reading fails.
         * On first failure these will be 0.
         */
    }

    /*
     * These are still example values.
     * Replace them with your actual light and
     * motion sensor readings when ready.
     */
    data->lightLevel = 500;
    data->motionDetected = false;
}

/* =====================================================
 * SENSOR TASK
 * ===================================================== */

void SensorTask(void *pvParameters)
{
    SensorData data = {
        .temperature = 0.0,
        .humidity = 0.0,
        .lightLevel = 500,
        .motionDetected = false
    };

    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /*
         * Read sensors
         */
        readSensors(&data);

        printf("\nSensorTask: Reading sensors...\n");

        printf("Temperature: %.2f C\n",
               data.temperature);

        printf("Humidity: %.2f %%\n",
               data.humidity);

        printf("Light Level: %d\n",
               data.lightLevel);

        printf("Motion: %s\n",
               data.motionDetected
                   ? "DETECTED"
                   : "NOT DETECTED");

        /*
         * Send sensor data to DisplayTask
         */
        if (xQueueSend(
                displayQueue,
                &data,
                portMAX_DELAY) != pdPASS)
        {
            printf("ERROR: Failed to send to display queue!\n");
        }

        /*
         * Send sensor data to AlarmTask
         */
        if (xQueueSend(
                alarmQueue,
                &data,
                portMAX_DELAY) != pdPASS)
        {
            printf("ERROR: Failed to send to alarm queue!\n");
        }

        /*
         * Run every 2 seconds
         */
        vTaskDelayUntil(
            &lastWakeTime,
            pdMS_TO_TICKS(2000)
        );
    }
}

/* =====================================================
 * DISPLAY TASK
 * ===================================================== */

void DisplayTask(void *pvParameters)
{
    SensorData data;

    for (;;)
    {
        if (xQueueReceive(
                displayQueue,
                &data,
                portMAX_DELAY))
        {
            printf("\n----- DISPLAY -----\n");

            printf("Temperature: %.2f C\n",
                   data.temperature);

            printf("Humidity: %.2f %%\n",
                   data.humidity);

            printf("Light Level: %d\n",
                   data.lightLevel);

            printf("Motion: %s\n",
                   data.motionDetected
                       ? "DETECTED"
                       : "NOT DETECTED");

            printf("-------------------\n");
        }
    }
}

/* =====================================================
 * ALARM TASK
 * ===================================================== */

void AlarmTask(void *pvParameters)
{
    SensorData data;

    for (;;)
    {
        if (xQueueReceive(
                alarmQueue,
                &data,
                portMAX_DELAY))
        {
            /*
             * Alarm condition:
             * Motion detected AND low light
             */
            if (data.motionDetected &&
                data.lightLevel < 100)
            {
                printf("\n!!! ALARM !!!\n");
                printf("Motion detected in darkness!\n");
            }
            else
            {
                printf("Alarm: NORMAL\n");
            }
        }
    }
}

/* =====================================================
 * MAIN
 * ===================================================== */

void app_main(void)
{
    printf("\n");
    printf("================================\n");
    printf(" ESP32 FREERTOS SENSOR SYSTEM\n");
    printf("================================\n");

    /*
     * Create Display Queue
     */
    displayQueue = xQueueCreate(
        10,
        sizeof(SensorData)
    );

    /*
     * Create Alarm Queue
     */
    alarmQueue = xQueueCreate(
        10,
        sizeof(SensorData)
    );

    /*
     * Check queues
     */
    if (displayQueue == NULL ||
        alarmQueue == NULL)
    {
        printf("ERROR: Queue creation failed!\n");
        return;
    }

    printf("Queues created successfully.\n");

    /*
     * Create Sensor Task
     */
    if (xTaskCreate(
            SensorTask,
            "SensorTask",
            4096,
            NULL,
            2,
            NULL) != pdPASS)
    {
        printf("ERROR: SensorTask creation failed!\n");
        return;
    }

    /*
     * Create Display Task
     */
    if (xTaskCreate(
            DisplayTask,
            "DisplayTask",
            4096,
            NULL,
            1,
            NULL) != pdPASS)
    {
        printf("ERROR: DisplayTask creation failed!\n");
        return;
    }

    /*
     * Create Alarm Task
     */
    if (xTaskCreate(
            AlarmTask,
            "AlarmTask",
            4096,
            NULL,
            1,
            NULL) != pdPASS)
    {
        printf("ERROR: AlarmTask creation failed!\n");
        return;
    }

    printf("All tasks started successfully.\n");
}