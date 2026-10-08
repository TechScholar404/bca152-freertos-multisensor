#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SENSOR";

void taskA(void *pvParameters)
{
    while (1)
    {
        ESP_LOGI(TAG, "Task A running");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskB(void *pvParameters)
{
    while (1)
    {
        ESP_LOGI(TAG, "Task B running");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "BCA152 FreeRTOS Multisensor");
    ESP_LOGI(TAG, "System starting...");

    xTaskCreate(taskA, "TaskA", 2048, NULL, 5, NULL);
    xTaskCreate(taskB, "TaskB", 2048, NULL, 5, NULL);
}