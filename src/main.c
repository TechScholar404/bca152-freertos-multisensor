#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MAIN";

void TaskA(void *pvParameters)
{
    while (1)
    {
        ESP_LOGI(TAG, "Task A running");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TaskB(void *pvParameters)
{
    while (1)
    {
        ESP_LOGI(TAG, "Task B running");

        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "BCA152 FreeRTOS Multisensor");
    ESP_LOGI(TAG, "System starting...");

    xTaskCreate(
        TaskA,
        "Task A",
        2048,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        TaskB,
        "Task B",
        2048,
        NULL,
        1,
        NULL
    );
}