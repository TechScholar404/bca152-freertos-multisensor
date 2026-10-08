#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "dht.h"

#define DHT_PIN GPIO_NUM_4
#define LDR_ADC_CHANNEL ADC_CHANNEL_6

void app_main(void)
{
    printf("BCA152 DHT22 and LDR Test\n");

    adc_oneshot_unit_handle_t adc_handle;

    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1
    };

    adc_oneshot_new_unit(&adc_config, &adc_handle);

    adc_oneshot_chan_cfg_t adc_channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12
    };

    adc_oneshot_config_channel(
        adc_handle,
        LDR_ADC_CHANNEL,
        &adc_channel_config
    );

    while (1)
    {
        float temperature = 0;
        float humidity = 0;

        esp_err_t result = dht_read_float_data(
            DHT_TYPE_AM2301,
            DHT_PIN,
            &humidity,
            &temperature
        );

        if (result == ESP_OK)
        {
            printf("Temperature: %.2f C\n", temperature);
            printf("Humidity: %.2f %%\n", humidity);
        }
        else
        {
            printf("DHT22 read failed: %s\n", esp_err_to_name(result));
        }

        int rawLdr = 0;

        adc_oneshot_read(
            adc_handle,
            LDR_ADC_CHANNEL,
            &rawLdr
        );

        int lightLevel = 100 - (((rawLdr - 34) * 100) / (4063 - 34));

        if (lightLevel < 0)
        {
            lightLevel = 0;
        }

        if (lightLevel > 100)
        {
            lightLevel = 100;
        }

        printf("LDR Raw: %d\n", rawLdr);
        printf("Relative Light Level: %d %%\n", lightLevel);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}