#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h" 

//------------- Constant and Variable Defenitions ---------------//
#define FSR1_channel            ADC_CHANNEL_6 // GPIO 34
#define FSR2_channel            ADC_CHANNEL_7 // GPIO 35
#define FSR_Transistor_cntrl    32 // GPIO32

const static char * TAG = "OUTPUT";

//------------- Function Declarations ---------------//
adc_oneshot_unit_handle_t setup_ADC1();
adc_oneshot_unit_handle_t init_ADC1();
void config_ADC1_channels(adc_oneshot_unit_handle_t);
int read_ADC_channel(int, adc_oneshot_unit_handle_t, adc_channel_t);

void app_main(void)
{
    //------------- ADC Setup ---------------//
    adc_oneshot_unit_handle_t adc1_handle = setup_ADC1();
    
    // Configure GPIO32 as output for transistor to control FSR circuitry
    gpio_set_direction(FSR_Transistor_cntrl, GPIO_MODE_OUTPUT);
    
    //------------- Main Loop ---------------//
    while (1)
    {
        // Turn FSR control transistor on
        gpio_set_level(FSR_Transistor_cntrl, 1);
        
        // Read FSR values
        int val = read_ADC_channel(ADC_UNIT_1, adc1_handle, FSR1_channel);
        val = read_ADC_channel(ADC_UNIT_1, adc1_handle, FSR2_channel);
        
        // Delay for 1s
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

adc_oneshot_unit_handle_t setup_ADC1()
{
    adc_oneshot_unit_handle_t adc1_handle = init_ADC1();
    config_ADC1_channels(adc1_handle);
    return adc1_handle;
}

adc_oneshot_unit_handle_t init_ADC1()
{
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    return adc1_handle;
}

void config_ADC1_channels(adc_oneshot_unit_handle_t adc1_handle)
{
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, FSR1_channel, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, FSR2_channel, &config));
}

int read_ADC_channel(int adc_unit, adc_oneshot_unit_handle_t adc_handle, adc_channel_t channel)
{
    static int adc_raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, channel, &adc_raw));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", adc_unit + 1, channel, adc_raw);
    return adc_raw;
}