//------------- Imports ---------------//
// Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <freertos/queue.h>
#include <esp_sleep.h>
// Custom Libraries
#include "constants.h"
#include "adc_functions.h"
#include "i2c_functions.h"
#include "occupancy_functions.h"
#include "interrupt_functions.h"
#include "coulomb_counting.h"
#include "networking.h"


//------------- Variable Defenitions ---------------//

// Debug String
const static char * TAG = "OUTPUT";

// ADC Vars
adc_oneshot_unit_handle_t adc1_handle;                          /*!< Global Var from adc_functions.h */

// Occupancy Vars
volatile TaskHandle_t occupancy_update__task_handle = NULL;     /*!< Global Var from occupancy_functions.h */
bool restart_occupancy_update = false;                          /*!< Global Var from occupancy_functions.h */

void app_main(void)
{
    //------------- Networking ---------------//
    nvs_flash_init();
    wifi_connection();

    vTaskDelay(pdMS_TO_TICKS(2000));
    printf("WIFI was initiated ...........\n\n");

    //------------- ADC Setup ---------------//
    adc1_handle = setup_ADC1();
    
    // Configure GPIO32 as output for transistor to control FSR circuitry
    gpio_set_direction(FSR_Transistor_cntrl, GPIO_MODE_OUTPUT);
    gpio_set_level(FSR_Transistor_cntrl, 1);

    //------------- I2C Init ---------------//
 
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization error: %s", esp_err_to_name(err));
        return;
    }

    //------------- ISR Init ---------------//
    gpio_install_isr_service(0);
    assign_interrupt(GPIO_INTR_ANYEDGE, GPIO34, gpio_34_35_isr_handler);
    assign_interrupt(GPIO_INTR_ANYEDGE, GPIO35, gpio_34_35_isr_handler);
    assign_interrupt(GPIO_INTR_NEGEDGE, GPIO33, coulomb_counter_isr_handler);

    //------------- CC Init ---------------// 
    initBatteryInfo(FULLY_CHARGED_BATTERY_mAH);

    //------------- Main Loop ---------------//
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Brief description of the function
 *
 * Detailed description of the function. You can provide additional information,
 * usage details, algorithm explanations, and any special considerations.
 *
 * @param parameter1 Type | Description
 * @param parameter2 Type | Description
 * @return Name | Type | Description
 *
 * @note Any additional notes or important considerations
 * @warning Any warnings or cautions
 * @attention Any attention points
 *
 * @authors Your Names
 * @date Updated:
 */