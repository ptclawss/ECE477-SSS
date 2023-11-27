//------------- Imports ---------------//
#include "occupancy_functions.h"

//------------- Local Variables ---------------//
// Debug String
const static char * TAG = "OUTPUT";

//------------- Functions ---------------//
/**
 * @brief Starts the occupancy_update task
 *
 * @param  arg      void*   |   Args
 * @return void
 *
 * @note
 * @warning
 * @attention
 *
 * @authors Roshan Sundar
 * @date Updated: 11/9/2023
 */
void occupancy_update_start(void* arg)
{
    // Slight delay before starting the task
    vTaskDelay(pdMS_TO_TICKS(TASK_DELETE_WAIT_TIME));

    // Start the occupancy_update task
    TaskHandle_t temp_handle = NULL;
    xTaskCreate(occupancy_update, "occupancy_update", 2048, NULL, 10, &temp_handle);
    occupancy_update__task_handle = temp_handle;

    // Stop this task
    vTaskDelete(NULL);
}

/**
 * @brief Resets the occupancy_update task on flag
 *
 * @return void
 *
 * @note
 * @warning
 * @attention
 *
 * @authors Roshan Sundar
 * @date Updated: 11/9/2023
 */
void resetchk_occupancy_update()
{
    if (restart_occupancy_update)
    {
        // Reset the flag
        restart_occupancy_update = false;
        ESP_LOGI(TAG, "Task RESTARTED!\n");

        // Save the current task handle
        TaskHandle_t temp_handle = occupancy_update__task_handle;
        occupancy_update__task_handle = NULL;

        // Run the occupancy_update_start task
        xTaskCreate(occupancy_update_start, "occupancy_update_start", 2048, NULL, 10, NULL);

        // Delete the current occupancy_update task
        vTaskDelete(temp_handle);
    }
}

/**
 * @brief Main task to sample sensors, prep data, and send to remote server
 *
 * @param  arg      void*   |   Args
 * @return void
 *
 * @note
 * @warning
 * @attention
 *
 * @authors Roshan Sundar
 * @date Updated: 11/9/2023
 */
void occupancy_update(void* arg)
{
    // Delay slightly once task is started for values to settle down
    ESP_LOGI(TAG, "Task Started!\n");
    vTaskDelay(pdMS_TO_TICKS(FSR_SENSOR_SETTLE_TIME));

    // Sample the FSR values
    for(int i = 0; i < FSR_SAMPLE_NUM; i++)
    {

        // Read FSRs
        int val = read_ADC_channel(ADC_UNIT_1, adc1_handle, FSR1_channel);
        printf("FSR1 val: %d\n", val);

        val = read_ADC_channel(ADC_UNIT_1, adc1_handle, FSR2_channel);
        printf("FSR2 val: %d\n", val);

        // Slight read delay
        vTaskDelay(pdMS_TO_TICKS(FSR_READ_DELAY));
    }

    float thermalData[64]; 
    readThermalArray(thermalData);
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            int index = row * 8 + col;
            ESP_LOGI("Test", "[%d, %d]: %f", row, col, thermalData[index]);
        }
    }

    // Reset check
    resetchk_occupancy_update();

    // Delete the task when it's done
    ESP_LOGI(TAG, "Task Finished!\n");
    occupancy_update__task_handle = NULL;
    vTaskDelete(NULL);
}