//------------- Imports ---------------//
#include "occupancy_functions.h"
#include "networking.h"

//------------- Local Variables ---------------//
const static char * TAG = "OUTPUT"; // Debug String
float fsr1_avg;
float fsr2_avg;
float thermalData[64];
float thermalAvg;
SemaphoreHandle_t fsrSamplingSemaphore;
SemaphoreHandle_t thermalProcessingSemaphore;
esp_err_t amg8833_err;

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
    xTaskCreate(occupancy_update, "occupancy_update", 4096, NULL, 10, &temp_handle);
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

    // Create semaphores
    fsrSamplingSemaphore = xSemaphoreCreateBinary();
    thermalProcessingSemaphore = xSemaphoreCreateBinary();

    // Create and start the FSR sampling task
    xTaskCreate(fsrSamplingTask, "FSR Sampling Task", 2048, NULL, 5, NULL);

    // Create and start the thermal processing task
    xTaskCreate(thermalProcessingTask, "Thermal Processing Task", 2048, NULL, 5, NULL);

    if ((xSemaphoreTake(fsrSamplingSemaphore, pdMS_TO_TICKS(FSR_SAMPLE_NUM * (FSR_READ_DELAY + 1))) == pdTRUE) &&
        (xSemaphoreTake(thermalProcessingSemaphore, portMAX_DELAY) == pdTRUE)
    )
    {
        printf("FSR1 Avg: %f\n", fsr1_avg);
        printf("FSR2 Avg: %f\n", fsr2_avg);
        printf("Thermal Avg: %f\n", thermalAvg);
    }

    if (amg8833_err != ESP_OK)
    {
        ESP_LOGE("ERROR", "AMG8833 Communication Failure");
    }

    // Delete the semaphores
    vSemaphoreDelete(fsrSamplingSemaphore);
    vSemaphoreDelete(thermalProcessingSemaphore);

    // Reset check
    resetchk_occupancy_update();

    // Send data
    post_rest_function(SEATID, "DATA", fsr1_avg, fsr2_avg, thermalAvg);

    // Reset check
    resetchk_occupancy_update();

    // Delete the task when it's done
    ESP_LOGI(TAG, "Task Finished!\n");
    occupancy_update__task_handle = NULL;
    vTaskDelete(NULL);
}

void fsrSamplingTask(void* arg)
{
    // Sample the FSRs
    for(int i = 0; i < FSR_SAMPLE_NUM; i++)
    {
        float val = read_ADC_channel(ADC_UNIT_1, adc1_handle, FSR1_channel);
        fsr1_avg += val;
        fsr2_avg += read_ADC_channel(ADC_UNIT_1, adc1_handle, FSR2_channel);
        vTaskDelay(pdMS_TO_TICKS(FSR_READ_DELAY));
    }

    // Average the FSRs
    fsr1_avg = fsr1_avg / FSR_SAMPLE_NUM;
    fsr2_avg = fsr2_avg / FSR_SAMPLE_NUM;

    // Give the semaphore to signal completion
    xSemaphoreGive(fsrSamplingSemaphore);

    // Delete the task when it's done
    vTaskDelete(NULL);
}

void thermalProcessingTask(void* arg)
{
    thermalAvg = 0;
    
    // Read and average thermal camera
    while(1)
    {
        bool reread = false;

        amg8833_err = readThermalArray(thermalData);
        if (amg8833_err != ESP_OK) {break;}

        for (int i = 0; i < PIXELS_NUM; i++)
        {
            // Check for anomalous negative value
            if (thermalData[i] < 0)
            {
                reread = true;
                break;
            }
            else {thermalAvg += thermalData[i];}
        }

        if (reread == false) {break;} // If no anomalous value read, break
        else
        {
            // Wait a bit before re-reading
            vTaskDelay(pdMS_TO_TICKS(AMG8833_REREAD_WAIT_TIME));
        }
    }
    thermalAvg = thermalAvg / PIXELS_NUM;

    // Signal completion of thermal data processing
    xSemaphoreGive(thermalProcessingSemaphore);

    // Delete the task when it's done
    vTaskDelete(NULL);
}