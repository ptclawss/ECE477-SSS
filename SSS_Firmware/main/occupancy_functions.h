//------------- Imports ---------------//
// Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Custom Libraries
#include "constants.h"
#include "adc_functions.h"
#include "i2c_functions.h"

//------------- Global Variables ---------------//
extern volatile TaskHandle_t occupancy_update__task_handle;     /*!< Handler for the occupancy_update task */
extern bool restart_occupancy_update;                           /*!< Status flag for need to reset the occupancy_update task */

//------------- Function Declarations ---------------//
void occupancy_update_start(void*);
void resetchk_occupancy_update();
void occupancy_update(void*);