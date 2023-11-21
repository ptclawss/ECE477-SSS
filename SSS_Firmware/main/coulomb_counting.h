//------------- Imports ---------------//
// Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <esp_timer.h>

// Custom Libraries
#include "constants.h"

//------------- Global Variables ---------------//

//------------- Function Declarations ---------------//
void IRAM_ATTR coulomb_counter_isr_handler(void*);
void initBatteryInfo(volatile double);