//------------- Imports ---------------//
// Standard Libraries
#include "driver/gpio.h"

// Custom Libraries
#include "constants.h"
#include "occupancy_functions.h"

//------------- Global Variables ---------------//

//------------- Function Declarations ---------------//
void IRAM_ATTR gpio_34_35_isr_handler(void* arg);
void assign_interrupt(gpio_int_type_t, int, gpio_isr_t);