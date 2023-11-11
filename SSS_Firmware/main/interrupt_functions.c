//------------- Imports ---------------//
#include "interrupt_functions.h"

//------------- Local Variables ---------------//
int last_interrupt_time_ticks = 0;                              /*!< Time since last GPIO34 GPIO35 ISR interrupt in ticks */

//------------- Functions ---------------//
/**
 * @brief ISR for GPIO34 and GPIO35
 *
 * Recieve posedge/negedge from FSRs connected to GPIO34 and GPIO35.
 * Debounce the transitions and execute the occupancy_update task
 *
 * @param  arg      void*   |   Args
 * @return void
 *
 * @note
 * @warning Keep this function as light as possible, ISRs need to be quick.
 * @attention
 *
 * @authors Roshan Sundar
 * @date Updated: 11/9/2023
 */
void IRAM_ATTR gpio_34_35_isr_handler(void* arg)
{
    // Get # of ticks since ISR started
    uint32_t current_time_ticks = xTaskGetTickCountFromISR();

    // Calculate the time since the last interrupt
    uint32_t time_since_last_interrupt = current_time_ticks - last_interrupt_time_ticks;

    // Handle tick count overflow
    if (current_time_ticks < last_interrupt_time_ticks) {
        // Overflow occurred, adjust the calculation
        time_since_last_interrupt = current_time_ticks + (UINT32_MAX - last_interrupt_time_ticks);
    }

    // If FSR has changed...
    if ((time_since_last_interrupt) > pdMS_TO_TICKS(FSR_DEBOUNCE_TIME)) // Debounce
    {
        last_interrupt_time_ticks = current_time_ticks; // Update # of ticks

        // If the occupancy_update task is not running, start it
        if (occupancy_update__task_handle == NULL)
        {
            xTaskCreate(occupancy_update_start, "occupancy_update_start", 2048, NULL, 10, NULL);
        }
        // If the occupancy_update task is in the middle of running, restart it
        else
        {
            restart_occupancy_update = true;
        }
    }
}

/**
 * @brief Assign interrupt to a pin.
 *
 * @param  interrupt_type           gpio_int_type_t     |   Type of interrupt (GPIO_INTR_{edge type})
 * @param  pin_num                  int                 |   Pin # to assign interrupt to
 * @param  isr_handler              gpio_isr_t          |   ISR handler function
 * @return void
 *
 * @note
 * @warning
 * @attention
 *
 * @authors Roshan Sundar
 * @date Updated: 11/7/2023
 */
void assign_interrupt(gpio_int_type_t interrupt_type, int pin_num, gpio_isr_t isr_handler)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};

    // interrupt of a specified type
    io_conf.intr_type = interrupt_type;

    // bit mask of the pin
    io_conf.pin_bit_mask = 1ULL << pin_num;

    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;

    // enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    gpio_config(&io_conf);

    // hook isr handler for a specific GPIO pin
    gpio_isr_handler_add(pin_num, isr_handler, (void*) pin_num);
}