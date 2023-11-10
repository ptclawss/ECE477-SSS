#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/i2c.h" 

//------------- Constant and Variable Defenitions ---------------//
// FSR Constants
#define FSR1_channel                ADC_CHANNEL_6   /*!< Attatched to GPIO 34 */
#define GPIO34                      34              /*!< GPIO 34 */
#define FSR2_channel                ADC_CHANNEL_7   /*!< Attatched to GPIO 35 */
#define GPIO35                      35              /*!< GPIO 35 */
#define FSR_Transistor_cntrl        32              /*!< GPIO 32 */
#define FSR_SAMPLE_NUM              10              /*!< humber of samples to read from FSR */

// I2C Constants
#define I2C_MASTER_SCL_IO           22              /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21              /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0               /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000            /*!< I2C master timeout in milliseconds */
#define AMG8833_ADDR                0x69            /*!< AMG8833 i2c address */

// Delays and Times
#define FSR_DEBOUNCE_TIME           300             /*!< Time for FSR to debounce */
#define TASK_DELETE_WAIT_TIME       100             /*!< Time to wait after task is deleted */
#define FSR_SENSOR_SETTLE_TIME      3000            /*!< Time to wait for FSR sensor to settle */
#define FSR_READ_DELAY              100             /*!< Delay between FSR read operations */

// Debug String
const static char * TAG = "OUTPUT";

// ADC Vars
adc_oneshot_unit_handle_t adc1_handle;              /*!< Handler for the adc1 unit */

// ISR Vars
int last_interrupt_time_ticks = 0;                              /*!< Time since last GPIO34 GPIO35 ISR interrupt in ticks */

// Occupancy Vars
volatile TaskHandle_t occupancy_update__task_handle = NULL;     /*!< Handler for the occupancy_update task */
bool restart_occupancy_update = false;                          /*!< Status flag for need to reset the occupancy_update task */

//------------- Function Declarations ---------------//
// ADC Functionality
adc_oneshot_unit_handle_t setup_ADC1();
adc_oneshot_unit_handle_t init_ADC1();
void config_ADC1_channels(adc_oneshot_unit_handle_t);
int read_ADC_channel(int, adc_oneshot_unit_handle_t, adc_channel_t);

// I2C Functionality
static esp_err_t i2c_master_init();
static esp_err_t i2c_register_read(uint8_t, uint8_t, uint8_t *, size_t);

// Interrupt Functionality
void assign_interrupt(gpio_int_type_t, int, gpio_isr_t);

// Occupancy Determination Functionality
static void occupancy_update_start(void*);
void resetchk_occupancy_update();
static void occupancy_update(void*);

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
static void IRAM_ATTR gpio_34_35_isr_handler(void* arg)
{
    // Get # of ticks since ISR started
    uint32_t current_time_ticks = xTaskGetTickCountFromISR();

    // If FSR has changed...
    if ((current_time_ticks - last_interrupt_time_ticks) > pdMS_TO_TICKS(FSR_DEBOUNCE_TIME)) // Debounce
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

void app_main(void)
{
    //------------- ADC Setup ---------------//
    adc1_handle = setup_ADC1();
    
    // Configure GPIO32 as output for transistor to control FSR circuitry
    gpio_set_direction(FSR_Transistor_cntrl, GPIO_MODE_OUTPUT);

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

    //------------- Main Loop ---------------//
    while (1)
    {
        // Turn FSR control transistor on
        gpio_set_level(FSR_Transistor_cntrl, 1);

        // Read AMG8833 registers
        //uint8_t data[1];
        //ESP_ERROR_CHECK(i2c_register_read(AMG8833_ADDR, 0xAA, data, 1));
        //ESP_LOGI(TAG, "Test = %X", data[0]);

        // Delay for 2s
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief ADC1 setup function
 *
 * Master function for ADC1 setup. References other helper functions below.
 * 
 * @return adc1_handle | adc_oneshot_unit_handle_t | Handler for ADC1
 *
 * @note 
 * @warning 
 * @attention 
 *
 * @authors Roshan Sundar
 * @date Updated: 11/1/2023
 */
adc_oneshot_unit_handle_t setup_ADC1()
{
    adc_oneshot_unit_handle_t adc1_handle = init_ADC1();
    config_ADC1_channels(adc1_handle);
    return adc1_handle;
}

/**
 * @brief Initialize ADC1
 *
 * Create a config and apply it to the ADC1 handler
 *
 * @return adc1_handle | adc_oneshot_unit_handle_t | Handler for ADC1
 *
 * @note Any additional notes or important considerations
 * @warning
 * @attention
 *
 * @authors Roshan Sundar
 * @date Updated: 11/1/2023
 */
adc_oneshot_unit_handle_t init_ADC1()
{
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    return adc1_handle;
}

/**
 * @brief Configure ADC1 channels
 *
 * ADC1 channels that will be used later on configured here.
 * Currently, FSR ADC pins config'd here.
 *
 * @param adc1_handle | adc_oneshot_unit_handle_t | Handler for ADC1
 * @return void
 *
 * @note Any additional notes or important considerations
 * @warning
 * @attention
 *
 * @authors Roshan Sundar
 * @date Updated: 11/1/2023
 */
void config_ADC1_channels(adc_oneshot_unit_handle_t adc1_handle)
{
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, FSR1_channel, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, FSR2_channel, &config));
}

/**
 * @brief Reads raw value of an ADC channel
 *
 * @param adc_unit      | int                       | Int constant for ADC1, ADC2, etc.
 * @param adc1_handle   | adc_oneshot_unit_handle_t | Handler for ADC1
 * @return adc_raw      | int                       | Raw value of ADC channel
 *
 * @note Comment out the log function to avoid printing values
 * @warning 
 * @attention 
 *
 * @authors Roshan Sundar
 * @date Updated: 11/1/2023
 */
int read_ADC_channel(int adc_unit, adc_oneshot_unit_handle_t adc_handle, adc_channel_t channel)
{
    static int adc_raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, channel, &adc_raw));
    return adc_raw;
}

/**
 * @brief Initialize ESP32 i2c
 *
 * @return err | esp_err_t | Error status
 *
 * @note Sets up i2c on this device only.
 * @warning Will not establish I2C connections.
 * @attention
 *
 * @authors Roshan Sundar
 * @date Updated: 11/3/2023
 */
static esp_err_t i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

/**
 * @brief Read a register from a device on i2c bus
 *
 * @param device_addr       uint8_t     |   Address of i2c device
 * @param reg_addr          uint8_t     |   Address of register on i2c device
 * @param data              uint8_t*    |   Pointer to read register into
 * @param len               size_t      |   length in bytes to be read
 * @return err | esp_err_t | Error status
 *
 * @note 
 * @warning @param lem is in bytes not bits
 * @attention 
 *
 * @authors Roshan Sundar
 * @date Updated: 11/3/2023
 */
static esp_err_t i2c_register_read(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, device_addr, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
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
static void occupancy_update_start(void* arg)
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

        // Run the occupancy_update_start task
        xTaskCreate(occupancy_update_start, "occupancy_update_start", 2048, NULL, 10, NULL);

        // Delete the current occupancy_update task
        TaskHandle_t temp_handle = occupancy_update__task_handle;
        occupancy_update__task_handle = NULL;
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
static void occupancy_update(void* arg)
{
    // Delay slightly once task is started for values to settle down
    ESP_LOGI(TAG, "Task Started!\n");
    vTaskDelay(pdMS_TO_TICKS(FSR_SENSOR_SETTLE_TIME));

    // Reset check
    resetchk_occupancy_update();

    // Sample the FSR values
    for(int i = 0; i < FSR_SAMPLE_NUM; i++)
    {
        // Reset check
        resetchk_occupancy_update();

        // Read FSRs
        int val = read_ADC_channel(ADC_UNIT_1, adc1_handle, FSR1_channel);
        printf("FSR1 val: %d\n", val);

        val = read_ADC_channel(ADC_UNIT_1, adc1_handle, FSR2_channel);
        printf("FSR2 val: %d\n", val);

        // Slight read delay
        vTaskDelay(pdMS_TO_TICKS(FSR_READ_DELAY));
    }

    // Delete the task when it's done
    ESP_LOGI(TAG, "Task Finished!\n");
    occupancy_update__task_handle = NULL;
    vTaskDelete(NULL);
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