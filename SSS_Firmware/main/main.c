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
#define FSR1_channel            ADC_CHANNEL_6       /*!< GPIO 34 */
#define FSR2_channel            ADC_CHANNEL_7       /*!< GPIO 35 */
#define FSR_Transistor_cntrl    32                  /*!< GPIO 32 */

#define I2C_MASTER_SCL_IO           22              /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21              /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0               /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000            /*!< I2C master timeout in milliseconds */
#define AMG8833_ADDR 0x69                           /*!< AMG8833 i2c address */

const static char * TAG = "OUTPUT";

//------------- Function Declarations ---------------//
adc_oneshot_unit_handle_t setup_ADC1();
adc_oneshot_unit_handle_t init_ADC1();
void config_ADC1_channels(adc_oneshot_unit_handle_t);
int read_ADC_channel(int, adc_oneshot_unit_handle_t, adc_channel_t);
static esp_err_t i2c_master_init();
static esp_err_t i2c_register_read(uint8_t, uint8_t, uint8_t *, size_t);

void app_main(void)
{
    //------------- ADC Setup ---------------//
    //adc_oneshot_unit_handle_t adc1_handle = setup_ADC1();
    
    // Configure GPIO32 as output for transistor to control FSR circuitry
    //gpio_set_direction(FSR_Transistor_cntrl, GPIO_MODE_OUTPUT);

    //------------- I2C Init ---------------//
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization error: %s", esp_err_to_name(err));
        return;
    }
    
    //------------- Main Loop ---------------//
    while (1)
    {
        // Turn FSR control transistor on
        //gpio_set_level(FSR_Transistor_cntrl, 1);
        
        // Read FSR values
        //int val = read_ADC_channel(ADC_UNIT_1, adc1_handle, FSR1_channel);
        //val = read_ADC_channel(ADC_UNIT_1, adc1_handle, FSR2_channel);

        // Read AMG8833 registers
        //uint8_t data[1];
        //ESP_ERROR_CHECK(i2c_register_read(AMG8833_ADDR, 0xAA, data, 1));
        //ESP_LOGI(TAG, "Test = %X", data[0]);

        // Delay for 1s
        vTaskDelay(pdMS_TO_TICKS(2000));
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
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", adc_unit + 1, channel, adc_raw);
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
 * @param device_addr       uint8_t     | Address of i2c device
 * @param reg_addr          uint8_t     | Address of register on i2c device
 * @param data              uint8_t*    | Pointer to read register into
 * @param len               size_t      | length in bytes to be read
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