//------------- Imports ---------------//
#include "adc_functions.h"

//------------- Local Variables ---------------//

//------------- Functions ---------------//

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