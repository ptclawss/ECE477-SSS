//------------- Imports ---------------//
// Standard Libraries
#include "esp_adc/adc_oneshot.h"

// Custom Libraries
#include "constants.h"

//------------- Global Variables ---------------//
extern adc_oneshot_unit_handle_t adc1_handle;               /*!< Handler for the adc1 unit */

//------------- Function Declarations ---------------//
adc_oneshot_unit_handle_t setup_ADC1();
adc_oneshot_unit_handle_t init_ADC1();
void config_ADC1_channels(adc_oneshot_unit_handle_t);
int read_ADC_channel(int, adc_oneshot_unit_handle_t, adc_channel_t);