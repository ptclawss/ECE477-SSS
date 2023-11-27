//------------- Imports ---------------//
// Standard Libraries
#include "driver/i2c.h"
#include "esp_log.h"

// Custom Libraries
#include "constants.h"

//------------- Global Variables ---------------//

//------------- Function Declarations ---------------//
esp_err_t i2c_master_init();
esp_err_t i2c_register_read(uint8_t, uint8_t, uint8_t *, size_t);
void readThermalArray(float *);
short shAMG_PUB_TMP_ConvTemperature(unsigned char aucRegVal[2]);
void vAMG_PUB_TMP_ConvTemperature64(unsigned char *, float *);