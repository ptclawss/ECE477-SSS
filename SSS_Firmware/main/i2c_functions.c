//------------- Imports ---------------//
#include "i2c_functions.h"

//------------- Local Variables ---------------//

//------------- Functions ---------------//
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
esp_err_t i2c_master_init()
{
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
esp_err_t i2c_register_read(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, device_addr, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t readThermalArray(float * thermalData) {
    uint8_t rawData[PIXELS_REG_NUM]; // 8x8 array, each element is two bytes

    // Read 128 bytes (8x8 array) starting from register 0x80
    esp_err_t amg8833_err = i2c_register_read(AMG8833_ADDR, 0x80, rawData, PIXELS_REG_NUM);
    if (amg8833_err == ESP_OK)
    {
        vAMG_PUB_TMP_ConvTemperature64(rawData, thermalData);
    }
    
    return amg8833_err;
}

short shAMG_PUB_TMP_ConvTemperature(unsigned char aucRegVal[2])
{
	short shVal = ((short)(aucRegVal[1] & 0x07) << 8) | aucRegVal[0];

	if (0 != (0x08 & aucRegVal[1]))
	{
		shVal -= 2048;
	}

	shVal *= 64;

	return (shVal);
}

void vAMG_PUB_TMP_ConvTemperature64(unsigned char *pucRegVal, float *pshVal)
{
	unsigned char ucCnt = 0;

	for (ucCnt = 0; ucCnt < PIXELS_NUM; ucCnt++)
	{
		short norm_val = shAMG_PUB_TMP_ConvTemperature(pucRegVal + ucCnt * 2);
        pshVal[ucCnt] = (float) norm_val / CONVERSION_CONSTANT_C;
	}
}