/*********************************  Program Constants  *********************************/

//------------- FSR Constants ---------------//
#define FSR1_channel                ADC_CHANNEL_6   /*!< Attatched to GPIO 34 */
#define GPIO34                      34              /*!< GPIO 34 */
#define FSR2_channel                ADC_CHANNEL_7   /*!< Attatched to GPIO 35 */
#define GPIO35                      35              /*!< GPIO 35 */
#define FSR_Transistor_cntrl        32              /*!< GPIO 32 */
#define FSR_SAMPLE_NUM              10              /*!< humber of samples to read from FSR */

//------------- I2C Constants ---------------//
#define I2C_MASTER_SCL_IO           22              /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21              /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0               /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000            /*!< I2C master timeout in milliseconds */
#define AMG8833_ADDR                0x69            /*!< AMG8833 i2c address */

//------------- Coulomb Counting Constants ---------------//
#define GPIO33 33
#define AH_QUANTA 0.17067759
#define FULLY_CHARGED_BATTERY_mAH 2000


//------------- Delays and Times Constants ---------------//
#define FSR_DEBOUNCE_TIME           300             /*!< Time for FSR to debounce */
#define TASK_DELETE_WAIT_TIME       100             /*!< Time to wait after task is deleted */
#define FSR_SENSOR_SETTLE_TIME      3000            /*!< Time to wait for FSR sensor to settle */
#define FSR_READ_DELAY              100             /*!< Delay between FSR read operations */