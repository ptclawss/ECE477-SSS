//------------- Imports ---------------//
#include "coulomb_counting.h"

//------------- Local Variables ---------------//
volatile double battery_mAh = 2000.0;
volatile double battery_percent = 100.0;

volatile long int _time, lasttime;
volatile double mA;
double percent_quanta;

void IRAM_ATTR coulomb_counter_isr_handler(void *arg)
{
    lasttime = _time;
    _time = esp_timer_get_time();

    battery_mAh -= AH_QUANTA;
    battery_percent -= percent_quanta;

    mA = 614.4 / ((_time - lasttime) / 1000000.0);

    xTaskCreate(testTask, "testTask", 2048, NULL, 10, NULL);
}

void initBatteryInfo(volatile double init_battery_mAH)
{
    battery_mAh = init_battery_mAH;
    percent_quanta = 1.0 / (battery_mAh / 1000.0 * 5859.0 / 100.0);
}