//------------- Imports ---------------//
// Standard Libraries
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"

// Custom Libraries
#include "constants.h"

//------------- Global Variables ---------------//


//------------- Function Declarations ---------------//
void wifi_event_handler(void *, esp_event_base_t , int32_t , void *);
void wifi_connection();
esp_err_t client_event_post_handler(esp_http_client_event_handle_t);
void post_rest_function(char *, char *, float, float, float);