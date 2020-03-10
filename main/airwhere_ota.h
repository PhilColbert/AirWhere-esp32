#ifndef AIRWHERE_OTA_H
#define AIRWHERE_OTA_H

/*
static esp_err_t event_handler(void *ctx, system_event_t *event);
static bool read_past_http_header(char text[], int total_len, esp_ota_handle_t update_handle);
static bool connect_to_http_server();
static void __attribute__((noreturn)) task_fatal_error();
static void ota_example_task(void *pvParameter);
*/
void update_airwhere(bool airwhere_live);


#endif /* OTA_H */

