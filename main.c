#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include <esp_wifi.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>

#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <esp_log_internal.h>
#include <stdlib.h>
#include "pn532.h"
#include "driver/gpio.h"
#include "protobuf/proto-c/custom_config.pb-c.h"

esp_mqtt_client_handle_t client;
EventGroupHandle_t wifi_event_group;
int angle;
int angle2;
bool provisioned;
bool wificonnected;
bool auth_servo = false;
const int WIFI_CONNECTED_EVENT = BIT0;
const int WIFI_PROV_TIMEOUT_EVENT = BIT1;
const int WIFI_PROV_DONE = BIT2;
static const char *TAG = "example";
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void event_handler(void *arg, esp_event_base_t event_base,
                   int32_t event_id, void *event_data);
static void mqtt5_app_start(void);
void init_wifi(void);
void mqtt_all_data();
void servo_go(void *somtehinh);
static inline uint32_t example_angle_to_compare(int angle);
void nfc_task(void *pvParameter);
static pn532_t nfc;
uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
void gpio_boot(void *pv);
void gpio_init();
void blink();
TaskHandle_t blinkHandle = NULL;

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE -90         // Minimum angle
#define SERVO_MAX_DEGREE 90          // Maximum angle

#define SERVO_PULSE_GPIO 26                  // GPIO connects to the PWM signal line
#define SERVO2_GPIO 25                       // GPIO FOR SERVO2
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms
const uint8_t play_uid[4] = {0xED, 0x77, 0x10, 0x2F};

void app_main(void)
{
    gpio_init();
    init_wifi();
    ESP_LOGI(TAG, "Initialising MQTT from Main.");
    mqtt5_app_start();
    //xTaskCreate(mqtt_all_data, "mqtt", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    xTaskCreate(gpio_boot, "boot", configMINIMAL_STACK_SIZE, NULL, 10, NULL);
    ESP_LOGI(TAG, "Initialization completed.");
    xTaskCreate(nfc_task, "nfc_task", 4096, NULL, 4, NULL);
    xTaskCreate(servo_go, "servo", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    vTaskDelete(NULL);
}

void gpio_boot(void *pv)
{

    while (1)
    {
        if (gpio_get_level(GPIO_NUM_0) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(3000));
            if (gpio_get_level(GPIO_NUM_0) == 00)
            {

                ESP_ERROR_CHECK(nvs_flash_erase());

                esp_restart();
            }
        }
    }
}

void gpio_init(void)
{
    gpio_config_t boot_button = {
        .pin_bit_mask = BIT64(GPIO_NUM_0),
        .mode = GPIO_MODE_INPUT,
        //.pull_up_en = GPIO_PULLUP_ENABLE,

    };
    gpio_config(&boot_button);

    gpio_config_t led_pin = {
        .pin_bit_mask = BIT64(GPIO_NUM_2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led_pin);
}

void blink()
{
    while (1)
    {
        if (auth_servo)
        {
            gpio_set_level(GPIO_NUM_2, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            gpio_set_level(GPIO_NUM_2, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(GPIO_NUM_2, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

void nfc_task(void *pvParameter)
{
    pn532_spi_init(&nfc, 21, 23, 22, 19);
    pn532_begin(&nfc);

    uint32_t versiondata = pn532_getFirmwareVersion(&nfc);
    if (!versiondata)
    {
        ESP_LOGI(TAG, "Didn't find PN53x board");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    // Got ok data, print it out!
    // ESP_LOGI(TAG, "Found chip PN5 %x", (versiondata >> 24) & 0xFF);
    // ESP_LOGI(TAG, "Firmware ver. %d.%d", (versiondata >> 16) & 0xFF, (versiondata >> 8) & 0xFF);

    // configure board to read RFID tags
    pn532_SAMConfig(&nfc);

    ESP_LOGI(TAG, "Waiting for an ISO14443A Card ...");

    while (1)
    {
        uint8_t success;
        // uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0}; // Buffer to store the returned UID
        uint8_t uidLength; // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

        uint8_t data_read[16];
        memset(data_read, 0, sizeof(data_read));
        // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
        // 'uid' will be populated with the UID, and uidLength will indicate
        // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
        success = pn532_readPassiveTargetID(&nfc, PN532_MIFARE_ISO14443A, uid, &uidLength, 0);

        if (success)
        {
            // Display some basic information about the card
            ESP_LOGI(TAG, "Found an ISO14443A card");
            ESP_LOGI(TAG, "UID Length: %d bytes", uidLength);
            ESP_LOGI(TAG, "UID Value:");
            esp_log_buffer_hexdump_internal(TAG, uid, uidLength, ESP_LOG_INFO);

            if (memcmp(play_uid, uid, 4) == 0)
            {
                auth_servo = true;
            }
            else
            {
                auth_servo = false;
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            // PN532 probably timed out waiting for a card
            ESP_LOGI(TAG, "Timed out waiting for a card");
        }
    }
}

void servo_go(void *pv)
{
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_handle_t timer2 = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer2)); // new

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    // new

    mcpwm_oper_handle_t oper2 = NULL;
    mcpwm_operator_config_t operator_config2 = {
        .group_id = 1, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper2));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, timer2));
    // done

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_cmpr_handle_t comparator2 = NULL; // newly added
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };

    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator2)); // added another comparator for 2nd servo

    mcpwm_gen_handle_t generator2 = NULL; // new generator  ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    mcpwm_generator_config_t generator2_config = {
        .gen_gpio_num = SERVO2_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator2_config, &generator2)); // new generator

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(0))); // new
    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    // new
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator2,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator2,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator2, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer2)); // new
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer2, MCPWM_TIMER_START_NO_STOP));

    while (1)
    {

        ESP_LOGI(TAG, "Angle of rotation of Servo1: %d", angle);
        ESP_LOGI(TAG, "Angle of rotation of Servo2: %d", angle2);
        // auth_servo=true;        // temporary
        if (auth_servo)
        {
           
            if ((angle >= -84 && angle <= 84))
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            if ((angle2 >= -84 && angle2 <= 84))
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(angle2)));
            }
        }
        else
        {
            ESP_LOGW(TAG, "Did not recv auth");
            vTaskDelay(pdMS_TO_TICKS(1500));
        }
    }
}

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

static void mqtt5_app_start(void)
{
    esp_mqtt_client_config_t mqtt5_cfg = {
        .broker.address.uri = "mqtt://mqtt.eclipseprojects.io",
        .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1,
        .network.disable_auto_reconnect = true,
        .task.priority =10,
    };

    client = esp_mqtt_client_init(&mqtt5_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void mqtt_all_data()
{
    while (1)
    {
        char angle_buff[10];
        sprintf(angle_buff, "%d", angle);
        // ESP_LOGI(TAG, "angle: %d", angle);
        esp_mqtt_client_publish(client, "krishnapc/LIPL/Servo", angle_buff, 0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{

    // int msg_id;
    esp_mqtt_event_handle_t event = event_data; // here esp_mqtt_event_handle_t is a struct which receieves struct event from mqtt app start funtion
    // esp_mqtt_client_handle_t client = event->client; // making obj client of struct esp_mqtt_client_handle_t and assigning it the receieved event client

    if (event->event_id == MQTT_EVENT_CONNECTED)
    {
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        xTaskCreate(blink, "blinkit", configMINIMAL_STACK_SIZE, NULL, 5, &blinkHandle);

        // msg_id = esp_mqtt_client_subscribe(client, "/krishnapc/LIPL/#", 0);
        //  in mqtt we require a topic to subscribe and client is from event client and 0 is quality of service it can be 1 or 2
        // ESP_LOGI(TAG, "sent subscribe successful");

        esp_mqtt_client_subscribe(client, "krishnapc/LIPL/Pitch", 1);
        esp_mqtt_client_subscribe(client, "krishnapc/LIPL/Roll", 1);
        // esp_mqtt_client_subscribe(client, "krishnapc/LIPL/Roll", 1);

        // esp_mqtt_client_subscribe(client, "/krishnapc/LIPL/Time", 0);
    }
    else if (event->event_id == MQTT_EVENT_DISCONNECTED)
    {
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED"); // if disconnected
        if (blinkHandle != NULL)
        {
            vTaskDelete(blinkHandle);
        }

        gpio_set_level(GPIO_NUM_2, 0);
    }
    else if (event->event_id == MQTT_EVENT_SUBSCRIBED)
    {
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
    }
    else if (event->event_id == MQTT_EVENT_UNSUBSCRIBED) // when subscribed
    {
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED");
    }
    else if (event->event_id == MQTT_EVENT_DATA) // when unsubscribed
    {
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // ESP_LOGI(TAG, "%.*s", event->topic_len, event->topic);
        int topic_len = event->topic_len;
        // Printing topic

        // printing data
        if (strncmp(event->topic, "krishnapc/LIPL/Pitch", topic_len) == 0)
        {

            int datalen = event->data_len;
            // ESP_LOGI(TAG, "%.*s", datalen, event->data);
            char ev_databuff[datalen + 1];
            strncpy(ev_databuff, event->data, datalen);
            ev_databuff[datalen] = '\0';
            angle = atoi(ev_databuff);
            ESP_LOGI(TAG, "value of angle after atoi: %d", angle);
        }
        else if (strncmp(event->topic, "krishnapc/LIPL/Roll", topic_len) == 0)
        {

            int datalen = event->data_len;
            // ESP_LOGI(TAG, "%.*s", datalen, event->data);
            char ev_databuff[datalen + 1];
            strncpy(ev_databuff, event->data, datalen);
            ev_databuff[datalen] = '\0';
            angle2 = atoi(ev_databuff);
            ESP_LOGI(TAG, "value of angle2 after atoi: %d", angle2);
        }
    }
    else if (event->event_id == MQTT_EVENT_ERROR) // when any error
    {
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    }
}

/**
 * @brief Initialises WiFi and Provisioning system.
 *
 */

void init_wifi()
{
    char *TAG = "init_wifi";
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    esp_netif_create_default_wifi_sta();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // This config requires NVS initialized
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_prov_mgr_config_t config = {
        /* What is the Provisioning Scheme that we want ?
         * wifi_prov_scheme_softap or wifi_prov_scheme_ble */
        .scheme = wifi_prov_scheme_ble,

        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM};
    /* Initialize provisioning manager with the
     * configuration parameters set above */
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    /* Let's find out if the device is provisioned */
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));
    // esp_wifi_get_mac(WIFI_IF_STA, mac_addr); // Get WiFi Mac Address

    /* If device is not yet provisioned start provisioning service */
    if (!provisioned)
    {
        ESP_LOGI(TAG, "Starting provisioning");
        /* What is the Device Service Name that we want
         * This translates to :
         *     - Wi-Fi SSID when scheme is wifi_prov_scheme_softap
         *     - device name when scheme is wifi_prov_scheme_ble
         */
        char service_name[] = "ESP- Servo";
        const char *pop = "LIPL24";
        const char *service_key = NULL;

        wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
        /* This step is only useful when scheme is wifi_prov_scheme_ble. This will
         * set a custom 128 bit UUID which will be included in the BLE advertisement
         * and will correspond to the primary GATT service that provides provisioning
         * endpoints as GATT characteristics. Each GATT characteristic will be
         * formed using the primary service UUID as base, with different auto assigned
         * 12th and 13th bytes (assume counting starts from 0th byte). The client side
         * applications must identify the endpoints by reading the User Characteristic
         * Description descriptor (0x2901) for each characteristic, which contains the
         * endpoint name of the characteristic */
        uint8_t custom_service_uuid[] = {
            /* LSB <---------------------------------------
             * ---------------------------------------> MSB */
            0xb4,
            0xdf,
            0x5a,
            0x1c,
            0x3f,
            0x6b,
            0xf4,
            0xbf,
            0xea,
            0x4a,
            0x82,
            0x03,
            0x04,
            0x90,
            0x1a,
            0x02,
        };
        wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);

        // wifi_prov_mgr_endpoint_create("custom-config"); // Keep this above wifi_prov_start_provisioning()
        // wifi_prov_mgr_endpoint_create("ble-communication");

        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, pop, service_name, service_key));
        // wifi_prov_mgr_endpoint_register("custom-config", custom_prov_config_data_handler, NULL);
        // wifi_prov_mgr_endpoint_register("ble-communication", ble_command_data_handler, NULL);
    }
    else
    {
        ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");
        /* We don't need the manager as device is already provisioned,
         * so let's release it's resources */
        wifi_prov_mgr_deinit();

        /* Start Wi-Fi station */
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        xEventGroupSetBits(wifi_event_group, WIFI_PROV_DONE); // let app_main() know that wifi prov is ok
    }
    /* Initialise SNTP */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, pdTRUE, pdFAIL, portMAX_DELAY);
    return;
}

/**
 * @brief WiFi and WiFi Provisioning Event handler.
 *
 * @param arg           Void ptr
 * @param event_base    Event Base type
 * @param event_id      Event ID
 * @param event_data    Data (void ptr)
 */
void event_handler(void *arg, esp_event_base_t event_base,
                   int32_t event_id, void *event_data)
{
    char *TAG = "event_handler";
    if (event_base == WIFI_PROV_EVENT)
    {
        switch (event_id)
        {
        case WIFI_PROV_START:
            ESP_LOGI(TAG, "Provisioning started");
            break;
        case WIFI_PROV_CRED_RECV:
        {
            wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
            ESP_LOGI(TAG, "Received Wi-Fi credentials"
                          "\n\tSSID     : %s\n\tPassword : %s",
                     (const char *)wifi_sta_cfg->ssid,
                     (const char *)wifi_sta_cfg->password);
            break;
        }
        case WIFI_PROV_CRED_FAIL:
        {
            wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
            ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                          "\n\tPlease reset to factory and retry provisioning",
                     (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
            ESP_ERROR_CHECK(nvs_flash_erase());
            esp_restart();
            break;
        }
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "Provisioning successful");
            xEventGroupSetBits(wifi_event_group, WIFI_PROV_DONE); // let app_main() know that wifi prov is ok
            break;
        case WIFI_PROV_END:
            /* De-initialize manager once provisioning is finished */
            wifi_prov_mgr_deinit();
            break;
        default:
            break;
        }
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {

        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
        wificonnected = true;
        /* Signal main application to continue execution */

        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {

        ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
        wificonnected = false;
        esp_wifi_connect();
    }
}
