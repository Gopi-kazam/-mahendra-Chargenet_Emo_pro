#include <esp_timer.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#define TX_PIN  GPIO_NUM_22
#define RX_PIN  GPIO_NUM_21
// #define LED_PIN GPIO_NUM_12

#define VCU_100 0x18FF0A81
#define RECTIFIER_DATA 0x18FF50E5

twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
// gpio_config_t led={
//     .pin_bit_mask = (1Ull<<LED_PIN),
//     .mode = GPIO_MODE_OUTPUT,
//     .pull_down_en = GPIO_PULLDOWN_DISABLE,
//     .pull_up_en = GPIO_PULLUP_DISABLE,
//     .intr_type = GPIO_INTR_DISABLE,
// };

QueueHandle_t transmitQueue = NULL;

double max_I_lt = 0, max_V_lt = 0;
double e_op_volt = 0, e_op_curr = 0;

uint8_t ch_enable;
uint8_t can_enable;

//CAN id
uint32_t CH1_id = 0x18FF0587;
uint32_t CH2_id = 0x18FF0687;
// uint32_t CH3_id = 0x18FF0887;
uint32_t bus_alert=0;
//Emo rx_id
uint32_t Emo_id = 0x1806E5F4;

int64_t lastVehicleCAN = 0;
int64_t lastEmoCAN = 0;

//helper function to set CAN tx
twai_message_t can_set(const uint32_t id)
{
    twai_message_t mesg_conf = {
        .extd = 1,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .identifier = id, //to emo
        .data_length_code = 8,
    };
    return mesg_conf;
}

// Task for vehicle related CAN
_Noreturn void vehicle_can_task(__unused void *params)
{
    while (1)
    {
        if (can_enable) //to check CAN enable or not 0x20
        {
            twai_message_t CN_tx2 = can_set(CH2_id); //setting MAX_curr&volt to ev //CH2
            CN_tx2.data[4] = (uint8_t)(0x32); //setting MAX curr(50A)
            CN_tx2.data[5] = (uint8_t)(0x3C); //setting MAX volt(60V)
            xQueueSend(transmitQueue, &CN_tx2, 0);

            uint16_t tx_e_op_curr = (uint16_t)(e_op_curr * 20);
            uint16_t tx_e_op_volt = (uint16_t)(e_op_volt * 20);

            twai_message_t CN_tx1 = can_set(CH1_id); //CH1 data to send ev
            CN_tx1.data[2]= tx_e_op_curr & 0xFF; //curr output intel
            CN_tx1.data[3]= tx_e_op_curr >> 8;
            CN_tx1.data[4]= tx_e_op_volt & 0xFF; //volt output intel
            CN_tx1.data[5]= tx_e_op_volt >> 8;
            CN_tx1.data[6] = (uint8_t)(0x10); //set mains available
            CN_tx1.data[7] = (uint8_t)(0x01); //set charge status
            xQueueSend(transmitQueue, &CN_tx1, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task for rectifier related CAN
_Noreturn void rectifier_can_task(__unused void *params)
{
    while (true)
    {
        bool rectifierEnable = true;


        if (max_I_lt < 1)
        {
            rectifierEnable = false;
        }

        if (max_V_lt < 35 || max_V_lt > 61)
        {
            rectifierEnable = false;
        }

        double tx_I_lt = max_I_lt > 50 ? 50 : max_I_lt;
        double tx_V_lt = max_V_lt > 58.8 ? 58.8 : max_V_lt;

        // printf("%f %f %f %f %d %d\n",max_V_lt, max_I_lt, tx_V_lt, tx_I_lt, rectifierEnable, ch_enable);

        twai_message_t rectifierMessage = can_set(Emo_id);
        // rectifierMessage.data_length_code = 5;

        if (rectifierEnable && ch_enable) {
            uint16_t tx_V_raw = (uint16_t)(tx_V_lt * 10);
            uint16_t tx_I_raw = (uint16_t)(tx_I_lt * 10);
            rectifierMessage.data[0] = (tx_V_raw >> 8) & 0xFF;
            rectifierMessage.data[1] = tx_V_raw & 0xFF;
            rectifierMessage.data[2] = (tx_I_raw >> 8) & 0xFF;
            rectifierMessage.data[3] = tx_I_raw & 0xFF;
            rectifierMessage.data[4] = 0x00;
            
        }
        else
        {
            rectifierMessage.data[4] = 1;
           
        }

        xQueueSend(transmitQueue, &rectifierMessage, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task to handle transmitting and receiving CAN data
_Noreturn void can_task(__unused void *params)
{
    twai_message_t CN_rx1;
    while (1)
    {
        if (twai_receive(&CN_rx1, 10 / portTICK_PERIOD_MS) == ESP_OK)
        {
            switch (CN_rx1.identifier) {
                case VCU_100:
                {
                    can_enable = (CN_rx1.data[4] & BIT5) != 0;
                    ch_enable = (CN_rx1.data[4] & (BIT0 | BIT1)) == 1;
                    const uint16_t raw_max_I_lt = (uint16_t)(CN_rx1.data[0] | (CN_rx1.data[1] << 8)); //stored lsb first and msb
                    max_I_lt = raw_max_I_lt * 0.5;
                    const uint16_t raw_max_V_lt = (uint16_t)(CN_rx1.data[2] | (CN_rx1.data[3] << 8));
                    max_V_lt = raw_max_V_lt * 0.01;
                    lastVehicleCAN = esp_timer_get_time();
                    break;
                }

                case RECTIFIER_DATA:
                {
                    const uint16_t raw_e_op_volt = (uint16_t)(CN_rx1.data[0] << 8 | CN_rx1.data[1]);
                    e_op_volt = raw_e_op_volt * 0.1;
                    const uint16_t raw_e_op_curr = (uint16_t)(CN_rx1.data[2] << 8 | CN_rx1.data[3]);
                    e_op_curr = raw_e_op_curr * 0.1;
                    lastEmoCAN = esp_timer_get_time();
                    break;
                }

                default:
                {
                    break;
                }
            }
        }

        twai_message_t transmitMessage;
        while (xQueueReceive(transmitQueue, &transmitMessage, 0) == pdTRUE)
        {
            esp_err_t err = twai_transmit(&transmitMessage, 0);


            // Handler for CAN bus failure
            twai_reconfigure_alerts(TWAI_ALERT_BUS_OFF|TWAI_ALERT_BUS_RECOVERED,NULL);
            if (twai_read_alerts(&bus_alert,portMAX_DELAY)==ESP_OK)
            {
                if (bus_alert & TWAI_ALERT_BUS_OFF)
                {
                    twai_stop();
                    twai_initiate_recovery();

                }else if (bus_alert & TWAI_ALERT_BUS_RECOVERED)
                {
                    twai_start();
                }

            }
        }

        // Reset vehicle parameters if absent
        if (esp_timer_get_time() - lastVehicleCAN > 1500000)
        {
            can_enable = 0;
            max_I_lt = 0;
            max_V_lt = 0;
            ch_enable = 0;
        }

        // Reset rectifier parameters if absent
        if (esp_timer_get_time() - lastEmoCAN > 5000000)
        {
            e_op_volt = 0;
            e_op_curr = 0;
        }
    }
}

_Noreturn void app_main(void)
{
    gpio_config(&led);
    transmitQueue = xQueueCreate(10, sizeof(twai_message_t));
    ESP_ERROR_CHECK(twai_driver_install(&g_config,&t_config,&f_config));
    ESP_ERROR_CHECK(twai_start());

    xTaskCreate(rectifier_can_task, "r_can", 4096, NULL, 2, NULL);
    xTaskCreate(vehicle_can_task, "v_can", 4096, NULL, 2, NULL);
    xTaskCreate(can_task, "task", 4096, NULL, 2, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
