#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#define TX_PIN  GPIO_NUM_22
#define RX_PIN  GPIO_NUM_21
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
//task var
BaseType_t st;
TaskHandle_t task_handle, ch12_task_handle;

uint16_t raw_max_I_lt = 0, raw_max_V_lt = 0;
uint16_t max_I_lt = 0, max_V_lt = 0;
uint16_t tx_max_I_lt = 0, tx_max_V_lt = 0;
uint8_t raw_ch_enable, ch_enable, tx_ch_enable;
uint8_t ch_mode_rq;
uint8_t emry_shutd;
uint8_t can_enable;
uint8_t hv_cont_st;
//rx var from emo
uint16_t raw_e_op_volt = 0, raw_e_op_curr = 0;
uint16_t e_op_volt = 0, e_op_curr = 0;
uint16_t tx_e_op_volt = 0, tx_e_op_curr = 0;

//tx_var
uint8_t tx_data[8];
//CAN id
uint32_t CH1_id = 0x18FF0587;
uint32_t CH2_id = 0x18FF0687;
uint32_t CH3_id = 0x18FF0887;

//Emo rx_id
uint32_t Emo_id = 0x1806E5F4;

int f = 0;
//helper function to set CAN tx
twai_message_t can_set(uint32_t* id)
{
    twai_message_t mesg_conf = {
        .extd = 1,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .identifier = *id, //to emo
        .data_length_code = 8,
    };
    return mesg_conf;
}

//ch12 task
_Noreturn void Ch12_task_handler(__unused void *params)
{
    while (1)
    {
        // printf("inside id\r\n");

        if (f) //to check CAN enable or not 0x20
        {
            printf("taskch12_inside can enb\r\n");
            twai_message_t CN_tx2 = can_set(&CH2_id); //setting MAX_curr&volt to ev //CH2
            memset(tx_data, 0, 8);
            //intel
            // tx_data[3]=(uint8_t)(0x1400);//set AC curr as 2A
            // tx_data[6]=(uint8_t)(0xE600); //set AC volt as 230v
            tx_data[4] = (uint8_t)(0x23); //setting MAX curr(50A)
            tx_data[5] = (uint8_t)(0x3C); //setting MAX volt(58.8v)
            memcpy(CN_tx2.data, tx_data, 8);
            if (twai_transmit(&CN_tx2,portMAX_DELAY) == ESP_OK) //to send CH2
            {
                printf("DATA_TO_CN_MAX_C&V_SET\r\n");
            }

            twai_message_t CN_tx1 = can_set(&CH1_id); //CH1 data to send ev
            memset(tx_data, 0, 8);
            // //intel
            //
            // tx_data[2]=(uint8_t) (0x2C);       //15A set init stage    //(tx_e_op_curr&0xFF); //curr output intel
            // tx_data[3]=(uint8_t) (0x01);                               //(tx_e_op_curr>>8);
            // tx_data[4]=(uint8_t) (0x44);       //58v set init stage    //(tx_e_op_volt&0xFF); //volt output intel
            // tx_data[5]=(uint8_t) (0x02);              //(tx_e_op_volt>>8);
            tx_data[6] = (uint8_t)(0x10); //set mains available
            tx_data[7] = (uint8_t)(0x01); //set charge status
            memcpy(CN_tx1.data, tx_data, 8);
            if (twai_transmit(&CN_tx1,portMAX_DELAY) == ESP_OK)
            {
                printf("DATA_TO_CN_TRANSFERRED\r\n");
            }
        }
        vTaskDelay(10);
    }
}

//task
_Noreturn void can_task(void*)
{
    while (1)
    {
        twai_message_t CN_rx1;
        if (twai_receive(&CN_rx1,portMAX_DELAY) == ESP_OK)
        {
            printf("data received\r\n");
            if (CN_rx1.identifier == 0x18FF0A81)
            {
                printf("inside id\r\n");
                can_enable = CN_rx1.data[4];
                if (can_enable & (1 << 5)) //to check CAN enable or not 0x20
                {
                    f = 1;
                    // printf("inside can enb\r\n");
                    // twai_message_t CN_tx2=can_set(&CH2_id); //setting MAX_curr&volt to ev //CH2
                    // memset(tx_data,0,8);
                    // //intel
                    // // tx_data[3]=(uint8_t)(0x1400);//set AC curr as 2A
                    // // tx_data[6]=(uint8_t)(0xE600); //set AC volt as 230v
                    // tx_data[4]= (uint8_t)(0x23); //setting MAX curr(50A)
                    // tx_data[5]= (uint8_t)(0x3C); //setting MAX volt(58.8v)
                    // memcpy(CN_tx2.data,tx_data,8);
                    // if (twai_transmit(&CN_tx2,portMAX_DELAY)==ESP_OK) //to send CH2
                    // {
                    //     printf("DATA_TO_CN_MAX_C&V_SET\r\n");
                    // }
                    //
                    // twai_message_t CN_tx1=can_set(&CH1_id); //CH1 data to send ev
                    // memset(tx_data,0,8);
                    // // //intel
                    // //
                    // // tx_data[2]=(uint8_t) (0x2C);       //15A set init stage    //(tx_e_op_curr&0xFF); //curr output intel
                    // // tx_data[3]=(uint8_t) (0x01);                               //(tx_e_op_curr>>8);
                    // // tx_data[4]=(uint8_t) (0x44);       //58v set init stage    //(tx_e_op_volt&0xFF); //volt output intel
                    // // tx_data[5]=(uint8_t) (0x02);              //(tx_e_op_volt>>8);
                    // tx_data[6]=(uint8_t)(0x10); //set mains available
                    // tx_data[7]=(uint8_t)(0x01);//set charge status
                    // memcpy(CN_tx1.data,tx_data,8);
                    // if (twai_transmit(&CN_tx1,portMAX_DELAY)==ESP_OK)
                    // {
                    //     printf("DATA_TO_CN_TRANSFERRED\r\n");
                    // }
                }
            }
            ///////
            //check charge enable
            ch_enable = CN_rx1.data[4];
            printf("ch_enable data:%x\r\n", ch_enable);
            if (ch_enable & (1 << 0)) //ch_enable&(1<<0))
            {
                //read I from Ev
                raw_max_I_lt = (uint16_t)(CN_rx1.data[0] | (CN_rx1.data[1] << 8)); //stored lsb first and msb
                printf("raw I data:%x\r\n", raw_max_I_lt);
                // max_I_lt=0; //clean
                max_I_lt = raw_max_I_lt * 0.5; //eng data
                printf("read I data:%d\r\n", max_I_lt);
                if (max_I_lt < 1) {
                    max_I_lt = 0;
                }
                if (max_I_lt > 50){ max_I_lt = 50;}
                // tx_max_I_lt=0;  //clean
                tx_max_I_lt = (uint16_t)(max_I_lt / 0.1);
                printf("emo raw I:%x\r\n", tx_max_I_lt);
                //read V from ev
                raw_max_V_lt = (uint16_t)(CN_rx1.data[2] | (CN_rx1.data[3] << 8));
                printf("raw V:%x\r\n", raw_max_V_lt);
                // max_V_lt=0; ////clean
                max_V_lt = raw_max_V_lt * 0.01;
                printf("read V:%d\r\n", max_V_lt);
                if (max_V_lt < 35) max_V_lt = 35;
                if (max_V_lt > 58.8) max_V_lt = 58.8;
                // tx_max_V_lt=0; //clean
                tx_max_V_lt = (uint16_t)(max_V_lt / 0.1);
                printf("emo raw V:%x\r\n", tx_max_V_lt);
                //////
                //send msg take from ev is to emo
                twai_message_t EM_tx = can_set(&Emo_id); //send to emo
                memset(tx_data, 0, 8);
                //motorola
                tx_data[0] = (uint8_t)(tx_max_V_lt >> 8); //msb first
                tx_data[1] = (uint8_t)(tx_max_V_lt & 0xFF); //lsb
                tx_data[2] = (uint8_t)(tx_max_I_lt >> 8);
                tx_data[3] = (uint8_t)(tx_max_I_lt & 0xFF);
                tx_data[4] = 0; //control bit
                memcpy(EM_tx.data, tx_data, 8);
                if (twai_transmit(&EM_tx,portMAX_DELAY) == ESP_OK)
                {
                    printf("DATA_SEND_TO_EMO\r\n");
                }
                ///////
                //data rx from emo
                twai_message_t EM_rx; //data rx from emo
                if (twai_receive(&EM_rx,portMAX_DELAY) == ESP_OK)
                {
                    if (EM_rx.identifier == 0x18FF50E5)
                    {
                        // raw_e_op_volt=0; //clean prev
                        raw_e_op_volt = (uint16_t)(EM_rx.data[0] << 8 | EM_rx.data[1]);
                        // e_op_volt=0; //clean
                        e_op_volt = raw_e_op_volt * 0.1;
                        printf("eng data volt:%d\r\n", e_op_volt);
                        // tx_e_op_volt = e_op_volt * 0.1; //to convert data for EV
                        // tx_e_op_volt =0;
                        tx_e_op_volt = e_op_volt / 0.1;
                        printf("raw_V_to_EV:%x\r\n", tx_e_op_volt);
                        raw_e_op_curr = (uint16_t)(EM_rx.data[2] << 8 | EM_rx.data[3]);
                        e_op_curr = raw_e_op_curr * 0.1;
                        printf("eng data curr:%d\r\n", e_op_curr);
                        // tx_e_op_curr = e_op_curr * 0.05;
                        tx_e_op_curr = 0;
                        tx_e_op_curr = e_op_curr / 0.05;
                        printf("raw_I_to_EV:%x\r\n", tx_e_op_curr);
                        ///////////
                        //data take from emo again send to EV (CH1)
                        twai_message_t CN_tx3 = can_set(&CH1_id); //tx data from emo to ev(CH1)
                        memset(tx_data, 0, 8);
                        //intel
                        tx_data[2] = (uint8_t)(tx_e_op_curr & 0xFF); //curr output intel
                        tx_data[3] = (uint8_t)(tx_e_op_curr >> 8);
                        tx_data[4] = (uint8_t)(tx_e_op_volt & 0xFF); //volt output intel
                        tx_data[5] = (uint8_t)(tx_e_op_volt >> 8);
                        // tx_data[2]=(uint8_t) (0x2C);       //15A set init stage    //(tx_e_op_curr&0xFF); //curr output intel
                        // tx_data[3]=(uint8_t) (0x01);               //(tx_e_op_curr>>8);
                        // tx_data[4]=(uint8_t) (0x44);        //58v set init stage       //(tx_e_op_volt&0xFF); //volt output intel
                        // tx_data[5]=(uint8_t) (0x02);              //(tx_e_op_volt>>8);
                        tx_data[6] = (uint8_t)(0x10); //set mains available
                        tx_data[7] = (uint8_t)(0x01); //set charge status
                        memcpy(CN_tx3.data, tx_data, 8);
                        if (twai_transmit(&CN_tx3,portMAX_DELAY) == ESP_OK)
                        {
                            printf("DATA_TRANSFERRED_TO_CN\r\n");
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

_Noreturn void app_main(void)
{
    ESP_ERROR_CHECK(twai_driver_install(&g_config,&t_config,&f_config));
    ESP_ERROR_CHECK(twai_start());

    xTaskCreate(Ch12_task_handler, "ch12_task", 4096,NULL, 2, &ch12_task_handle);
    xTaskCreate(can_task, "task", 4096, NULL, 2, &task_handle);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
