#pragma once

#include "ST-LIB.hpp"

#define SPI_RS_PIN PB2

#define EN_LDU_ID 9998
#define SET_LDU_ID 9999
#define STATE_ID 9996
#define SEND_STATE_RECV_LPU_ID 8888
#define SET_DESIRED_CURRENT_ID 8889
#define START_CONTROL_SLAVE_ID 8887
#define STOP_CONTROL_SLAVE_ID 8886
#define START_PWM_SLAVE_ID 8885
#define STOP_PWM_SLAVE_ID 8884

extern uint8_t set_ldu_spi_id;
extern uint8_t en_ldu_spi_id;
extern uint16_t state_id;

#define PACKET_LDU_TYPE uint8_t, float, uint32_t
#define SHUNT_ARR_TYPE float, float, float, float, float, float, float, float, float, float
#define VBAT_ARR_TYPE float, float, float, float, float, float, float, float, float, float
#define LDU_REF_ARR_TYPE float, float, float, float, float, float, float, float, float, float
#define LDU_EXIT_ARR_TYPE float, float, float, float, float, float, float, float, float, float

extern uint8_t curr_state;

class SPI_DATA
{
    public:
    //shared values
    static uint8_t id_ldu;
    static float duty;
    static float desired_current;
    static uint32_t frequency;
    static int64_t datetime;
    static uint8_t id_buffer;

    static uint8_t confirm_byte;
    static uint8_t en_buffer_byte;

    static uint8_t spi_id;
    static SPIBasePacket *LDU_packet;
    static SPIBasePacket *id_buffer_packet;
    static SPIBasePacket *state_packet;
    static SPIBasePacket *nonePacket;
    static SPIBasePacket *en_buffer_packet;
    static SPIBasePacket *data_LPU_slave_packet;
    static SPIBasePacket *current_ldu_packet;

    static SPIStackOrder* LDU_order;
    static SPIStackOrder* en_LDU_buffer_order;
    static SPIStackOrder* send_state_receive_data_lpu_order; 
    static SPIStackOrder* initial_order;
    static SPIStackOrder* set_current_order;
    static SPIStackOrder* start_control_order;
    static SPIStackOrder* stop_control_order;
    static SPIStackOrder* start_pwm_order;
    static SPIStackOrder* stop_pwm_order;

    static unordered_map<uint8_t, SPIBasePacket*> packets;
    static float shunt_arr[10];
    static float vbat_arr[10];
    static float ldu_ref[10];
    static float ldu_exit[10];



    static void inscribe_spi();

    static void send_initial_check();

    static void start();
    static void send_ldu_pwm();

};