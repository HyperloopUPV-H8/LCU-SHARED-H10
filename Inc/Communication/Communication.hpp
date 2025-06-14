#pragma once

#include "ST-LIB.hpp"

#define SPI_RS_PIN PB2

#define SET_LDU_ID 9999
#define EN_LDU_ID 9998
#define RST_LDU_ID 9997
#define STATE_ID 9996
#define DIS_LDU_ID 9996
#define CURR_LDU_ID 9995
#define START_CONTROL_ID 9994
#define STOP_CONTROL_ID 9993
#define START_PWM_ID 9992
#define STOP_PWM_ID 9991

#define SEND_STATE_RECV_LPU_ID 8888
#define SET_DESIRED_CURRENT_ID 8889
#define START_CONTROL_1DOF_ID 8887
#define STOP_CONTROL_SLAVE_ID 8886
#define START_PWM_SLAVE_ID 8885
#define STOP_PWM_SLAVE_ID 8884
#define RECEIVE_AIRGAP_ID 8883
#define ENTER_BOOSTER_ID 8882
#define RECEIVE_REF_ID 8881
#define FAULT_ID 7777
#define VBAT_ID 8880
#define START_CONTROL_3DOF_ID 8879


#define PACKET_LDU_TYPE uint8_t, float, uint32_t
#define PACKET_STATES_TYPE uint8_t, uint8_t, uint8_t
#define SHUNT_ARR_TYPE float, float, float, float, float, float, float, float, float, float
#define AIRGAP_ARR_TYPE float, float, float, float, float, float, float, float
#define VBAT_ARR_TYPE float, float, float, float, float, float, float, float, float, float
#define LDU_REF_ARR_TYPE float, float, float, float, float, float, float, float, float, float
#define LDU_EXIT_ARR_TYPE float, float, float, float, float, float, float, float, float, float
#define DIS_REF_ARR_TYPE float, float, float, float, float, float, float, float
#define DIS_EXIT_ARR_TYPE float, float, float, float, float, float, float, float
#define DIST_AND_ROT_3DOF_TYPE float, float, float, float, float

extern uint8_t *curr_state;
extern uint8_t *curr_state_horizontal;
extern uint8_t *curr_state_vertical;

class SPI_DATA
{
    public:
    //shared values
    static uint8_t id_ldu;
    static uint8_t booster_status;
    static float duty;
    static float desired_current;
    static float desired_distance;
    static uint32_t frequency;
    static uint8_t id_buffer;

    //3dof values
    static float values_rot_and_dis[5];

    static uint8_t confirm_byte;
    static uint8_t en_buffer_byte;

    static uint8_t spi_id;
    static SPIBasePacket *LDU_packet;
    static SPIBasePacket *id_ldu_packet;
    static SPIBasePacket *nonePacket;
    static SPIBasePacket *en_buffer_packet;
    static SPIBasePacket *data_LPU_slave_packet;
    static SPIBasePacket *data_arigap_packet;
    static SPIBasePacket *current_ldu_packet;
    static SPIBasePacket *booster_control_packet;
    static SPIBasePacket *data_refs_packet;
    static SPIBasePacket *vbat_packet;
    static SPIBasePacket *distance_packet;

    static SPIStackOrder* LDU_order;
    static SPIStackOrder* en_LDU_buffer_order;
    static SPIStackOrder* receive_data_airgap_order;
    static SPIStackOrder* send_state_receive_data_lpu_order; 
    static SPIStackOrder* initial_order;
    static SPIStackOrder* set_current_order;
    static SPIStackOrder* start_control_1dof_order;
    static SPIStackOrder* stop_control_order;
    static SPIStackOrder* start_pwm_order;
    static SPIStackOrder* stop_pwm_order;
    static SPIStackOrder* booster_control_order;
    static SPIStackOrder* receive_refs_order;
    static SPIStackOrder* fault_order;
    static SPIStackOrder* send_fixed_vbat_order;
    static SPIStackOrder* start_3dof_order;

    static float shunt_arr[10];
    static float vbat_arr[10];
    static float ldu_ref[10];
    static float ldu_exit[10];

    static float airgap_arr[8];
    static float dis_exit[8];
    static float dis_ref[8];
    static float vbat_fixed;



    static void inscribe_spi();

    static void start();

};