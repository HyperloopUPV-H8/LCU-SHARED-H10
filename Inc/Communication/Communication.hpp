#pragma once

#include "ST-LIB.hpp"

#define SPI_RS_PIN PB2

extern uint8_t *curr_state;
extern uint8_t *curr_state_horizontal;
extern uint8_t *curr_state_vertical;
extern uint8_t *fault_id;

class SPI_DATA
{
    public:
    //shared values
    inline static uint8_t id_ldu{0};
    inline static uint8_t booster_status{0};
    inline static float duty{0.0};
    inline static float desired_current{0.0};
    inline static float desired_distance{0.0};
    inline static uint32_t frequency{0};
    inline static uint8_t id_buffer{0};

    //3dof values
    inline static uint8_t use_5dof{0};
    inline static float values_rot_and_dis[5]{};
    inline static float matrix_input[15]{};

    inline static uint8_t confirm_byte{0};
    inline static uint8_t en_buffer_byte{0};

    inline static uint8_t spi_id{0};
    inline static SPIBasePacket *LDU_packet{nullptr};
    inline static SPIBasePacket *id_ldu_packet{nullptr};
    inline static SPIBasePacket *nonePacket{nullptr};
    inline static SPIBasePacket *en_buffer_packet{nullptr};
    inline static SPIBasePacket *data_LPU_slave_packet{nullptr};
    inline static SPIBasePacket *data_arigap_packet{nullptr};
    inline static SPIBasePacket *current_ldu_packet{nullptr};
    inline static SPIBasePacket *booster_control_packet{nullptr};
    inline static SPIBasePacket *data_refs_packet{nullptr};
    inline static SPIBasePacket *vbat_packet{nullptr};
    inline static SPIBasePacket *distance_packet{nullptr};
    inline static SPIBasePacket *levitation_packet{nullptr};
    inline static SPIBasePacket *multi_current_packet{nullptr};
    inline static SPIBasePacket *matrix_input_packet{nullptr};

    inline static SPIStackOrder* LDU_order{nullptr};
    inline static SPIStackOrder* en_LDU_buffer_order{nullptr};
    inline static SPIStackOrder* receive_data_airgap_order{nullptr};
    inline static SPIStackOrder* send_state_receive_data_lpu_order{nullptr}; 
    inline static SPIStackOrder* initial_order{nullptr};
    inline static SPIStackOrder* set_current_order{nullptr};
    inline static SPIStackOrder* start_control_1dof_order{nullptr};
    inline static SPIStackOrder* stop_control_order{nullptr};
    inline static SPIStackOrder* start_pwm_order{nullptr};
    inline static SPIStackOrder* stop_pwm_order{nullptr};
    inline static SPIStackOrder* booster_control_order{nullptr};
    inline static SPIStackOrder* receive_refs_order{nullptr};
    inline static SPIStackOrder* fault_order{nullptr};
    inline static SPIStackOrder* send_fixed_vbat_order{nullptr};
    inline static SPIStackOrder* stick_down_order{nullptr};
    inline static SPIStackOrder* start_3dof_order{nullptr};
    inline static SPIStackOrder* multi_current_order{nullptr};
    inline static SPIStackOrder* matrix_input_order{nullptr};

    inline static float shunt_arr[10]{};
    inline static float vbat_arr[10]{};
    inline static float ldu_ref[10]{};
    inline static float ldu_exit[10]{};

    inline static float airgap_arr[8]{};
    inline static float dis_exit[8]{};
    inline static float dis_ref[8]{};
    inline static float vbat_fixed{255.0};
    inline static float multi_current_arr[4]{};



    static void inscribe_spi();

    static void start();

};