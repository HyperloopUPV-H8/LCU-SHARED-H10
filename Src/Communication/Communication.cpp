#include "Communication/Communication.hpp"

float SPI_DATA::shunt_arr[10] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};
float SPI_DATA::vbat_arr[10] = {};
float SPI_DATA::ldu_ref[10] = {};
float SPI_DATA::ldu_exit[10] = {};
float SPI_DATA::dis_ref[8] = {};
float SPI_DATA::dis_exit[8] = {};

float SPI_DATA::airgap_arr[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float SPI_DATA::values_rot_and_dis[5] = {};

uint8_t SPI_DATA::id_ldu = 0;
uint8_t SPI_DATA::booster_status = 0;
float SPI_DATA::duty = 0.0;
float SPI_DATA::desired_current = 0.0;
uint32_t SPI_DATA::frequency = 0;
uint8_t SPI_DATA::id_buffer = 0;

uint8_t SPI_DATA::spi_id = 0;
uint8_t SPI_DATA::use_5dof = 0;
uint8_t SPI_DATA::en_buffer_byte = 0;
float SPI_DATA::desired_distance = 0.0;
float SPI_DATA::vbat_fixed = 200.0F;

SPIBasePacket* SPI_DATA::LDU_packet = nullptr;
SPIBasePacket* SPI_DATA::id_ldu_packet = nullptr;
SPIBasePacket* SPI_DATA::data_LPU_slave_packet = nullptr;
SPIBasePacket* SPI_DATA::data_arigap_packet = nullptr;
SPIBasePacket* SPI_DATA::nonePacket = nullptr;
SPIBasePacket* SPI_DATA::en_buffer_packet = nullptr;
SPIBasePacket* SPI_DATA::current_ldu_packet = nullptr;
SPIBasePacket* SPI_DATA::booster_control_packet = nullptr;
SPIBasePacket* SPI_DATA::data_refs_packet = nullptr;
SPIBasePacket* SPI_DATA::vbat_packet = nullptr;
SPIBasePacket* SPI_DATA::distance_packet = nullptr;
SPIBasePacket* SPI_DATA::levitation_packet = nullptr;

SPIStackOrder* SPI_DATA::LDU_order = nullptr;
SPIStackOrder* SPI_DATA::en_LDU_buffer_order = nullptr;
SPIStackOrder* SPI_DATA::receive_data_airgap_order = nullptr;
SPIStackOrder* SPI_DATA::initial_order = nullptr;
SPIStackOrder* SPI_DATA::send_state_receive_data_lpu_order = nullptr;
SPIStackOrder* SPI_DATA::set_current_order = nullptr;
SPIStackOrder* SPI_DATA::start_control_1dof_order = nullptr;
SPIStackOrder* SPI_DATA::stop_control_order = nullptr;
SPIStackOrder* SPI_DATA::start_pwm_order = nullptr;
SPIStackOrder* SPI_DATA::stop_pwm_order = nullptr;
SPIStackOrder* SPI_DATA::receive_refs_order = nullptr;
SPIStackOrder* SPI_DATA::fault_order = nullptr;
SPIStackOrder* SPI_DATA::send_fixed_vbat_order = nullptr;
SPIStackOrder* SPI_DATA::booster_control_order = nullptr;
SPIStackOrder* SPI_DATA::stick_down_order = nullptr;
SPIStackOrder* SPI_DATA::start_3dof_order = nullptr;

void SPI_DATA::inscribe_spi()
{
    spi_id = SPI::inscribe(SPI::spi3);
	//SPI::assign_RS(spi_id, SPI_RS_PIN);
}

void SPI_DATA::start()
{
    LDU_packet = new SPIPacket<9, PACKET_LDU_TYPE>(&id_ldu, &duty, &frequency);
    
    id_ldu_packet = new SPIPacket<1, uint8_t>(&id_ldu);

    data_LPU_slave_packet = new SPIPacket<83, PACKET_STATES_TYPE, SHUNT_ARR_TYPE, VBAT_ARR_TYPE>(
        curr_state, curr_state_horizontal, curr_state_vertical,
        &shunt_arr[0], &shunt_arr[1], &shunt_arr[2], &shunt_arr[3], &shunt_arr[4], &shunt_arr[5], &shunt_arr[6], &shunt_arr[7], &shunt_arr[8], &shunt_arr[9],
        &vbat_arr[0], &vbat_arr[1], &vbat_arr[2], &vbat_arr[3], &vbat_arr[4], &vbat_arr[5], &vbat_arr[6],&vbat_arr[7], &vbat_arr[8], &vbat_arr[9]   
    );
    data_refs_packet = new SPIPacket<80, LDU_REF_ARR_TYPE, LDU_EXIT_ARR_TYPE>(
        &ldu_ref[0], &ldu_ref[1], &ldu_ref[2], &ldu_ref[3], &ldu_ref[4], &ldu_ref[5], &ldu_ref[6], &ldu_ref[7], &ldu_ref[8], &ldu_ref[9],
        &ldu_exit[0], &ldu_exit[1], &ldu_exit[2], &ldu_exit[3], &ldu_exit[4], &ldu_exit[5], &ldu_exit[6], &ldu_exit[7], &ldu_exit[8], &ldu_exit[9]
    );
    data_arigap_packet = new SPIPacket<52, AIRGAP_ARR_TYPE, DIST_AND_ROT_3DOF_TYPE>(&airgap_arr[0], &airgap_arr[1], &airgap_arr[2], &airgap_arr[3], &airgap_arr[4], &airgap_arr[5], &airgap_arr[6], &airgap_arr[7],
        &values_rot_and_dis[0], &values_rot_and_dis[1], &values_rot_and_dis[2], &values_rot_and_dis[3], &values_rot_and_dis[4]
    );
    
    en_buffer_packet = new SPIPacket<1, uint8_t>(&en_buffer_byte);
    nonePacket = new SPIPacket<0>;

    current_ldu_packet = new SPIPacket<5, uint8_t, float>(&id_ldu, &desired_current);
    distance_packet = new SPIPacket<5, uint8_t, float>(&id_ldu, &desired_distance);
    levitation_packet = new SPIPacket<5, uint8_t, float>(&use_5dof, &desired_distance);
    booster_control_packet = new SPIPacket<1, uint8_t>(&booster_status);

    vbat_packet = new SPIPacket<4, float>(
        &vbat_fixed
    );

    initial_order = new SPIStackOrder(STATE_ID, *nonePacket, *nonePacket);


    LDU_order = new SPIStackOrder(SET_LDU_ID, *LDU_packet, *nonePacket);
    send_state_receive_data_lpu_order = new SPIStackOrder(SEND_STATE_RECV_LPU_ID, *nonePacket, *data_LPU_slave_packet);
    receive_data_airgap_order = new SPIStackOrder(RECEIVE_AIRGAP_ID, *nonePacket, *data_arigap_packet);
    
    receive_refs_order = new SPIStackOrder(RECEIVE_REF_ID, *nonePacket, *data_refs_packet);

    en_LDU_buffer_order = new SPIStackOrder(EN_LDU_ID, *en_buffer_packet, *nonePacket); 

    set_current_order = new SPIStackOrder(SET_DESIRED_CURRENT_ID, *current_ldu_packet, *nonePacket);

    start_control_1dof_order = new SPIStackOrder(START_CONTROL_1DOF_ID, *distance_packet, *nonePacket);
    stop_control_order = new SPIStackOrder(STOP_CONTROL_SLAVE_ID, *nonePacket, *nonePacket);

    start_pwm_order = new SPIStackOrder(START_PWM_SLAVE_ID, *id_ldu_packet, *nonePacket);
    stop_pwm_order = new SPIStackOrder(STOP_PWM_SLAVE_ID, *id_ldu_packet, *nonePacket);

    fault_order = new SPIStackOrder(FAULT_ID, *nonePacket, *nonePacket);
    booster_control_order = new SPIStackOrder(ENTER_BOOSTER_ID, *booster_control_packet, *nonePacket);
    start_3dof_order = new SPIStackOrder(START_CONTROL_3DOF_ID, *levitation_packet, *nonePacket);
    
    send_fixed_vbat_order = new SPIStackOrder(VBAT_ID, *vbat_packet, *nonePacket);
    stick_down_order = new SPIStackOrder(STICK_DOWN_ID, *nonePacket, *nonePacket);

}

