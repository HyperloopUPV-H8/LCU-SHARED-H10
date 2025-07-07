#include "Communication/Communication.hpp"



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
#define STICK_DOWN_ID 8878
#define MULTI_CURRENT_CONTROL_ID 8877
#define MATRIX_INPUT_ID 8876


#define PACKET_LDU_TYPE uint8_t, float, uint32_t
#define PACKET_STATES_TYPE uint8_t, uint8_t, uint8_t, uint8_t
#define SHUNT_ARR_TYPE float, float, float, float, float, float, float, float, float, float
#define AIRGAP_ARR_TYPE float, float, float, float, float, float, float, float
#define VBAT_ARR_TYPE float, float, float, float, float, float, float, float, float, float
#define LDU_REF_ARR_TYPE float, float, float, float, float, float, float, float, float, float
#define LDU_EXIT_ARR_TYPE float, float, float, float, float, float, float, float, float, float
#define DIS_REF_ARR_TYPE float, float, float, float, float, float, float, float
#define DIS_EXIT_ARR_TYPE float, float, float, float, float, float, float, float
#define DIST_AND_ROT_3DOF_TYPE float, float, float, float, float
#define MULTI_CURRENT_TYPE float, float, float, float
#define MATRIX_INPUT_TYPE float, float, float, float, float, float, float, float, float, float, float, float, float, float, float


void SPI_DATA::inscribe_spi()
{
    spi_id = SPI::inscribe(SPI::spi3);
	//SPI::assign_RS(spi_id, SPI_RS_PIN);
}

void SPI_DATA::start()
{
    LDU_packet = new SPIPacket<9, PACKET_LDU_TYPE>(&id_ldu, &duty, &frequency);
    
    id_ldu_packet = new SPIPacket<1, uint8_t>(&id_ldu);

    data_LPU_slave_packet = new SPIPacket<84, PACKET_STATES_TYPE, SHUNT_ARR_TYPE, VBAT_ARR_TYPE>(
        curr_state, curr_state_horizontal, curr_state_vertical, fault_id,
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

    matrix_input_packet = new SPIPacket<60, MATRIX_INPUT_TYPE>(
        &matrix_input[0], &matrix_input[1], &matrix_input[2], &matrix_input[3], &matrix_input[4], &matrix_input[5], &matrix_input[6], &matrix_input[7], &matrix_input[8], &matrix_input[9], &matrix_input[10], &matrix_input[11], &matrix_input[12], &matrix_input[13], &matrix_input[14]
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

    multi_current_packet = new SPIPacket <4*4, MULTI_CURRENT_TYPE>(
        &multi_current_arr[0], &multi_current_arr[1], &multi_current_arr[2], &multi_current_arr[3]
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
    multi_current_order = new SPIStackOrder(MULTI_CURRENT_CONTROL_ID, *multi_current_packet, *nonePacket);
    matrix_input_order = new SPIStackOrder(MATRIX_INPUT_ID, *nonePacket, *matrix_input_packet);

}

