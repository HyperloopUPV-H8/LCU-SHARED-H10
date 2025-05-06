#include "Communication/Communication.hpp"


void SPI_DATA::inscribe_spi()
{
    spi_id = SPI::inscribe(SPI::spi3);
	SPI::assign_RS(spi_id, SPI_RS_PIN);
}

void SPI_DATA::start()
{
    LDU_packet = new SPIPacket<9, PACKET_LDU_TYPE>(&id_ldu, &duty, &frequency);
    
    id_ldu_packet = new SPIPacket<5, uint8_t, float>(&id_ldu, &desired_distance);
    data_LPU_slave_packet = new SPIPacket<43, uint8_t, uint8_t, uint8_t, SHUNT_ARR_TYPE, VBAT_ARR_TYPE>(
        curr_state, curr_state_horizontal, curr_state_vertical,
        &shunt_arr[0], &shunt_arr[1], &shunt_arr[2], &shunt_arr[3], &shunt_arr[4], &shunt_arr[5], &shunt_arr[6], &shunt_arr[7], &shunt_arr[8], &shunt_arr[9],
        &vbat_arr[0], &vbat_arr[1], &vbat_arr[2], &vbat_arr[3], &vbat_arr[4], &vbat_arr[5], &vbat_arr[6],&vbat_arr[7], &vbat_arr[8], &vbat_arr[9]
    );
    data_arigap_packet = new SPIPacket<32, AIRGAP_ARR_TYPE>(&airgap_arr[0], &airgap_arr[1], &airgap_arr[2], &airgap_arr[3], &airgap_arr[4], &airgap_arr[5], &airgap_arr[6], &airgap_arr[7]);
    en_buffer_packet = new SPIPacket<1, uint8_t>(&en_buffer_byte);
    nonePacket = new SPIPacket<0>;

    current_ldu_packet = new SPIPacket<5, uint8_t, float>(&id_ldu, &desired_current);
    booster_control_packet = new SPIPacket<1, uint8_t>(&booster_status);


    initial_order = new SPIStackOrder(STATE_ID, *nonePacket, *nonePacket);


    LDU_order = new SPIStackOrder(SET_LDU_ID, *LDU_packet, *nonePacket);
    send_state_receive_data_lpu_order = new SPIStackOrder(SEND_STATE_RECV_LPU_ID, *nonePacket, *data_LPU_slave_packet);
    receive_data_airgap_order = new SPIStackOrder(RECEIVE_AIRGAP_ID, *nonePacket, *data_arigap_packet);

    en_LDU_buffer_order = new SPIStackOrder(EN_LDU_ID, *en_buffer_packet, *nonePacket); 

    set_current_order = new SPIStackOrder(SET_DESIRED_CURRENT_ID, *current_ldu_packet, *nonePacket);

    start_control_order = new SPIStackOrder(START_CONTROL_SLAVE_ID, *id_ldu_packet, *nonePacket);
    stop_control_order = new SPIStackOrder(STOP_CONTROL_SLAVE_ID, *nonePacket, *nonePacket);

    start_pwm_order = new SPIStackOrder(START_PWM_SLAVE_ID, *current_ldu_packet, *nonePacket);
    stop_pwm_order = new SPIStackOrder(STOP_PWM_SLAVE_ID, *current_ldu_packet, *nonePacket);

    booster_control_order = new SPIStackOrder(ENTER_BOOSTER_ID, *booster_control_packet, *nonePacket);


}

