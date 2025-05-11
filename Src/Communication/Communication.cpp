#include "Communication/Communication.hpp"

#define MAX_LEN_DATA 30

uint8_t SPI_DATA::id_ldu = 0;
float SPI_DATA::duty = 0.0;
float SPI_DATA::desired_current = 0.0;
uint32_t SPI_DATA::frequency = 0;
int64_t SPI_DATA::datetime = 0;
uint8_t SPI_DATA::id_buffer = 0;

uint8_t SPI_DATA::spi_id = 0;
uint8_t SPI_DATA::en_buffer_byte = 0;
float SPI_DATA::desired_distance = 0.0;
float SPI_DATA::vbat_fixed = 200.0F;

SPIBasePacket* SPI_DATA::LDU_packet = nullptr;
SPIBasePacket* SPI_DATA::id_buffer_packet = nullptr;
SPIBasePacket* SPI_DATA::id_ldu_packet = nullptr;
SPIBasePacket* SPI_DATA::state_packet = nullptr;
SPIBasePacket* SPI_DATA::data_LPU_slave_packet = nullptr;
SPIBasePacket* SPI_DATA::data_arigap_packet = nullptr;
SPIBasePacket* SPI_DATA::nonePacket = nullptr;
SPIBasePacket* SPI_DATA::en_buffer_packet = nullptr;
SPIBasePacket* SPI_DATA::current_ldu_packet = nullptr;
SPIBasePacket* SPI_DATA::data_refs_packet = nullptr;
SPIBasePacket* SPI_DATA::vbat_packet = nullptr;

SPIStackOrder* SPI_DATA::LDU_order = nullptr;
SPIStackOrder* SPI_DATA::en_LDU_buffer_order = nullptr;
SPIStackOrder* SPI_DATA::receive_data_airgap_order = nullptr;
SPIStackOrder* SPI_DATA::initial_order = nullptr;
SPIStackOrder* SPI_DATA::send_state_receive_data_lpu_order = nullptr;
SPIStackOrder* SPI_DATA::set_current_order = nullptr;
SPIStackOrder* SPI_DATA::start_control_order = nullptr;
SPIStackOrder* SPI_DATA::stop_control_order = nullptr;
SPIStackOrder* SPI_DATA::start_pwm_order = nullptr;
SPIStackOrder* SPI_DATA::stop_pwm_order = nullptr;
SPIStackOrder* SPI_DATA::receive_refs_order = nullptr;
SPIStackOrder* SPI_DATA::fault_order = nullptr;
SPIStackOrder* SPI_DATA::send_fixed_vbat_order = nullptr;

unordered_map<uint8_t, SPIBasePacket*> SPI_DATA::packets = {};

uint16_t state_id = 9996;

float SPI_DATA::shunt_arr[10] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};
float SPI_DATA::vbat_arr[10] = {};
float SPI_DATA::ldu_ref[10] = {};
float SPI_DATA::ldu_exit[10] = {};
float SPI_DATA::dis_ref[8] = {};
float SPI_DATA::dis_exit[8] = {};

float SPI_DATA::airgap_arr[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};




void SPI_DATA::inscribe_spi()
{
    spi_id = SPI::inscribe(SPI::spi3);
	//SPI::assign_RS(spi_id, SPI_RS_PIN);
}






void SPI_DATA::start()
{
    SPI::start();
    LDU_packet = new SPIPacket<9, PACKET_LDU_TYPE>(&id_ldu, &duty, &frequency);
    
    id_ldu_packet = new SPIPacket<5, uint8_t, float>(&id_ldu, &desired_distance);
    id_buffer_packet = new SPIPacket<1, uint8_t>(&id_buffer);
    state_packet = new SPIPacket<1, uint8_t>(curr_state);
    data_LPU_slave_packet = new SPIPacket<80, SHUNT_ARR_TYPE, VBAT_ARR_TYPE>(
        &shunt_arr[0], &shunt_arr[1], &shunt_arr[2], &shunt_arr[3], &shunt_arr[4], &shunt_arr[5], &shunt_arr[6], &shunt_arr[7], &shunt_arr[8], &shunt_arr[9],
        &vbat_arr[0], &vbat_arr[1], &vbat_arr[2], &vbat_arr[3], &vbat_arr[4], &vbat_arr[5], &vbat_arr[6],&vbat_arr[7], &vbat_arr[8], &vbat_arr[9]   
    );
    data_refs_packet = new SPIPacket<80, LDU_REF_ARR_TYPE, LDU_EXIT_ARR_TYPE>(
        &ldu_ref[0], &ldu_ref[1], &ldu_ref[2], &ldu_ref[3], &ldu_ref[4], &ldu_ref[5], &ldu_ref[6], &ldu_ref[7], &ldu_ref[8], &ldu_ref[9],
        &ldu_exit[0], &ldu_exit[1], &ldu_exit[2], &ldu_exit[3], &ldu_exit[4], &ldu_exit[5], &ldu_exit[6], &ldu_exit[7], &ldu_exit[8], &ldu_exit[9]
    );
    data_arigap_packet = new SPIPacket<32, AIRGAP_ARR_TYPE>(&airgap_arr[0], &airgap_arr[1], &airgap_arr[2], &airgap_arr[3], &airgap_arr[4], &airgap_arr[5], &airgap_arr[6], &airgap_arr[7]);
    en_buffer_packet = new SPIPacket<1, uint8_t>(&en_buffer_byte);
    nonePacket = new SPIPacket<0>;

    current_ldu_packet = new SPIPacket<5, uint8_t, float>(&id_ldu, &desired_current);

    vbat_packet = new SPIPacket<4, float>(
        &vbat_fixed
    );

    initial_order = new SPIStackOrder(STATE_ID, *state_packet, *nonePacket);


    LDU_order = new SPIStackOrder(SET_LDU_ID, *LDU_packet, *nonePacket);
    send_state_receive_data_lpu_order = new SPIStackOrder(SEND_STATE_RECV_LPU_ID, *state_packet, *data_LPU_slave_packet);
    receive_data_airgap_order = new SPIStackOrder(RECEIVE_AIRGAP_ID, *nonePacket, *data_arigap_packet);
    
    receive_refs_order = new SPIStackOrder(RECEIVE_REF_ID, *nonePacket, *data_refs_packet);

    en_LDU_buffer_order = new SPIStackOrder(EN_LDU_ID, *en_buffer_packet, *nonePacket); 

    set_current_order = new SPIStackOrder(SET_DESIRED_CURRENT_ID, *current_ldu_packet, *nonePacket);

    start_control_order = new SPIStackOrder(START_CONTROL_SLAVE_ID, *id_ldu_packet, *nonePacket);
    stop_control_order = new SPIStackOrder(STOP_CONTROL_SLAVE_ID, *nonePacket, *nonePacket);

    start_pwm_order = new SPIStackOrder(START_PWM_SLAVE_ID, *current_ldu_packet, *nonePacket);
    stop_pwm_order = new SPIStackOrder(STOP_PWM_SLAVE_ID, *current_ldu_packet, *nonePacket);

    fault_order = new SPIStackOrder(FAULT_ID, *nonePacket, *nonePacket);

    
    send_fixed_vbat_order = new SPIStackOrder(VBAT_ID, *vbat_packet, *nonePacket);

}

