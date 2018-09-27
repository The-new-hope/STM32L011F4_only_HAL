#include "header.h"
uint8_t self_addr[] = OWN_ADDRESS;
uint8_t remote_addr[] = REMOTE_ADDRESS;
extern uint8_t Tcounter;
extern uint8_t Tcounter1;
//extern uint8_t buffer_RX[];
extern uint8_t status_TX;
extern uint8_t status_RX;
//extern uint8_t Data[255];				////������ ��� �������///////////////////////////////////////
//extern UART_HandleTypeDef huart2;		////������ ��� �������///////////////////////////////////////	
extern SPI_HandleTypeDef hspi1;
//extern uint16_t size_UART;			////������ ��� �������///////////////////////////////////////
//extern char str1[20];
//***********************************************************************************************************
//------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
  micros *= (SystemCoreClock / 1000000) / 8;
  /* Wait till done */
  while (micros--) ;
}
//--------------------------------------------------
//***********************************************************************************************************
unsigned char SPIx_Transfer(uint8_t data){
uint8_t rx_data;
HAL_SPI_TransmitReceive(&hspi1, &data, &rx_data, sizeof(data), 0x1000);	
return rx_data;	
}	

//***********************************************************************************************************
// ��������� ������� cmd, � ������ count ���� ������, ������� �� � ����� buf, ���������� ������� �������
unsigned char NRF_read_buf(unsigned char cmd, unsigned char * buf, unsigned char count) {
	CSN0;
	uint8_t status;
	status = SPIx_Transfer(cmd);
	while (count--) {
		*(buf++) = SPIx_Transfer(0xFF);
	}
	CSN1;
	return status;
 }
//***********************************************************************************************************
// ������ �������� � ���������� ������� �������
uint8_t NRF_readreg_STATUC(){
	CSN0;
	uint8_t answ = SPIx_Transfer(NOP);
	CSN1;
	return answ;
 }
//***********************************************************************************************************
// ���������� �������� ������������� �������� reg (�� 0 �� 31), ���������� ������� �������
uint8_t NRF_writereg(unsigned char reg, unsigned char val){
	CSN0;
	uint8_t status = SPIx_Transfer((reg & 31) | W_REGISTER);
	SPIx_Transfer(val);
	CSN1;
	return status;
 }
//***********************************************************************************************************
// ������ �������� ������������� �������� reg (�� 0 �� 31) � ���������� ���
uint8_t NRF_readreg(uint8_t reg){
	CSN0;
	SPIx_Transfer((reg & 31) | R_REGISTER);
	uint8_t answ = SPIx_Transfer(NOP);
	CSN1;
	return answ;
 }
//***********************************************************************************************************
// ��������� ������� cmd, � ������� count ���� ���������� �� ������ buf, ���������� ������� �������
uint8_t NRF_write_buf(uint8_t cmd, uint8_t * buf, uint8_t count) {
	CSN0;		
	uint8_t status = SPIx_Transfer(cmd);
//	SPIx_Transfer(buf);
	while (count--) {
	SPIx_Transfer(*(buf++));
	}
	CSN1;
	return status;
 }
//***********************************************************************************************************
// ���������� count ���� �� ������ buf � ������������� ������� reg (�� 0 �� 31), ���������� ������� �������
uint8_t NRF_writereg_buf(uint8_t reg, uint8_t * buf, uint8_t count) {
	return NRF_write_buf((reg & 31) | W_REGISTER, buf, count);
 }
//***********************************************************************************************************
// ���������� ������ ������ � ������ FIFO ������� ��������
uint8_t NRF_read_rx_payload_width() {
	CSN0;
	SPIx_Transfer(R_RX_PL_WID);
	uint8_t answ = SPIx_Transfer(0xFF);
	CSN1;
	return answ;
 }
//***********************************************************************************************************
// ��������� �������. ���������� ������� �������
uint8_t NRF_cmd(uint8_t cmd) {
	CSN0;
	uint8_t status = SPIx_Transfer(cmd);
	CSN1;
	return status;
 }
//***********************************************************************************************************
// ���������� 1, ���� �� ����� IRQ �������� (������) �������.
uint8_t IRQ_is_interrupt(){	
	if (HAL_GPIO_ReadPin(SPI_GPIO, SPI_PIN_IRQ) == 0) {
		return 1;
		} else {
		return 0;
		}
 }
//***********************************************************************************************************
// ������� ����������� ���������� �����������. ����������� � �������� �����
uint8_t check_NRF() {
	if (status_RX == 1){
		return status_RX == 1;
	}
	if (!(IRQ_is_interrupt()==1)){ // ���� ���������� ���, �� �� �������������
		return status_RX = 0;							// ���������� 0 ��� ���� ��� ������ ���
	}
//**********************************************
	uint8_t status = NRF_cmd(NOP);
//***********************************************
	if (status & (1 << RX_DR)){								// ���� ������ �����
		uint8_t l = NRF_read_rx_payload_width();			// ����� ����� ������
		uint8_t buf[10]; 									// ����� ��� ��������� ������
		NRF_read_buf(R_RX_PAYLOAD, &buf[0], l); 			// ������������ �����
		on_packet(&buf[0], l); 								// �������� ���������� ����������� ������, ������� ���������� ���������� ������ � ������ buffer_RX[]
		NRF_cmd(FLUSH_RX);									// �������� ������� ��������
		status = NRF_cmd(NOP);								// ������ ������
		NRF_writereg(STATUS, status);						// ���������� ����� ��������� STATUS �������
	}
	return status_RX = 1;									// ���������� 1 ��� ���� ��� ������ ����� ����������
 }
//***********************************************************************************************************
// ����������, ����� ��������� ����� ������� ��������, � ������������� ��� � �� ���� ��������.
void on_send_error() {
 // ����� ����� ������� ���������� ��������� ��������
 }
//***********************************************************************************************************
// ���������� ��� ��������� ������ ������ �� ������ 1 �� �������� �������. buf - ����� � �������, size - ����� ������ (�� 1 �� 32)
void on_packet(uint8_t * buf, uint8_t size) {
//	uint8_t i;									//
//	for (i=0; i<=size; i++){					//
//		buffer_RX[i] = buf[i];					// �����������������, ���� ����� ��������� ������
//	}											//
 }
//***********************************************************************************************************
// �������� ����� � ������� ��������.
// buf - ����� � �������, size - ����� ������ (�� 1 �� 32)
uint8_t send_data_NRF(uint8_t * buf, uint8_t size) {
	CE0; 													// ���� � ������ �����, �� ��������� ���
	NRF_write_buf(W_TX_PAYLOAD, buf, size); 				// ������ ������ �� ��������	
	Conf_NRF_Tx(); 											// configuring as transmitter and send data
//							size_UART = sprintf((char *)Data, "I wait interrupt...\n\r");/////////////////////////////////////������ ��� �������///////////////////////////////////////
//							HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);	///////////////////////////////////////////////������ ��� �������////////////////////////////////////
	
 	while(IRQ_is_interrupt() != 1){ 						// ��� 2 ��� ���������� � ����������
		if (Tcounter1 >=20){
//							size_UART = sprintf((char *)Data, "I didnt wait interrupt(((\n\r");////////////////////////////////////������ ��� �������/////////////////////////////////////
//							HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);			/////////////////////////////////////////////������ ��� �������/////////////////////////////////////
			break;}
	}
		
	DelayMicro(40);
	uint8_t status = NRF_readreg(STATUS);					//read register STATUS
	if (status & (1 << TX_DS)){
		status_TX = 1;  									// ���� �� ������, ����������  1 �������� �������		
//							size_UART = sprintf((char *)Data, "Status %X\n\r", status);/////////////////////////////////////������ ��� �������///////////////////////////////////////////////
//							HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);		/////////////////////////////////////������ ��� �������////////////////////////////////////////////////
		
						}	else {
//							size_UART = sprintf((char *)Data, "Transmit Bad!!!\n\r");/////////////////////////////////////������ ��� �������///////////////////////////////////////////////
//							HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);		/////////////////////////////////////������ ��� �������////////////////////////////////////////////////
//							size_UART = sprintf((char *)Data, "Status %X\n\r", status);/////////////////////////////////////������ ��� �������///////////////////////////////////////////////
//							HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);		/////////////////////////////////////������ ��� �������////////////////////////////////////////////////
						}			
	status = NRF_readreg(STATUS);							//read register STATUS
	NRF_writereg(STATUS, status);							//write register STATUS and clear him
	Conf_NRF_Rx();											// configuring as receiver
	return status_TX;  										// return status_TX
 }
//***********************************************************************************************************

uint8_t Conf_NRF_Rx(){	  //����� ����� RX
	  CE0;
	  NRF_cmd(NOP);
	  NRF_writereg(RX_PW_P0,RF_DATA_SIZE); //������ ���� ������, ����.
//	  NRF_writereg(ENAA_P4,0); //��������� ����������������� �� ������ 4
	  NRF_writereg(RF_CH, CHAN); 
	  NRF_writereg(RF_SETUP, RF_SPEED); // ����� �������� 250 k���/� � �������� 0dBm
	  NRF_writereg(SETUP_RETR, SETUP_RETR_DELAY_1000MKS); // Delay for speed 250	k���/�
	  NRF_writereg_buf(RX_ADDR_P0, &remote_addr[0], 5);
	  NRF_writereg_buf(TX_ADDR, &remote_addr[0], 5);
	  NRF_writereg_buf(RX_ADDR_P4, &self_addr[0], 5);
	  NRF_writereg(CONFIG,(1<<EN_CRC)|(1<<CRCO)|(1<<PWR_UP)|(0<<PRIM_RX));
	  DelayMicro(3);
	  NRF_writereg(CONFIG,(1<<EN_CRC)|(1<<CRCO)|(1<<PWR_UP)|(1<<PRIM_RX));
	  CE1;
	  DelayMicro(140);
	  return (NRF_readreg(CONFIG) == ((1 << EN_CRC) |(1<<CRCO)| (1 << PWR_UP) | (1 << PRIM_RX))) ? 1 : 0;
 }
//***********************************************************************************************************
uint8_t Conf_NRF_Tx(){
	  CE0;
	  NRF_cmd(NOP);
		NRF_writereg(RX_PW_P0,RF_DATA_SIZE);//������ ���� ������, ����.
//	  NRF_writereg(ENAA_P4,0);//��������� ����������������� �� ������ 0
		NRF_writereg(RF_CH, CHAN); // ����� ���������� ������
	  NRF_writereg(RF_SETUP, RF_SPEED); // ����� �������� 250 k���/� � �������� 0dBm
	  NRF_writereg(SETUP_RETR, SETUP_RETR_DELAY_1000MKS); // Delay for speed 250 k���/�	
	  NRF_writereg_buf(RX_ADDR_P0, &remote_addr[0], 5); // ������������� �������� �� ����� 0
	  NRF_writereg_buf(TX_ADDR, &remote_addr[0], 5);
//	  NRF_writereg_buf(RX_ADDR_P0, &self_addr[0], 5);
	  NRF_writereg(CONFIG,(1<<PWR_UP)|(1<<EN_CRC)|(1<<CRCO)|(0<<PRIM_RX)|(0 << MASK_MAX_RT));
	  CE1;
	  DelayMicro(15);
	  CE0;
	  DelayMicro(140);
	  return (NRF_readreg(CONFIG) == ((1 << EN_CRC)|(1<<CRCO) | (1 << PWR_UP) | (0 << PRIM_RX) | (0 << MASK_MAX_RT))) ? 1 : 0;
 }


