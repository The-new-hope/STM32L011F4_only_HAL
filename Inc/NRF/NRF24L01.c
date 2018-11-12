#include "header.h"
uint8_t self_addr[] = OWN_ADDRESS;
uint8_t remote_addr[] = REMOTE_ADDRESS;
extern uint8_t Tcounter;
extern uint8_t Tcounter1;
//extern uint8_t buffer_RX[];
extern uint8_t status_TX;
extern uint8_t status_RX;
//extern uint8_t Data[255];				////строка для отладки///////////////////////////////////////
//extern UART_HandleTypeDef huart2;		////строка для отладки///////////////////////////////////////	
extern SPI_HandleTypeDef hspi1;
//extern uint16_t size_UART;			////строка для отладки///////////////////////////////////////
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
// Выполняет команду cmd, и читает count байт ответа, помещая их в буфер buf, возвращает регистр статуса
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
// Читает значение и возвращает регистр статуса
uint8_t NRF_readreg_STATUC(){
	CSN0;
	uint8_t answ = SPIx_Transfer(NOP);
	CSN1;
	return answ;
 }
//***********************************************************************************************************
// Записывает значение однобайтового регистра reg (от 0 до 31), возвращает регистр статуса
uint8_t NRF_writereg(unsigned char reg, unsigned char val){
	CSN0;
	uint8_t status = SPIx_Transfer((reg & 31) | W_REGISTER);
	SPIx_Transfer(val);
	CSN1;
	return status;
 }
//***********************************************************************************************************
// Читает значение однобайтового регистра reg (от 0 до 31) и возвращает его
uint8_t NRF_readreg(uint8_t reg){
	CSN0;
	SPIx_Transfer((reg & 31) | R_REGISTER);
	uint8_t answ = SPIx_Transfer(NOP);
	CSN1;
	return answ;
 }
//***********************************************************************************************************
// Выполняет команду cmd, и передаёт count байт параметров из буфера buf, возвращает регистр статуса
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
// Записывает count байт из буфера buf в многобайтовый регистр reg (от 0 до 31), возвращает регистр статуса
uint8_t NRF_writereg_buf(uint8_t reg, uint8_t * buf, uint8_t count) {
	return NRF_write_buf((reg & 31) | W_REGISTER, buf, count);
 }
//***********************************************************************************************************
// Возвращает размер данных в начале FIFO очереди приёмника
uint8_t NRF_read_rx_payload_width() {
	CSN0;
	SPIx_Transfer(R_RX_PL_WID);
	uint8_t answ = SPIx_Transfer(0xFF);
	CSN1;
	return answ;
 }
//***********************************************************************************************************
// Выполняет команду. Возвращает регистр статуса
uint8_t NRF_cmd(uint8_t cmd) {
	CSN0;
	uint8_t status = SPIx_Transfer(cmd);
	CSN1;
	return status;
 }
//***********************************************************************************************************
// Возвращает 1, если на линии IRQ активный (низкий) уровень.
uint8_t IRQ_is_interrupt(){	
	if (HAL_GPIO_ReadPin(SPI_GPIO, SPI_PIN_IRQ) == 0) {
		return 1;
		} else {
		return 0;
		}
 }
//***********************************************************************************************************
// Функция обработчика прерываний радиомодуля. Размещается в основном цикле
uint8_t check_NRF() {
	if (status_RX == 1){
		return status_RX == 1;
	}
	if (!(IRQ_is_interrupt()==1)){ 									// Если прерывания нет, то не задерживаемся
		return status_RX = 0;													// Возвращаем 0 как флаг что ничего нет
	}
//**********************************************
	uint8_t status = NRF_cmd(NOP);
//***********************************************
	if (status & (1 << RX_DR)){											// если пришел пакет
		uint8_t l = NRF_read_rx_payload_width();			// Узнаём длину пакета
		uint8_t buf[10]; 															// буфер для принятого пакета
		NRF_read_buf(R_RX_PAYLOAD, &buf[0], l); 			// начитывается пакет
		on_packet(&buf[0], l); 												// вызываем обработчик полученного пакета, который записывает полученные данные в массив buffer_RX[]
		NRF_cmd(FLUSH_RX);														// Сбросить очередь приёмника
		status = NRF_cmd(NOP);												// читаем статус
		NRF_writereg(STATUS, status);									// сбрасываем флаги записывая STATUS обратно
	}
	return status_RX = 1;														// Возвращаем 1 как флаг что данные нужно обработать
 }
//***********************************************************************************************************
// Вызывается, когда превышено число попыток отправки, а подтверждение так и не было получено.
void on_send_error() {
 // здесь можно описать обработчик неудачной отправки
 }
//***********************************************************************************************************
// Вызывается при получении нового пакета по каналу 1 от удалённой стороны. buf - буфер с данными, size - длина данных (от 1 до 32)
void on_packet(uint8_t * buf, uint8_t size) {
//	uint8_t i;									//
//	for (i=0; i<=size; i++){					//
//		buffer_RX[i] = buf[i];					// Рассомментировать, если нудно принимать данные
//	}											//
 }
//***********************************************************************************************************
// Помещает пакет в очередь отправки.
// buf - буфер с данными, size - длина данных (от 1 до 32)
uint8_t send_data_NRF(uint8_t * buf, uint8_t size) {
	CE0; 													// Если в режиме приёма, то выключаем его
	NRF_write_buf(W_TX_PAYLOAD, buf, size); 				// Запись данных на отправку	
	Conf_NRF_Tx(); 											// configuring as transmitter and send data
//							size_UART = sprintf((char *)Data, "I wait interrupt...\n\r");/////////////////////////////////////строка для отладки///////////////////////////////////////
//							HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);	///////////////////////////////////////////////строка для отладки////////////////////////////////////
	
 	while(IRQ_is_interrupt() != 1){ 						// Ждём 2 сек прерывания с квитанцией
		if (Tcounter1 >=20){
//							size_UART = sprintf((char *)Data, "I didnt wait interrupt(((\n\r");////////////////////////////////////строка для отладки/////////////////////////////////////
//							HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);			/////////////////////////////////////////////строка для отладки/////////////////////////////////////
			break;}
	}
		
	DelayMicro(40);
	uint8_t status = NRF_readreg(STATUS);					//read register STATUS
	if (status & (1 << TX_DS)){
		status_TX = 1;  									// Если всё хорошо, возвращаем  1 передача удалась		
//							size_UART = sprintf((char *)Data, "Status %X\n\r", status);/////////////////////////////////////строка для отладки///////////////////////////////////////////////
//							HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);		/////////////////////////////////////строка для отладки////////////////////////////////////////////////
		
						}	else {
//							size_UART = sprintf((char *)Data, "Transmit Bad!!!\n\r");/////////////////////////////////////строка для отладки///////////////////////////////////////////////
//							HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);		/////////////////////////////////////строка для отладки////////////////////////////////////////////////
//							size_UART = sprintf((char *)Data, "Status %X\n\r", status);/////////////////////////////////////строка для отладки///////////////////////////////////////////////
//							HAL_UART_Transmit(&huart2, Data, size_UART, 0xFFFF);		/////////////////////////////////////строка для отладки////////////////////////////////////////////////
						}			
	status = NRF_readreg(STATUS);							//read register STATUS
	NRF_writereg(STATUS, status);							//write register STATUS and clear him
	Conf_NRF_Rx();											// configuring as receiver
	return status_TX;  										// return status_TX
 }
//***********************************************************************************************************

uint8_t Conf_NRF_Rx(){	  //режим према RX
	  CE0;
	  NRF_cmd(NOP);
	  NRF_writereg(RX_PW_P0,RF_DATA_SIZE); //размер поля данных, байт.
//	  NRF_writereg(ENAA_P4,0); //выключить автоподтверждение по каналу 4
	  NRF_writereg(RF_CH, CHAN); 
	  NRF_writereg(RF_SETUP, RF_SPEED); // выбор скорости 250 kбит/с и мощности 0dBm
	  NRF_writereg(SETUP_RETR, SETUP_RETR_DELAY_1000MKS); // Delay for speed 250	kбит/с
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
		NRF_writereg(RX_PW_P0,RF_DATA_SIZE);//размер поля данных, байт.
//	  NRF_writereg(ENAA_P4,0);//выключить автоподтверждение по каналу 0
		NRF_writereg(RF_CH, CHAN); // Выбор частотного канала
	  NRF_writereg(RF_SETUP, RF_SPEED); // выбор скорости 250 kбит/с и мощности 0dBm
	  NRF_writereg(SETUP_RETR, SETUP_RETR_DELAY_1000MKS); // Delay for speed 250 kбит/с	
	  NRF_writereg_buf(RX_ADDR_P0, &remote_addr[0], 5); // Подтверждения приходят на канал 0
	  NRF_writereg_buf(TX_ADDR, &remote_addr[0], 5);
//	  NRF_writereg_buf(RX_ADDR_P0, &self_addr[0], 5);
	  NRF_writereg(CONFIG,(1<<PWR_UP)|(1<<EN_CRC)|(1<<CRCO)|(0<<PRIM_RX)|(0 << MASK_MAX_RT));
	  CE1;
	  DelayMicro(15);
	  CE0;
	  DelayMicro(140);
	  return (NRF_readreg(CONFIG) == ((1 << EN_CRC)|(1<<CRCO) | (1 << PWR_UP) | (0 << PRIM_RX) | (0 << MASK_MAX_RT))) ? 1 : 0;
 }


