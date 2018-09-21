/* ������� */
#define R_REGISTER          0x00 // + n ��������� ������� n
#define W_REGISTER          0x20 // + n �������� ������� n
#define R_RX_PAYLOAD        0x61 // ������� ������ ������ �� �������� ����� ������� ��������. 
#define W_TX_PAYLOAD        0xA0 // �������� � ������� ����������� ������ ��� ��������
#define FLUSH_TX            0xE1 // �������� ������� �����������
#define FLUSH_RX            0xE2 // �������� ������� ��������
#define REUSE_TX_PL         0xE3 // ������������ �������� ��������� ���������� �����
#define R_RX_PL_WID         0x60 // ��������� ������ ������ ��������� ������ � ������ ������� ��������. 
#define W_ACK_PAYLOAD       0xA8 // + p �������� ������ ��� �������� � ������� ������������� �� ������ p. 
#define W_TX_PAYLOAD_NOACK  0xB0 // �������� � ������� ����������� ������, ��� �������� ��� �������������
#define NOP                 0xFF // ��� ��������. ����� ���� ������������ ��� ������ �������� �������

/* �������� */
#define CONFIG      0x00 // ������� ��������
#define EN_AA       0x01 // ����� �����������������
#define EN_RXADDR   0x02 // ����� ������� ��������
#define SETUP_AW    0x03 // ��������� ������� ������
#define SETUP_RETR  0x04 // ��������� ��������� ��������
#define RF_CH       0x05 // ����� �����������, �� ������� �������������� ������. �� 0 �� 125. 
#define RF_SETUP    0x06 // ��������� �����������
#define STATUS      0x07 // ������� �������. 
#define OBSERVE_TX  0x08 // ���������� �������� �������� � ���������� �������
#define RPD         0x09 // �������� ������������ �������. ���� ������� ��� = 1, �� ������� ����� -64dBm 
#define RX_ADDR_P0  0x0A // 3-5 ���� (������� � �������� �����). ����� ������ 0 ��������. 
#define RX_ADDR_P1  0x0B // 3-5 ���� (������� � �������� �����). ����� ������ 1 ��������.
#define RX_ADDR_P2  0x0C // ������� ���� ������ ������ 2 ��������. ������� ����� �� RX_ADDR_P1
#define RX_ADDR_P3  0x0D // ������� ���� ������ ������ 3 ��������. ������� ����� �� RX_ADDR_P1
#define RX_ADDR_P4  0x0E // ������� ���� ������ ������ 4 ��������. ������� ����� �� RX_ADDR_P1
#define RX_ADDR_P5  0x0F // ������� ���� ������ ������ 5 ��������. ������� ����� �� RX_ADDR_P1
#define TX_ADDR     0x10 // 3-5 ���� (������� � �������� �����). ����� ��������� ���������� ��� ��������
#define RX_PW_P0    0x11 // ������ ������ ��� ����� �� ������ 0: �� 1 �� 32. 0 - ����� �� ������������.
#define RX_PW_P1    0x12 // ������ ������ ��� ����� �� ������ 1: �� 1 �� 32. 0 - ����� �� ������������.
#define RX_PW_P2    0x13 // ������ ������ ��� ����� �� ������ 2: �� 1 �� 32. 0 - ����� �� ������������.
#define RX_PW_P3    0x14 // ������ ������ ��� ����� �� ������ 3: �� 1 �� 32. 0 - ����� �� ������������.
#define RX_PW_P4    0x15 // ������ ������ ��� ����� �� ������ 4: �� 1 �� 32. 0 - ����� �� ������������.
#define RX_PW_P5    0x16 // ������ ������ ��� ����� �� ������ 5: �� 1 �� 32. 0 - ����� �� ������������.
#define FIFO_STATUS 0x17 // ��������� �������� FIFO �������� � �����������
#define DYNPD       0x1C // ����� ������� �������� ��� ������� ������������ ������������ ����� �������.
#define FEATURE     0x1D // ������� �����


/* ���� ��������� */
// CONFIG					00001111
#define MASK_RX_DR  6 // ��������� ���������� �� RX_DR (��������� ������)
#define MASK_TX_DS  5 // ��������� ���������� �� TX_DS (���������� �������� ������) 
#define MASK_MAX_RT 4 // ��������� ���������� �� MAX_RT (���������� ����� ��������� ������� ��������) 
#define EN_CRC      3 // �������� CRC
#define CRCO        2 // ������ ���� CRC: 0 - 1 ����; 1 - 2 �����
#define PWR_UP      1 // ��������� �������
#define PRIM_RX     0 // ����� ������: 0 - PTX (����������) 1 - PRX (�������)

// EN_AA
#define ENAA_P5 5 // �������� ����������������� ������, ���������� �� ������ 5
#define ENAA_P4 4 // �������� ����������������� ������, ���������� �� ������ 4
#define ENAA_P3 3 // �������� ����������������� ������, ���������� �� ������ 3
#define ENAA_P2 2 // �������� ����������������� ������, ���������� �� ������ 2
#define ENAA_P1 1 // �������� ����������������� ������, ���������� �� ������ 1
#define ENAA_P0 0 // �������� ����������������� ������, ���������� �� ������ 0

// EN_RXADDR
#define ERX_P5 5 // �������� ����� 5 ��������
#define ERX_P4 4 // �������� ����� 4 �������� 
#define ERX_P3 3 // �������� ����� 3 �������� 
#define ERX_P2 2 // �������� ����� 2 �������� 
#define ERX_P1 1 // �������� ����� 1 �������� 
#define ERX_P0 0 // �������� ����� 0 �������� 

// SETUP_AW
#define AW 0 // ��� ����, �������� ������ ���� ������: 1 - 3 �����; 2 - 4 �����; 3 - 5 ����.

#define SETUP_AW_3BYTES_ADDRESS (1 << AW)
#define SETUP_AW_4BYTES_ADDRESS (2 << AW)
#define SETUP_AW_5BYTES_ADDRESS (3 << AW)

// SETUP_RETR 
#define ARD 4 // 4 ����. ����� �������� �������� ����� ��������� ��������� ������: 250 x (n + 1) ���
#define ARC 0 // 4 �����. ���������� ��������� ������� ��������, 0 - ��������� �������� ���������.

#define SETUP_RETR_DELAY_250MKS  (0 << ARD)
#define SETUP_RETR_DELAY_500MKS  (1 << ARD)
#define SETUP_RETR_DELAY_750MKS  (2 << ARD)
#define SETUP_RETR_DELAY_1000MKS (3 << ARD)
#define SETUP_RETR_DELAY_1250MKS (4 << ARD)
#define SETUP_RETR_DELAY_1500MKS (5 << ARD)
#define SETUP_RETR_DELAY_1750MKS (6 << ARD)
#define SETUP_RETR_DELAY_2000MKS (7 << ARD)
#define SETUP_RETR_DELAY_2250MKS (8 << ARD)
#define SETUP_RETR_DELAY_2500MKS (9 << ARD)
#define SETUP_RETR_DELAY_2750MKS (10 << ARD)
#define SETUP_RETR_DELAY_3000MKS (11 << ARD)
#define SETUP_RETR_DELAY_3250MKS (12 << ARD)
#define SETUP_RETR_DELAY_3500MKS (13 << ARD)
#define SETUP_RETR_DELAY_3750MKS (14 << ARD)
#define SETUP_RETR_DELAY_4000MKS (15 << ARD)

#define SETUP_RETR_NO_RETRANSMIT (0 << ARC)
#define SETUP_RETR_UP_TO_1_RETRANSMIT (1 << ARC)
#define SETUP_RETR_UP_TO_2_RETRANSMIT (2 << ARC)
#define SETUP_RETR_UP_TO_3_RETRANSMIT (3 << ARC)
#define SETUP_RETR_UP_TO_4_RETRANSMIT (4 << ARC)
#define SETUP_RETR_UP_TO_5_RETRANSMIT (5 << ARC)
#define SETUP_RETR_UP_TO_6_RETRANSMIT (6 << ARC)
#define SETUP_RETR_UP_TO_7_RETRANSMIT (7 << ARC)
#define SETUP_RETR_UP_TO_8_RETRANSMIT (8 << ARC)
#define SETUP_RETR_UP_TO_9_RETRANSMIT (9 << ARC)
#define SETUP_RETR_UP_TO_10_RETRANSMIT (10 << ARC)
#define SETUP_RETR_UP_TO_11_RETRANSMIT (11 << ARC)
#define SETUP_RETR_UP_TO_12_RETRANSMIT (12 << ARC)
#define SETUP_RETR_UP_TO_13_RETRANSMIT (13 << ARC)
#define SETUP_RETR_UP_TO_14_RETRANSMIT (14 << ARC)
#define SETUP_RETR_UP_TO_15_RETRANSMIT (15 << ARC)

// RF_SETUP
#define CONT_WAVE   7 // (������ ��� nRF24L01+) ����������� �������� ������� (��� ������)
#define RF_DR_LOW   5 // (������ ��� nRF24L01+) �������� �������� 250����/�. RF_DR_HIGH ������ ���� 0
#define PLL_LOCK    4 // ��� ������
#define RF_DR_HIGH  3 // ����� �������� ������ (��� �������� ���� RF_DR_LOW = 0): 0 - 1����/�; 1 - 2����/�
#define RF_PWR      1 // 2����. �������� �������� �����������: 0 - -18dBm; 1 - -16dBm; 2 - -6dBm; 3 - 0dBm

#define RF_SETUP_MINUS18DBM (0 << RF_PWR)
#define RF_SETUP_MINUS12DBM (1 << RF_PWR)
#define RF_SETUP_MINUS6DBM  (2 << RF_PWR)
#define RF_SETUP_0DBM       (3 << RF_PWR)

#define RF_SETUP_1MBPS (0 << RF_DR_HIGH)|(0 << RF_DR_LOW)
#define RF_SETUP_2MBPS (1 << RF_DR_HIGH)|(0 << RF_DR_LOW)
#define RF_SETUP_250KBPS (1 << RF_DR_LOW)|(0 << RF_DR_HIGH) 

// STATUS
#define RX_DR   6 // ���� ��������� ����� ������ � FIFO ��������. ��� ������ ����� ����� �������� 1
#define TX_DS   5 // ���� ���������� ��������. ��� ������ ����� ����� �������� 1
#define MAX_RT  4 // ���� ���������� �������������� ����� ��������. ��� ������ (�������� 1) ����� ����������
#define RX_P_NO 1 // 3 ����. ����� ������, ������ ��� �������� �������� � FIFO ��������. 7 -  FIFO �����.
#define TX_FULL_STATUS 0 // ������� ���������� FIFO �����������: 1 - ���������; 0 - ���� ��������� ����� 
        // (������������� �� TX_FULL �� ��������� �������� � ���������� ����� �� �������� FIFO_STATUS)

// OBSERVE_TX
#define PLOS_CNT  4 // 4 ����. ����� ���������� ������� ��� �������������. ������������ ������� RF_CH
#define ARC_CNT   0 // 4 ����. ���������� ������������ �������� ��� ��������� ��������. 

// FIFO_STATUS
#define TX_REUSE      6 // ������� ���������� ���������� ������ ��� ��������� ��������. 
#define TX_FULL_FIFO  5 // ���� ������������ FIFO ������� �����������.
        // (������������� �� TX_FULL �� ��������� �������� � ���������� ����� �� �������� STATUS)
#define TX_EMPTY      4 // ���� ������������ FIFO ������� �����������.
#define RX_FULL       1 // ���� ������������ FIFO ������� ��������.
#define RX_EMPTY      0 // ���� ������������ FIFO ������� ��������.

// DYNDP
#define DPL_P5 5 // �������� ���� ������� ������������ ����� �� ������ 5
#define DPL_P4 4 // �������� ���� ������� ������������ ����� �� ������ 4
#define DPL_P3 3 // �������� ���� ������� ������������ ����� �� ������ 3
#define DPL_P2 2 // �������� ���� ������� ������������ ����� �� ������ 2
#define DPL_P1 1 // �������� ���� ������� ������������ ����� �� ������ 1
#define DPL_P0 0 // �������� ���� ������� ������������ ����� �� ������ 0

// FEATURE  
#define EN_DPL      2 // �������� ��������� ����� � �������� ������� ������������ �����
#define EN_ACK_PAY  1 // ��������� �������� ������ � �������� ������������� �����
#define EN_DYN_ACK  0 // ��������� ������������� W_TX_PAYLOAD_NOACK

unsigned char SPIx_Transfer(unsigned char data);
unsigned char Conf_NRF_Tx(void);
unsigned char Conf_NRF_Rx(void);
unsigned char NRF_read_buf(unsigned char cmd, unsigned char * buf, unsigned char count);// ��������� ������� cmd, � ������ count ���� ������, ������� �� � ����� buf, ���������� ������� �������
unsigned char NRF_readreg_STATUC(void);// ������ ���� � ���������� ������� �������
unsigned char NRF_writereg(unsigned char reg, unsigned char val);// ���������� �������� ������������� �������� reg (�� 0 �� 31), ���������� ������� �������
unsigned char NRF_readreg(unsigned char reg);// ������ ���� ������������� �������� reg (�� 0 �� 31) � ���������� ���
unsigned char NRF_write_buf(unsigned char cmd, unsigned char * buf, unsigned char count);// ��������� ������� cmd, � ������� count ���� ���������� �� ������ buf, ���������� ������� �������
unsigned char NRF_read_rx_payload_width(void);// ���������� ������ ������ � ������ FIFO ������� ��������
unsigned char NRF_cmd(unsigned char cmd);// ��������� �������. ���������� ������� �������
unsigned char IRQ_is_interrupt(void);// ���������� 1, ���� �� ����� IRQ �������� (������) �������.
unsigned char NRF_writereg_buf(unsigned char reg, unsigned char * buf, unsigned char count);// ���������� count ���� �� ������ buf � ������������� ������� reg (�� 0 �� 31), ���������� ������� �������
unsigned char send_data_NRF(unsigned char * buf, unsigned char size);// �������� ����� � ������� ��������. buf - ����� � �������, size - ����� ������ (�� 1 �� 32)
unsigned char check_NRF(void);// ������� ����������� ���������� �����������. ����������� � �������� �����
void on_send_error(void);// ����������, ����� ��������� ����� ������� ��������, � ������������� ��� � �� ���� ��������.
void on_packet(unsigned char * buf, unsigned char size);// ���������� ��� ��������� ������ ������ �� ������ 1 �� �������� �������.
