// Simple ADC example to read temperature, battery voltage, and ADC channel 0 (PA4)
#include "ch32fun.h"
#include "../uart_send_receive/fun_uart_ch5xx.h"

#define TARGET_UART &R32_UART1_CTRL

void printf(const char *str) {
	uart_send_ch5xx(TARGET_UART, str, strlen(str));
}

#define USB_IRQ_FLAG_NUM     4
#define THIS_ENDP0_SIZE         64
#define MAX_PACKET_SIZE         64

#ifndef USB_GET_DESCRIPTOR
#define USB_GET_STATUS          0x00
#define USB_CLEAR_FEATURE       0x01
#define USB_SET_FEATURE         0x03
#define USB_SET_ADDRESS         0x05
#define USB_GET_DESCRIPTOR      0x06
#define USB_SET_DESCRIPTOR      0x07
#define USB_GET_CONFIGURATION   0x08
#define USB_SET_CONFIGURATION   0x09
#define USB_GET_INTERFACE       0x0A
#define USB_SET_INTERFACE       0x0B
#define USB_SYNCH_FRAME         0x0C
#endif

/* OUT */
#define OUT_ACK                         0
#define OUT_TIMOUT                      1
#define OUT_NAK                         2
#define OUT_STALL                       3
/* IN */
#define IN_ACK                          0
#define IN_NORSP                        1
#define IN_NAK                          2
#define IN_STALL                        3

#define ENDP_TYPE_IN                    0x00                                    /* ENDP is IN Type */
#define ENDP_TYPE_OUT                   0x01                                    /* ENDP is OUT Type */

#define ENDP0                           0x00
#define ENDP1                           0x01
#define ENDP2                           0x02
#define ENDP3                           0x03
#define ENDP4                           0x04

#define DEF_NETWORK_CONNECTION         0x00
#define DEF_RESPONSE_AVAILABLE         0x01
#define DEF_SERIAL_STATE               0x20

#define DEF_SEND_ENCAPSULATED_COMMAND  0x00
#define DEF_GET_ENCAPSULATED_RESPONSE  0x01
#define DEF_SET_COMM_FEATURE           0x02
#define DEF_GET_COMM_FEATURE           0x03
#define DEF_CLEAR_COMM_FEATURE         0x04
#define DEF_SET_LINE_CODING          0x20   // Configures DTE rate, stop-bits, parity, and number-of-character
#define DEF_GET_LINE_CODING          0x21   // This request allows the host to find out the currently configured line coding.
//#define DEF_SET_CTL_LINE_STE         0X22   // This request generates RS-232/V.24 style control signals.
#define DEF_SET_CONTROL_LINE_STATE     0x22
#define DEF_SEND_BREAK                 0x23

#define DEF_BIT_USB_RESET               0x01 
#define DEF_BIT_USB_DEV_DESC            0x02 
#define DEF_BIT_USB_ADDRESS             0x04 
#define DEF_BIT_USB_CFG_DESC            0x08 
#define DEF_BIT_USB_SET_CFG             0x10 
#define DEF_BIT_USB_WAKE                0x20 
#define DEF_BIT_USB_SUPD                0x40 
#define DEF_BIT_USB_HS                  0x80 

#define DEF_VEN_DEBUG_READ              0X95
#define DEF_VEN_DEBUG_WRITE             0X9A
#define DEF_VEN_UART_INIT             0XA1  
#define DEF_VEN_UART_M_OUT              0XA4
#define DEF_VEN_BUF_CLEAR             0XB2  
#define DEF_VEN_I2C_CMD_X             0X54  
#define DEF_VEN_DELAY_MS              0X5E  
#define DEF_VEN_GET_VER                 0X5F


#define HAL_UART_NO_PARITY                   0x00
#define HAL_UART_ODD_PARITY                  0x01
#define HAL_UART_EVEN_PARITY                 0x02
#define HAL_UART_MARK_PARITY                 0x03
#define HAL_UART_SPACE_PARITY                0x04

#define HAL_UART_5_BITS_PER_CHAR             5
#define HAL_UART_6_BITS_PER_CHAR             6
#define HAL_UART_7_BITS_PER_CHAR             7
#define HAL_UART_8_BITS_PER_CHAR             8

#define HAL_UART_ONE_STOP_BIT                1
#define HAL_UART_TWO_STOP_BITS               2

#ifndef USB_REQ_TYP_MASK
#define USB_REQ_TYP_IN          0x80            /* control IN, device to host */
#define USB_REQ_TYP_OUT         0x00            /* control OUT, host to device */
#define USB_REQ_TYP_READ        0x80            /* control read, device to host */
#define USB_REQ_TYP_WRITE       0x00            /* control write, host to device */
#define USB_REQ_TYP_MASK        0x60            /* bit mask of request type */
#define USB_REQ_TYP_STANDARD    0x00
#define USB_REQ_TYP_CLASS       0x20
#define USB_REQ_TYP_VENDOR      0x40
#define USB_REQ_TYP_RESERVED    0x60
#define USB_REQ_RECIP_MASK      0x1F            /* bit mask of request recipient */
#define USB_REQ_RECIP_DEVICE    0x00
#define USB_REQ_RECIP_INTERF    0x01
#define USB_REQ_RECIP_ENDP      0x02
#define USB_REQ_RECIP_OTHER     0x03
#endif

#define DEF_IC_PRG_VER                 0x31

typedef unsigned char           *PUINT8;

static  PUINT8  pDescr;

const u8 TAB_USB_CDC_DEV_DES[18] = {
	0x12,
	0x01,
	0x10,
	0x01,
	0x02,
	0x00,
	0x00,
	0x40,
	0x86, 0x1a,
	0x40, 0x80,
	0x00, 0x30,
	0x01,
	0x02,
	0x03,
	0x01 
};

const u8 My_QueDescr[ ] = { 0x0A, 0x06, 0x00, 0x02, 0xFF, 0x00, 0xFF, 0x40, 0x01, 0x00 };

const u8 TAB_USB_CDC_CFG_DES[] = {
	0x09,0x02,0x43,0x00,0x02,0x01,0x00,0x80,0x30,

	0x09, 0x04,0x00,0x00,0x01,0x02,0x02,0x01,0x00,

	0x05,0x24,0x00,0x10,0x01,
	0x04,0x24,0x02,0x02,
	0x05,0x24,0x06,0x00,0x01,
	0x05,0x24,0x01,0x01,0x00,

	0x07,0x05,0x84,0x03,0x08,0x00,0x01,

	0x09,0x04,0x01,0x00,0x02,0x0a,0x00,0x00,0x00,

	0x07,0x05,0x01,0x02,0x40,0x00,0x00,
	0x07,0x05,0x81,0x02,0x40,0x00,0x00,
};

const u8 TAB_USB_VEN_DEV_DES[] = {
	0x12,
	0x01,
	0x10, 0x01,
	0xff, 0x00, 0x02, 0x40,//0x40,
	0x86, 0x1a, 0x23, 0x75,
	0x00, DEF_IC_PRG_VER,

	0x01,                
	0x02,                
	0x03,                

	0x01
};

const u8 TAB_USB_VEN_CFG_DES[39] = {
	0x09, 0x02, 0x27, 0x00, 0x01, 0x01, 0x00, 0x80, 0x30,
	0x09, 0x04, 0x00, 0x00, 0x03, 0xff, 0x01, 0x02, 0x00,
	0x07, 0x05, 0x82, 0x02, 0x20, 0x00, 0x00,
	0x07, 0x05, 0x02, 0x02, 0x20, 0x00, 0x00,
	0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x01
};


const u8 USB_DEV_PARA_CDC_SERIAL_STR[]=     "WCH121212TS13";
const u8 USB_DEV_PARA_CDC_PRODUCT_STR[]=    "USB2.0 To Serial Port3";
const u8 USB_DEV_PARA_CDC_MANUFACTURE_STR[]= "wch.cn3";
const u8 USB_DEV_PARA_VEN_SERIAL_STR[]=     "WCH454545TS2";
const u8 USB_DEV_PARA_VEN_PRODUCT_STR[]=   "USB2.0 To Serial Port";
const u8 USB_DEV_PARA_VEN_MANUFACTURE_STR[]= "wch.cn";

typedef struct DevInfo {
	u8 UsbConfig;    
	u8 UsbAddress;   
	u8 gSetupReq;    
	u8 gSetupLen;    
	u8 gUsbInterCfg; 
	u8 gUsbFlag;     
} DevInfo_Parm;


DevInfo_Parm  devinf;

u8 CDCSetSerIdx = 0;
u8 CDCSer0ParaChange = 0;

#define PACKED __attribute__((packed))

typedef struct PACKED {
	u32  BaudRate; 
	u8 StopBits;   
	u8 ParityType; 
	u8 DataBits;   
} LINE_CODE, *PLINE_CODE;

/* ��������������� */
LINE_CODE Uart0Para;

u8 usb_irq_w_idx = 0;
u8 usb_irq_r_idx = 0;

volatile u8 usb_irq_len[USB_IRQ_FLAG_NUM];
volatile u8 usb_irq_pid[USB_IRQ_FLAG_NUM];
volatile u8 usb_irq_flag[USB_IRQ_FLAG_NUM];

#define USB_CDC_MODE      0
#define USB_VENDOR_MODE   1

u8 usb_work_mode = USB_CDC_MODE; //USB_VENDOR_MODE;

u8 SetupReqCode, SetupLen;

typedef struct PACKED {
    u8 bRequestType;
    u8 bRequest;
    u16 wValue;
    u16 wIndex;
    u16 wLength;
} USB_SETUP_REQ;

typedef struct {
    u8 bRequestType;
    u8 bRequest;
    u8 wValueL;
    u8 wValueH;
    u8 wIndexL;
    u8 wIndexH;
    u8 wLengthL;
    u8 wLengthH;
} USB_SETUP_REQ_t;

#define UsbSetupBuf     ((USB_SETUP_REQ_t *)Ep0Buffer)

__attribute__((aligned(4))) u8  Ep0Buffer[MAX_PACKET_SIZE];
__attribute__((aligned(4))) u8  Ep1Buffer[2*MAX_PACKET_SIZE];   //OUT & IN
__attribute__((aligned(4))) u8  Ep2Buffer[2*MAX_PACKET_SIZE];   //OUT & IN
__attribute__((aligned(4))) u8  Ep3Buffer[2*MAX_PACKET_SIZE];   //OUT & IN

u8 Ep1DataINFlag = 0;
u8 Ep2DataINFlag = 0;

u8 Ep2DataOUTFlag = 0;
u8 Ep2DataOUTLen = 0;

u8 Ep1DataOUTFlag = 0;
u8 Ep1DataOUTLen = 0;

u8 Ep4DataINFlag;
u8 Ep3DataINFlag;

u8 Ep3DataOUTFlag = 0;
u8 Ep4DataOUTFlag = 0;

u8 VENSer0SendFlag = 0;

u8 cdc_uart_sta_trans_step = 0;
u8 ven_ep1_trans_step = 0;

u8 VENSer0ParaChange = 0;

__attribute__((aligned(4))) u8 Ep1OUTDataBuf[MAX_PACKET_SIZE];
__attribute__((aligned(4))) u8 Ep2OUTDataBuf[MAX_PACKET_SIZE];

u8 UART0_RTS_Val = 0;
u8 UART0_DTR_Val = 0;
u8 UART0_OUT_Val = 0;

u8 ep0_send_buf[256];

const u8 TAB_USB_LID_STR_DES[ ] = { 0x04, 0x03, 0x09, 0x04 };

const u8 TAB_USB_VEN_STR_DES[ ] = { 0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0 };
const u8 TAB_USB_PRD_STR_DES[ ] = {
	0x1e,0x03,0x55,0x00,0x53,0x00,0x42,0x00,0x20,0x00,0x43,0x00,0x44,0x00,0x43,0x00,
	0x2d,0x00,0x53,0x00,0x65,0x00,0x72,0x00,0x69,0x00,0x61,0x00,0x6c,0x00
};

typedef volatile unsigned char  *PUINT8V;

void USBDevEPnINSetStatus(u8 ep_num, u8 type, u8 sta) {
  u8 *p_UEPn_CTRL;

  p_UEPn_CTRL = (u8 *)(USB_BASE_ADDR + 0x22 + ep_num * 4);
  if(type == ENDP_TYPE_IN) *((PUINT8V)p_UEPn_CTRL) = (*((PUINT8V)p_UEPn_CTRL) & (~(0x03))) | sta;
  else *((PUINT8V)p_UEPn_CTRL) = (*((PUINT8V)p_UEPn_CTRL) & (~(0x03<<2))) | (sta<<2);
}


#define CH341_REG_NUM     10
u8 CH341_Reg_Add[CH341_REG_NUM];
u8 CH341_Reg_val[CH341_REG_NUM];

u8 CH341RegRead(u8 reg_add,u8 *reg_val) {
	u8 find_flag;
	u8 i;

	find_flag = 0;
	*reg_val = 0;

	for(i=0; i<CH341_REG_NUM; i++) {
		if(CH341_Reg_Add[i] == reg_add)   //�ҵ���ͬ��ַ�ļĴ���
		{
		find_flag = 1;
		*reg_val = CH341_Reg_val[i];
		break;
		}
		if(CH341_Reg_Add[i] == 0xff)      //�ҵ���ǰ��һ����
		{
		find_flag = 0;
		*reg_val = 0x00;
		break;
		}
	}

	switch(reg_add) {
		case 0x06:
			{
				u8  reg_pb_val = 0;
				*reg_val = reg_pb_val;
				break;
			}
		case 0x07:
			{
				u8  reg_pc_val = 0;
				*reg_val = reg_pc_val;
				break;
			}
		case 0x18:   //SFR_UART_CTRL -->���ڵĲ����Ĵ���
			{
				u8  reg_uart_ctrl_val;
				u8  ram_uart_ctrl_val;

				reg_uart_ctrl_val = R8_UART0_LCR;
				//����breakλ
				ram_uart_ctrl_val = *reg_val;
				reg_uart_ctrl_val |= (ram_uart_ctrl_val & 0x40);
				*reg_val = reg_uart_ctrl_val;

				break;
			}
		case 0x25:  break;
	}

	return find_flag;
}


void CH341RegWrite(u8 reg_add,u8 reg_val) {
	u8 find_idx;
	u8 find_flag;
	u8 i;

	find_flag = 0;
	find_idx = 0;

	for(i=0; i<CH341_REG_NUM; i++) {
		if(CH341_Reg_Add[i] == reg_add) {
			find_flag = 1;
			break;
		}
		if(CH341_Reg_Add[i] == 0xff) {
			find_flag = 0;
			break;
		}
	}
	find_idx = i;
	if(find_flag) {
		CH341_Reg_val[find_idx] = reg_val;
	}
	else {
		CH341_Reg_Add[find_idx] = reg_add;
		CH341_Reg_val[find_idx] = reg_val;
	}

	switch(reg_add) {
		case 0x06:break; //IO
		case 0x07:break; //IO
		case 0x18: //SFR_UART_CTRL -->���ڵĲ����Ĵ���
			{
				u8 reg_uart_ctrl;
				u8 data_bit_val;
				u8 stop_bit_val;
				u8 parity_val;
				u8 break_en;

				reg_uart_ctrl = reg_val;
				/* breakλ */
				break_en = (reg_uart_ctrl & 0x40)?(0):(1);
			//      SetUART0BreakENStatus(break_en);

				data_bit_val = reg_uart_ctrl & 0x03;
				if   (data_bit_val == 0x00)   data_bit_val = HAL_UART_5_BITS_PER_CHAR;
				else if(data_bit_val == 0x01) data_bit_val = HAL_UART_6_BITS_PER_CHAR;
				else if(data_bit_val == 0x02) data_bit_val = HAL_UART_7_BITS_PER_CHAR;
				else if(data_bit_val == 0x03) data_bit_val = HAL_UART_8_BITS_PER_CHAR;

				stop_bit_val = reg_uart_ctrl & 0x04;
				if(stop_bit_val) stop_bit_val = HAL_UART_TWO_STOP_BITS;
				else             stop_bit_val = HAL_UART_ONE_STOP_BIT;

				parity_val = reg_uart_ctrl & (0x38);
				if(parity_val == 0x00)      parity_val = HAL_UART_NO_PARITY;
				else if(parity_val == 0x08) parity_val = HAL_UART_ODD_PARITY;
				else if(parity_val == 0x18) parity_val = HAL_UART_EVEN_PARITY;
				else if(parity_val == 0x28) parity_val = HAL_UART_MARK_PARITY;
				else if(parity_val == 0x38) parity_val = HAL_UART_SPACE_PARITY;

				//Uart0Para.BaudRate;
				Uart0Para.StopBits = stop_bit_val;
				Uart0Para.ParityType = parity_val;
				Uart0Para.DataBits = data_bit_val;

				// printf("CH341 set para:%d %d %d break:%02x\r\n",data_bit_val,(int)stop_bit_val,parity_val,break_en);

				//ֱ�����üĴ���
				VENSer0ParaChange = 1;
				break;
			}
		case 0x25: break;
		case 0x27:
				// printf("modem set:%02x\r\n",reg_val);
			//      SetUART0ModemVendorSta(reg_val);
				break;
	}
}

typedef unsigned char           UINT8;
typedef unsigned long           UINT32;


void USB_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast"))) __attribute__((section(".highcode")))  __attribute((interrupt));
void USB_IRQHandler(void) {
	UINT8   i;
	UINT8   j;

	if(!(R8_USB_INT_FG & RB_UIF_TRANSFER)) { return; }

	if((R8_USB_INT_ST & MASK_UIS_TOKEN) != MASK_UIS_TOKEN){     // 非空闲
		usb_irq_flag[usb_irq_w_idx] = 1;
		usb_irq_pid[usb_irq_w_idx]  = R8_USB_INT_ST;  //& 0x3f;//(0x30 | 0x0F);
		usb_irq_len[usb_irq_w_idx]  = R8_USB_RX_LEN;

		switch(usb_irq_pid[usb_irq_w_idx] & 0x3f) {
		case UIS_TOKEN_OUT | 2:
			{
			printf("USB_IRQHandler 3\n");
			if( R8_USB_INT_FG & RB_U_TOG_OK ){
				R8_UEP2_CTRL ^=  RB_UEP_R_TOG;
				R8_UEP2_CTRL = (R8_UEP2_CTRL & 0xf3) | 0x08; //OUT_NAK
				for(j=0; j<(MAX_PACKET_SIZE/4); j++)
				((UINT32 *)Ep2OUTDataBuf)[j] = ((UINT32 *)Ep2Buffer)[j];
			}
			else usb_irq_flag[usb_irq_w_idx] = 0;
			break;
			}
		case UIS_TOKEN_IN | 2:
			{
			printf("USB_IRQHandler 4\n");
			R8_UEP2_CTRL ^=  RB_UEP_T_TOG;
			R8_UEP2_CTRL = (R8_UEP2_CTRL & 0xfc) | IN_NAK; //IN_NAK
			break;
			}
		case UIS_TOKEN_OUT | 1:
			{
			printf("USB_IRQHandler 5\n");
			if( R8_USB_INT_FG & RB_U_TOG_OK ){
				R8_UEP1_CTRL ^=  RB_UEP_R_TOG;
				R8_UEP1_CTRL = (R8_UEP1_CTRL & 0xf3) | 0x08; //OUT_NAK
				for(j=0; j<(MAX_PACKET_SIZE/4); j++)
				((UINT32 *)Ep1OUTDataBuf)[j] = ((UINT32 *)Ep1Buffer)[j];
			}
			else usb_irq_flag[usb_irq_w_idx] = 0;
			break;
			}
		case UIS_TOKEN_IN | 1:
			printf("USB_IRQHandler 6\n");
			R8_UEP1_CTRL ^=  RB_UEP_T_TOG;
			R8_UEP1_CTRL = (R8_UEP1_CTRL & 0xfc) | IN_NAK; //IN_NAK
			break;
		case UIS_TOKEN_OUT | 0:
			printf("\nUSB_IRQHandler 7\n");
			if( R8_USB_INT_FG & RB_U_TOG_OK )
			R8_UEP0_CTRL = (R8_UEP0_CTRL & 0xf3) | 0x08; //OUT_NAK
			else usb_irq_flag[usb_irq_w_idx] = 0;
			break;
		case UIS_TOKEN_IN | 0:
			printf("\nUSB_IRQHandler 8\n");
			R8_UEP0_CTRL = (R8_UEP0_CTRL & 0xfc) | IN_NAK; //IN_NAK
			break;
		}

		if (usb_irq_flag[usb_irq_w_idx]) {
		usb_irq_w_idx++;
		if(usb_irq_w_idx >= USB_IRQ_FLAG_NUM) usb_irq_w_idx = 0;
		}

		R8_USB_INT_FG = RB_UIF_TRANSFER;
	}

	if (R8_USB_INT_ST & RB_UIS_SETUP_ACT) {
		printf("\nUSB_IRQHandler 9\n");
		usb_irq_flag[usb_irq_w_idx] = 1;
		usb_irq_pid[usb_irq_w_idx]  = UIS_TOKEN_SETUP | 0;
		usb_irq_len[usb_irq_w_idx]  = 8;
		usb_irq_w_idx++;

		if(usb_irq_w_idx >= USB_IRQ_FLAG_NUM) usb_irq_w_idx = 0;
		R8_USB_INT_FG = RB_UIF_TRANSFER;
	}
}


void USBParaInit(void) {
	Ep1DataINFlag = 1;
	Ep1DataOUTFlag = 0;
	Ep2DataINFlag = 1;
	Ep2DataOUTFlag = 0;
	Ep3DataINFlag = 1;
	Ep3DataOUTFlag = 0;
	Ep4DataINFlag = 1;
	Ep4DataOUTFlag = 0;
}

UINT8 handle_USB_TYP_CLASS(UINT8 data_dir) {
	UINT8 len;

	if(data_dir == USB_REQ_TYP_OUT) {
		switch( SetupReqCode ) {
		case DEF_SET_LINE_CODING:
			{
			printf("\nSET_LINE_CODING\r\n");
			// for (UINT8 i=0; i<8; i++) printf("%02x ",Ep0Buffer[i]);
			printf("\r\n");

			if( Ep0Buffer[ 4 ] == 0x00 ) {
				CDCSetSerIdx = 0;
				len = 0x00;
			}
			else if( Ep0Buffer[ 4 ] == 0x02 ) {
				CDCSetSerIdx = 1;
				len = 0x00;
			}
			else len = 0xFF;
			break;
			}
		case DEF_SET_CONTROL_LINE_STATE:  /* SET_CONTROL_LINE_STATE */
			{
			UINT8 carrier_sta, present_sta;
			// printf("\ncontrol* %02x %02x\r\n",Ep0Buffer[2],Ep0Buffer[3]);
			carrier_sta = Ep0Buffer[2] & (1<<1);   //RTS状态
			present_sta = Ep0Buffer[2] & (1<<0);   //DTR状态
			len = 0;
			break;
			}
		default:
			// printf("CDC ReqCode%x\r\n",SetupReqCode);
			len = 0xFF;
			break;
		}
	}
	else {
		switch( SetupReqCode ) {
		case DEF_GET_LINE_CODING:
			// printf("\nGET_LINE_CODING:%d\r\n",Ep0Buffer[ 4 ]);
			pDescr = Ep0Buffer;
			len = sizeof( LINE_CODE );
			( ( PLINE_CODE )Ep0Buffer )->BaudRate   = Uart0Para.BaudRate;
			( ( PLINE_CODE )Ep0Buffer )->StopBits   = Uart0Para.StopBits;
			( ( PLINE_CODE )Ep0Buffer )->ParityType = Uart0Para.ParityType;
			( ( PLINE_CODE )Ep0Buffer )->DataBits   = Uart0Para.DataBits;
			break;

		case DEF_SERIAL_STATE:
			// printf("GET_SERIAL_STATE:%d\r\n",Ep0Buffer[ 4 ]);
			len = 2;
			CDCSetSerIdx = 0;
			Ep0Buffer[0] = 0;
			Ep0Buffer[1] = 0;
			break;

		default:
			// printf("CDC ReqCode%x\r\n",SetupReqCode);
			len = 0xFF;
			break;
		}
	}

	return len;
}

UINT8 handle_USB_TYP_VENDOR(UINT8 data_dir) {
	UINT8 len;
	UINT32  bps;
	UINT8   buf[8];

	switch(SetupReqCode) {
		case DEF_VEN_DEBUG_WRITE:
		{
			printf("IM HERE a1\n");
			UINT32 bps = 0;
			UINT32 CalClock, CalDiv;
			UINT8 wIndexL = UsbSetupBuf->wIndexL;
			UINT8 wIndexH = UsbSetupBuf->wIndexH;

			len = 0;
			UINT8 write_reg_add1 = Ep0Buffer[2];
			UINT8 write_reg_add2 = Ep0Buffer[3];
			UINT8 write_reg_val1 = Ep0Buffer[4];
			UINT8 write_reg_val2 = Ep0Buffer[5];

			if ((write_reg_add1 == 0x12) && (write_reg_add2 == 0x13)) {
			if((wIndexL==0x87)&&(wIndexH==0xf3)) {
				bps = 921600;  //13 * 921600 = 11980800
			}
			else if((wIndexL==0x87)&&(wIndexH==0xd9)) {
				bps = 307200;  //39 * 307200 = 11980800
			}

			else if( wIndexL == 0x88 ) {
				CalClock = 36923077 / 8;
				CalDiv = 0 - wIndexH;
				bps = CalClock / CalDiv;
			}
			else if( wIndexL == 0x89 ) {
				CalClock = 36923077 / 8 / 256;
				CalDiv = 0 - wIndexH;
				bps = CalClock / CalDiv;
			}
			else if( wIndexL == 0x8A ) {
				CalClock = 32000000 / 8;
				CalDiv = 0 - wIndexH;
				bps = CalClock / CalDiv;
			}
			else if( wIndexL == 0x8B ) {
				CalClock = 32000000 / 8 / 256;
				CalDiv = 0 - wIndexH;
				bps = CalClock / CalDiv;
			}
			else {
				//115384
				if((wIndexL & 0x7f) == 3) {
				CalClock = 6000000;
				CalDiv = 0 - wIndexH;
				bps = CalClock / CalDiv;
				}
				else if((wIndexL & 0x7f) == 2) {
				CalClock = 750000;  //6000000 / 8
				CalDiv = 0 - wIndexH;
				bps = CalClock / CalDiv;
				}
				else if((wIndexL & 0x7f) == 1) {
				CalClock = 93750; //64 分频
				CalDiv = 0 - wIndexH;
				bps = CalClock / CalDiv;
				}
				else if((wIndexL & 0x7f) == 0) {
				CalClock = 11719;  //约512
				CalDiv = 0 - wIndexH;
				bps = CalClock / CalDiv;
				}
				else {
				bps = 115200;
				}
			}
			Uart0Para.BaudRate = bps;
			// printf("CH341 set bps:%d\r\n",(int)bps);
			//UART0BpsSet(bps);
			}
			else {
			CH341RegWrite(write_reg_add1,write_reg_val1);
			CH341RegWrite(write_reg_add2,write_reg_val2);
			}

			break;
		}
		case DEF_VEN_DEBUG_READ:
		{
			printf("IM HERE a2\n");
			UINT8 read_reg_add1, read_reg_add2;
			UINT8 read_reg_val1, read_reg_val2;

			read_reg_add1 = UsbSetupBuf->wValueL;
			read_reg_add2 = UsbSetupBuf->wValueH;

			CH341RegRead(read_reg_add1,&read_reg_val1);
			CH341RegRead(read_reg_add2,&read_reg_val2);

			len = 2;
			pDescr = buf;
			buf[0] = read_reg_val1;
			buf[1] = read_reg_val2;
			SetupLen = len;
			memcpy(Ep0Buffer, pDescr, len);

			break;
		}
		case DEF_VEN_UART_INIT:  //初始化串口 0XA1
		{
			printf("IM HERE a3\n");
			UINT8 reg_uart_ctrl;
			UINT8  parity_val, data_bit_val, stop_bit_val;
			UINT8  uart_reg1_val, uart_reg2_val, uart_set_m;

			len = 0;

			if(Ep0Buffer[2] & 0x80) {
			reg_uart_ctrl = Ep0Buffer[3];

			data_bit_val = reg_uart_ctrl & 0x03;
			if   (data_bit_val == 0x00) data_bit_val = HAL_UART_5_BITS_PER_CHAR;
			else if(data_bit_val == 0x01) data_bit_val = HAL_UART_6_BITS_PER_CHAR;
			else if(data_bit_val == 0x02) data_bit_val = HAL_UART_7_BITS_PER_CHAR;
			else if(data_bit_val == 0x03) data_bit_val = HAL_UART_8_BITS_PER_CHAR;

			stop_bit_val = reg_uart_ctrl & 0x04;
			if(stop_bit_val) stop_bit_val = HAL_UART_TWO_STOP_BITS;
			else stop_bit_val = HAL_UART_ONE_STOP_BIT;

			parity_val = reg_uart_ctrl & (0x38);
			if(parity_val == 0x00) parity_val = HAL_UART_NO_PARITY;
			else if(parity_val == 0x08) parity_val = HAL_UART_ODD_PARITY;
			else if(parity_val == 0x18) parity_val = HAL_UART_EVEN_PARITY;
			else if(parity_val == 0x28) parity_val = HAL_UART_MARK_PARITY;
			else if(parity_val == 0x38) parity_val = HAL_UART_SPACE_PARITY;

			//Uart0Para.BaudRate;
			Uart0Para.StopBits = stop_bit_val;
			Uart0Para.ParityType = parity_val;
			Uart0Para.DataBits = data_bit_val;

			//直接设置寄存器
			// UART0ParaSet(data_bit_val, stop_bit_val,parity_val);

			uart_set_m = 0;
			uart_reg1_val = UsbSetupBuf->wIndexL;
			uart_reg2_val = UsbSetupBuf->wIndexH;

			if(uart_reg1_val & (1<<6)) {
				uart_set_m = 1;
			}
			else {
				uart_set_m = 1;
				uart_reg1_val = uart_reg1_val & 0xC7;
			}

			if (uart_set_m) {
				UINT32 CalClock, CalDiv;

				if((uart_reg1_val == 0x87)&&(uart_reg2_val == 0xf3)) {
				bps = 921600;  //13 * 921600 = 11980800
				}
				else if((uart_reg1_val == 0x87)&&(uart_reg2_val == 0xd9)) {
				bps = 307200;  //39 * 307200 = 11980800
				}

				else if( uart_reg1_val == 0xC8 ) {
				CalClock = 36923077 / 8;
				CalDiv = 0 - uart_reg2_val;
				bps = CalClock / CalDiv;
				}
				else if( uart_reg1_val == 0xC9 ) {
				CalClock = 36923077 / 8 / 256;
				CalDiv = 0 - uart_reg2_val;
				bps = CalClock / CalDiv;
				}
				else if( uart_reg1_val == 0xCA ) {
				CalClock = 32000000 / 8;
				CalDiv = 0 - uart_reg2_val;
				bps = CalClock / CalDiv;
				}
				else if( uart_reg1_val == 0xCB ) {
				CalClock = 32000000 / 8 / 256;
				CalDiv = 0 - uart_reg2_val;
				bps = CalClock / CalDiv;
				}
				else {
				//115384
				if((uart_reg1_val & 0x7f) == 3) {
					CalClock = 6000000;
					CalDiv = 0 - uart_reg2_val;
					bps = CalClock / CalDiv;
				}
				else if((uart_reg1_val & 0x7f) == 2) {
					CalClock = 750000;  //6000000 / 8
					CalDiv = 0 - uart_reg2_val;
					bps = CalClock / CalDiv;
				}
				else if((uart_reg1_val & 0x7f) == 1) {
					CalClock = 93750; //64 分频
					CalDiv = 0 - uart_reg2_val;
					bps = CalClock / CalDiv;
				}
				else if((uart_reg1_val & 0x7f) == 0) {
					CalClock = 11719;  //约512
					CalDiv = 0 - uart_reg2_val;
					bps = CalClock / CalDiv;
				}
				else {
					bps = 115200;
				}
				}
				Uart0Para.BaudRate = bps;
				// printf("CH341 set bps:%d\r\n",(int)bps);
				//UART0BpsSet(bps);
			}
			}
			break;
		}
		case DEF_VEN_UART_M_OUT:
		{
			printf("IM HERE 222\n");
			UINT8 reg_pb_out;
			len = 0;
			reg_pb_out = Ep0Buffer[2];
			if(reg_pb_out & (1<<4)) UART0_OUT_Val = 1;
			else UART0_OUT_Val = 0;
			break;
		}
		case DEF_VEN_BUF_CLEAR:
		{
			printf("IM HERE 333\n");
			len = 0;
			VENSer0ParaChange = 1;
			break;
		}
		case DEF_VEN_I2C_CMD_X:
		printf("IM HERE 444\n");
		len = 0; break;
		case DEF_VEN_DELAY_MS:
		printf("IM HERE 555\n");
		len = 0; break;
		case DEF_VEN_GET_VER:
		{
			printf("IM HERE 666\n");
			len = 2;
			pDescr = buf;
			buf[0] = 0x30;
			buf[1] = 0x00;
			SetupLen = len;
			memcpy( Ep0Buffer, pDescr, len );
			break;
		}
		default:
		printf("IM HERE 777\n");
		//len = 0xFF;
		len = 0;
		break;
	}

	return len;
}

UINT8 handle_USB_TYP_STANDARD(UINT8 data_dir) {
	UINT8 wValueL = UsbSetupBuf->wValueL;
	UINT8 len;

	switch( SetupReqCode ) {
		case USB_GET_DESCRIPTOR:
		{
			switch( UsbSetupBuf->wValueH ) {
			case 1:
				{
				printf("\nUSB_GET_DESCRIPTOR A\n");

				if(usb_work_mode == USB_VENDOR_MODE) {
					memcpy(ep0_send_buf, &TAB_USB_VEN_DEV_DES[0], sizeof( TAB_USB_VEN_DEV_DES ));
					pDescr = ep0_send_buf;
					len = sizeof( TAB_USB_VEN_DEV_DES );
				}
				else {
					memcpy(ep0_send_buf, &TAB_USB_CDC_DEV_DES[0], sizeof( TAB_USB_CDC_DEV_DES ));
					pDescr = ep0_send_buf;
					len = sizeof( TAB_USB_CDC_DEV_DES );
				}
				break;
				}
			case 2:
				{
				printf("\nUSB_GET_DESCRIPTOR B\n");

				if(usb_work_mode == USB_VENDOR_MODE) {
					memcpy(ep0_send_buf, &TAB_USB_VEN_CFG_DES[0], sizeof( TAB_USB_VEN_CFG_DES ));
					pDescr = ep0_send_buf;
					len = sizeof( TAB_USB_VEN_CFG_DES );
				}
				else {
					memcpy(ep0_send_buf, &TAB_USB_CDC_CFG_DES[0], sizeof( TAB_USB_CDC_CFG_DES ));
					pDescr = ep0_send_buf;
					len = sizeof( TAB_USB_CDC_CFG_DES );
				}
				break;
				}
			case 3:  // 字符串描述符
				{
				// printf("\nUSB_GET_DESCRIPTOR C: %d\r\n", wValueL);

				switch(wValueL) {
					case 0:
					pDescr = (PUINT8)( &TAB_USB_LID_STR_DES[0] );
					len = sizeof( TAB_USB_LID_STR_DES );
					break;
					case 1:  //iManufacturer
					case 2:   //iProduct
					case 3:   //iSerialNumber
					{
						UINT8 *manu_str;

						if(usb_work_mode == USB_VENDOR_MODE) {
						if(wValueL == 1)
							manu_str = (UINT8 *)USB_DEV_PARA_VEN_MANUFACTURE_STR;
						else if(wValueL == 2)
							manu_str = (UINT8 *)USB_DEV_PARA_VEN_PRODUCT_STR;
						else if(wValueL == 3)
							manu_str = (UINT8 *)USB_DEV_PARA_VEN_SERIAL_STR;
						}
						else {
						if(wValueL == 1)
							manu_str = (UINT8 *)USB_DEV_PARA_CDC_MANUFACTURE_STR;
						else if(wValueL == 2)
							manu_str = (UINT8 *)USB_DEV_PARA_CDC_PRODUCT_STR;
						else if(wValueL == 3)
							manu_str = (UINT8 *)USB_DEV_PARA_CDC_SERIAL_STR;
						}

						UINT8 str_len = (UINT8)strlen((char *)manu_str);
						ep0_send_buf[0] = str_len * 2 + 2;  // Total length: (chars * 2) + 2 header bytes
						ep0_send_buf[1] = 0x03;             // String descriptor type

						for(UINT8 i = 0; i < str_len; i++) {
						ep0_send_buf[2 + i*2] = manu_str[i];    // Character (low byte)
						ep0_send_buf[3 + i*2] = 0x00;           // Null (high byte)
						}

						// printf("manu_str: %s\n", manu_str);

						pDescr = ep0_send_buf;
						len = ep0_send_buf[0];
						break;
					}
					default:
					len = 0xFF; break;
				}

				break;
				}
			case 6:
				printf("USB_GET_DESCRIPTOR D\n");
				pDescr = (PUINT8)( &My_QueDescr[0] );
				len = sizeof( My_QueDescr );
				break;
			default:
				printf("USB_GET_DESCRIPTOR E\n");
				len = 0xFF; break;
			}

			if ( SetupLen > len ) SetupLen = len;
			len = (SetupLen >= THIS_ENDP0_SIZE) ? THIS_ENDP0_SIZE : SetupLen;
			memcpy( Ep0Buffer, pDescr, len );
			SetupLen -= len;
			pDescr += len;
			break;
		}
		case USB_SET_ADDRESS:
		// printf("\nSET_ADDRESS: %d\r\n", UsbSetupBuf->wValueL);
		devinf.gUsbFlag |= DEF_BIT_USB_ADDRESS;
		devinf.UsbAddress = UsbSetupBuf->wValueL;
		break;

		case USB_GET_CONFIGURATION:
		printf("\nGET_CONFIGURATION\r\n");
		Ep0Buffer[0] = devinf.UsbConfig;
		if ( SetupLen >= 1 ) len = 1;
		break;

		case USB_SET_CONFIGURATION:
		// printf("\nSET_CONFIGURATION: %d\r\n", UsbSetupBuf->wValueL);
		devinf.gUsbFlag |= DEF_BIT_USB_SET_CFG;
		devinf.UsbConfig = UsbSetupBuf->wValueL;
		break;

		case USB_CLEAR_FEATURE:
		{
			printf("CLEAR_FEATURE\r\n");
			len = 0;

			if( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE ) {
			R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
			R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
			R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;
			R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK;

			Ep1DataINFlag = 1;
			Ep2DataINFlag = 1;
			Ep3DataINFlag = 1;
			Ep4DataINFlag = 1;

			Ep1DataOUTFlag = 0;
			Ep2DataOUTFlag = 0;
			Ep3DataOUTFlag = 0;
			Ep4DataOUTFlag = 0;

			cdc_uart_sta_trans_step = 0;
			ven_ep1_trans_step = 0;
			}
			else if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP ) {
			switch( UsbSetupBuf->wIndexL ) {
				case 0x84: R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
				case 0x04: R8_UEP4_CTRL = (R8_UEP4_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
				case 0x83: R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
				case 0x03: R8_UEP3_CTRL = (R8_UEP3_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
				case 0x82: R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
				case 0x02: R8_UEP2_CTRL = (R8_UEP2_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
				case 0x81: R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_T_TOG | MASK_UEP_T_RES ))) | UEP_T_RES_NAK; break;
				case 0x01: R8_UEP1_CTRL = (R8_UEP1_CTRL & (~ ( RB_UEP_R_TOG | MASK_UEP_R_RES ))) | UEP_R_RES_ACK; break;
				default: len = 0xFF;  break;
			}
			}
			else len = 0xFF;
			break;
		}
		case USB_GET_INTERFACE:
			printf("GET_INTERFACE\r\n");
			Ep0Buffer[0] = 0x00;
			if ( SetupLen >= 1 ) len = 1;
			break;

		case USB_GET_STATUS:
			printf("GET_STATUS\r\n");
			Ep0Buffer[0] = 0x00;
			Ep0Buffer[1] = 0x00;
			if ( SetupLen >= 2 ) len = 2;
			else len = SetupLen;
			break;

		default:
		printf("IM HERE 999\n");
		len = 0xFF; break;
	}

	return len;
}

void USB_IRQProcessHandler( void ) {
	UINT8 len;
	UINT8   data_dir = 0;
	UINT8   i;

	{
		i = usb_irq_r_idx;

		if(usb_irq_flag[i]) {
		usb_irq_r_idx++;
		if(usb_irq_r_idx >= USB_IRQ_FLAG_NUM) usb_irq_r_idx = 0;

		switch ( usb_irq_pid[i] & 0x3f ) {
			case UIS_TOKEN_IN | 4:  Ep4DataINFlag = ~0; break;
			case UIS_TOKEN_IN | 3:  Ep3DataINFlag = ~0; break;
			case UIS_TOKEN_OUT | 2:
			{
				printf("usb_rec\n");
				len = usb_irq_len[i];
				for(int i = 0;i<len;i++)
				// printf("%02x  ",Ep2OUTDataBuf[i]);
				printf("\n");

				Ep2DataOUTFlag = 1;
				Ep2DataOUTLen = len;
				VENSer0SendFlag = 1;
				NVIC_DisableIRQ(USB_IRQn);
				R8_UEP2_CTRL = R8_UEP2_CTRL & 0xf3; //OUT_ACK
				NVIC_EnableIRQ(USB_IRQn);
				break;
			}
			case UIS_TOKEN_IN | 2:  Ep2DataINFlag = 1; break;
			case UIS_TOKEN_OUT | 1:
			{
				printf("usb_rec\n");
				len = usb_irq_len[i];
				//Ep1OUTDataBuf
				for(int i = 0;i<len;i++)
				// printf("%02x  ",Ep1OUTDataBuf[i]);
				printf("\n");

				Ep1DataOUTFlag = 1;
				Ep1DataOUTLen = len;
				VENSer0SendFlag = 1;
				NVIC_DisableIRQ(USB_IRQn);
				R8_UEP1_CTRL = R8_UEP1_CTRL & 0xf3; //OUT_ACK
				NVIC_EnableIRQ(USB_IRQn);
				break;
			}
			case UIS_TOKEN_IN | 1:  Ep1DataINFlag = 1; break;
			case UIS_TOKEN_SETUP | 0:    // endpoint 0# SETUP
			{
				len = usb_irq_len[i];

				if(len == sizeof(USB_SETUP_REQ)) {
				SetupLen = UsbSetupBuf->wLengthL;
				if(UsbSetupBuf->wLengthH) SetupLen = 0xff;
				len = 0;
				SetupReqCode = UsbSetupBuf->bRequest;

				data_dir = USB_REQ_TYP_OUT;
				if(UsbSetupBuf->bRequestType & USB_REQ_TYP_IN) data_dir = USB_REQ_TYP_IN;

				if( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_VENDOR ) {
					len = handle_USB_TYP_VENDOR(data_dir);
				}

				else if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD) {
					len = handle_USB_TYP_STANDARD(data_dir);
				}

				else if( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS ) {
					len = handle_USB_TYP_CLASS(data_dir);
				}
				
				else len = 0xFF;
				}
				else {
				len = 0xFF;
				}

				if ( len == 0xFF ) {
				printf("IM HERE 10\n");

				SetupReqCode = 0xFF;
				NVIC_DisableIRQ(USB_IRQn);
				R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;  // STALL
				NVIC_EnableIRQ(USB_IRQn);
				}
				else if ( len <= THIS_ENDP0_SIZE ) {
				if( SetupReqCode ==  USB_SET_ADDRESS) {
					// printf("add in:%d\r\n",len);
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_T_LEN = len;
					R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
					NVIC_EnableIRQ(USB_IRQn);
				}
				else if( SetupReqCode ==  USB_SET_CONFIGURATION)  {
					printf("IM HERE 11\n");
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_T_LEN = len;
					R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
					NVIC_EnableIRQ(USB_IRQn);
				}
				else if( SetupReqCode ==  USB_GET_DESCRIPTOR) {
					printf("IM HERE 12\n");
					R8_UEP0_T_LEN = len;
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
					NVIC_EnableIRQ(USB_IRQn);
				}
				else if( SetupReqCode ==  DEF_VEN_UART_INIT ) {
					printf("IM HERE 13\n");
					R8_UEP0_T_LEN = len;
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
					NVIC_EnableIRQ(USB_IRQn);
				}
				else if( SetupReqCode ==  DEF_VEN_DEBUG_WRITE ) {
					printf("IM HERE 14\n");
					R8_UEP0_T_LEN = len;
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
					NVIC_EnableIRQ(USB_IRQn);
				}
				else if( SetupReqCode ==  DEF_VEN_UART_M_OUT ) {
					printf("IM HERE 15\n");
					R8_UEP0_T_LEN = len;
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
					NVIC_EnableIRQ(USB_IRQn);
				}
				else if( SetupReqCode ==  DEF_SET_CONTROL_LINE_STATE ) {
					printf("IM HERE 16\n");
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_T_LEN = len;
					R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
					NVIC_EnableIRQ(USB_IRQn);
				}
				else if( SetupReqCode ==  USB_CLEAR_FEATURE ) {
					printf("IM HERE 17\n");
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_T_LEN = len;
					R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
					NVIC_EnableIRQ(USB_IRQn);
				}
				else {
					if(data_dir == USB_REQ_TYP_IN) {
					printf("IM HERE 18\n");
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_T_LEN = len;
					R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_ACK;
					NVIC_EnableIRQ(USB_IRQn);
					}
					else {
					printf("IM HERE 19\n");
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_T_LEN = len;
					R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
					NVIC_EnableIRQ(USB_IRQn);
					}
				}
				}
				else {
				printf("IM HERE 20\n");
				R8_UEP0_T_LEN = 0;
				NVIC_DisableIRQ(USB_IRQn);
				R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
				NVIC_EnableIRQ(USB_IRQn);
				}
				break;
			}
			case UIS_TOKEN_IN | 0:
			{
				switch( SetupReqCode ) {
				case USB_GET_DESCRIPTOR:  //0x06  获取描述符
					{
					printf("IM HERE 21\n");
					len = (SetupLen >= THIS_ENDP0_SIZE) ? THIS_ENDP0_SIZE : SetupLen;
					memcpy( Ep0Buffer, pDescr, len );
					SetupLen -= len;
					pDescr += len;

					if(len) {
						R8_UEP0_T_LEN = len;
						NVIC_DisableIRQ(USB_IRQn);
						R8_UEP0_CTRL ^=  RB_UEP_T_TOG;
						USBDevEPnINSetStatus(ENDP0, ENDP_TYPE_IN, IN_ACK);
						NVIC_EnableIRQ(USB_IRQn);
					}
					else {
						R8_UEP0_T_LEN = len;
						NVIC_DisableIRQ(USB_IRQn);
						R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
						NVIC_EnableIRQ(USB_IRQn);
					}
					break;
					}
				case USB_SET_ADDRESS:   //0x05
					{
					R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | (devinf.UsbAddress);
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
					NVIC_EnableIRQ(USB_IRQn);
					printf("add in deal\r\n");

					break;
					}
				case DEF_VEN_DEBUG_READ:     //0X95
				case DEF_VEN_GET_VER:         //0X5F
					{
					printf("IM HERE 23\n");
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
					NVIC_EnableIRQ(USB_IRQn);
					break;
					}
				case DEF_GET_LINE_CODING:  //0x21
					{
					printf("IM HERE 24\n");
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_ACK | UEP_T_RES_NAK;
					NVIC_EnableIRQ(USB_IRQn);
					break;
					}
				case DEF_SET_LINE_CODING:   //0x20
					{
					printf("IM HERE 25\n");
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
					NVIC_EnableIRQ(USB_IRQn);
					break;
					}
				default:
					{
					printf("IM HERE 26\n");
					R8_UEP0_T_LEN = 0;
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
					NVIC_EnableIRQ(USB_IRQn);

					break;
					}
				}
				break;
			}
			case UIS_TOKEN_OUT | 0:      // endpoint 0# OUT
			{
				len = usb_irq_len[i];

				if(len) {
				if(usb_work_mode == USB_CDC_MODE) {
					switch(SetupReqCode) {
					case DEF_SET_LINE_CODING:
						{
						UINT32 set_bps;
						UINT8  data_bit, stop_bit, ver_bit, set_stop_bit;

						memcpy(&set_bps,Ep0Buffer,4);
						stop_bit = Ep0Buffer[4];
						ver_bit = Ep0Buffer[5];
						data_bit = Ep0Buffer[6];

						// printf("LINE_CODING %d %d %d %d %d\r\n",CDCSetSerIdx
						// 					,(int)set_bps ,data_bit ,stop_bit ,ver_bit);

						Uart0Para.BaudRate = set_bps;
						Uart0Para.StopBits = stop_bit;
						Uart0Para.ParityType = ver_bit;
						Uart0Para.DataBits = data_bit;
						CDCSer0ParaChange = 1;

						NVIC_DisableIRQ(USB_IRQn);
						R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK|UEP_T_RES_ACK;
						NVIC_EnableIRQ(USB_IRQn);
						break;
						}
					default:
						NVIC_DisableIRQ(USB_IRQn);
						R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
						NVIC_EnableIRQ(USB_IRQn);
						break;
					}
				}
				else {
					NVIC_DisableIRQ(USB_IRQn);
					R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
					NVIC_EnableIRQ(USB_IRQn);
				}
				}
				else {
				NVIC_DisableIRQ(USB_IRQn);
				R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK|UEP_T_RES_NAK;
				NVIC_EnableIRQ(USB_IRQn);
				}
				break;
			}
			default:
			break;
		}

		NVIC_DisableIRQ(USB_IRQn);
		usb_irq_flag[i] = 0;
		NVIC_EnableIRQ(USB_IRQn);
		}
	}

	if ( R8_USB_INT_FG & RB_UIF_BUS_RST ) {
		R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;
		R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

		if(usb_work_mode != USB_VENDOR_MODE) {
		R8_UEP3_CTRL = UEP_T_RES_NAK;
		R8_UEP4_CTRL = UEP_T_RES_NAK;
		}

		cdc_uart_sta_trans_step = 0;
		ven_ep1_trans_step = 0;

		R8_USB_DEV_AD = 0x00;
		devinf.UsbAddress = 0;

		R8_USB_INT_FG = RB_UIF_BUS_RST;
	}
	else if (  R8_USB_INT_FG & RB_UIF_SUSPEND ) {
		Ep1DataINFlag = 1;
		Ep2DataINFlag = 1;
		Ep3DataINFlag = 1;
		Ep4DataINFlag = 1;

		Ep1DataOUTFlag = 0;
		Ep2DataOUTFlag = 0;
		Ep3DataOUTFlag = 0;
		Ep4DataOUTFlag = 0;

		if ( R8_USB_MIS_ST & RB_UMS_SUSPEND ) {
		if(usb_work_mode == USB_VENDOR_MODE) {
			VENSer0ParaChange = 1;
		}
		else {
			CDCSer0ParaChange = 1;
		}

		Ep1DataINFlag = 0;
		Ep2DataINFlag = 0;
		Ep3DataINFlag = 0;
		Ep4DataINFlag = 0;
		}

		cdc_uart_sta_trans_step = 0;
		ven_ep1_trans_step = 0;

		R8_USB_INT_FG = RB_UIF_SUSPEND;
	}
}


void InitCDCDevice(void){
	USBParaInit();

	R8_USB_CTRL = 0x00;

	R8_UEP4_1_MOD = RB_UEP4_TX_EN|RB_UEP1_TX_EN|RB_UEP1_RX_EN;
	R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_TX_EN;

	R16_UEP0_DMA = (u16)(u32)&Ep0Buffer[0];
	R16_UEP1_DMA = (u16)(u32)&Ep1Buffer[0];
	R16_UEP2_DMA = (u16)(u32)&Ep2Buffer[0];
	R16_UEP3_DMA = (u16)(u32)&Ep3Buffer[0];
	//R16_UEP4_DMA = (u16)(u32)&Ep2Buffer[0];

	R8_UEP0_CTRL = UEP_R_RES_NAK | UEP_T_RES_NAK;

	R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

	R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

	R8_UEP3_CTRL = UEP_T_RES_NAK;

	R8_UEP4_CTRL = UEP_T_RES_NAK;

	R8_USB_DEV_AD = 0x00;

	R8_UDEV_CTRL = RB_UD_PD_DIS;

	R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;

	R8_USB_INT_FG = 0xFF;

	R8_USB_INT_EN = RB_UIE_TRANSFER ;
	NVIC_EnableIRQ(USB_IRQn);

	R8_UDEV_CTRL |= RB_UD_PORT_EN;

	devinf.UsbConfig = 0;
	devinf.UsbAddress = 0;
}

u8 UART0_RI_Val = 0;
u8 UART0_RI_Change = 0;
u8 UART0_DSR_Val = 0;
u8 UART0_DSR_Change = 0;
u8 UART0_CTS_Val = 0;
u8 UART0_CTS_Change = 0;

u8 UART0_DCD_Val = 0;

void InitUSBDevPara(void) {
	u8 i;

	Uart0Para.BaudRate = 115200;
	Uart0Para.DataBits = HAL_UART_8_BITS_PER_CHAR;
	Uart0Para.ParityType = HAL_UART_NO_PARITY;
	Uart0Para.StopBits = HAL_UART_ONE_STOP_BIT;

	VENSer0ParaChange = 0;
	VENSer0SendFlag = 0;
	CDCSetSerIdx = 0;
	CDCSer0ParaChange = 0;

	for(i=0; i<CH341_REG_NUM; i++)
	{
		CH341_Reg_Add[i] = 0xff;
		CH341_Reg_val[i] = 0x00;
	}

	UART0_DCD_Val = 0;
	UART0_RI_Val = 0;
	UART0_DSR_Val = 0;
	UART0_CTS_Val = 0;

	UART0_RTS_Val = 0;
	UART0_DTR_Val = 0;
	UART0_OUT_Val = 0;

	for(i=0; i<USB_IRQ_FLAG_NUM; i++) {
		usb_irq_flag[i] = 0;
	}
}

void ch5xx_setClock(u8 clock_source) {
	SYS_SAFE_ACCESS (
		R8_PLL_CONFIG &= ~(1<<5);
		R16_CLK_SYS_CFG =  (clock_source & 0x1f);
		asm volatile( "nop\nnop\nnop\nnop" );
	);
}

#define FUNCONF_UART_PRINTF_BAUD 115200

int main(void) {
	SystemInit();
    Delay_Ms(100);
    
	uart_init_ch5xx(TARGET_UART, FUNCONF_UART_PRINTF_BAUD);
    printf("CH583 USB CDC Demo Started\n");
    
	InitUSBDevPara();
    InitCDCDevice();
    
	NVIC_EnableIRQ( USB_IRQn );

    while(1) {
		USB_IRQProcessHandler();
    }
}
