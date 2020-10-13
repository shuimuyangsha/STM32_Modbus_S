/*
*
*	�����������
*
*/

#include"master.h"
#include"usart.h"
#include"modbus.h"
#include"delay.h"
#include"desk.h"

u8 FLAG_caiji;	   //����0ʱ���ɼ���ѹ������1ʱ,�ɼ�����
extern u8 index_qd;
extern u8 FLAG_comerr;

void construct_rtu_frm ( u8 *dst_buf,u8 *src_buf,u8 lenth)
{
    unsigned short  crc_tmp;
    crc_tmp = crc16(src_buf, lenth);	
    *(src_buf+lenth) = crc_tmp >> 8 ;
    *(src_buf+lenth+1) = crc_tmp & 0xff;
    lenth++;
    lenth++;
    while ( lenth--)
    {
       *dst_buf = *src_buf;
       dst_buf++;
       src_buf++;     
    }
}

u16  rtu_neizu_order ( u8 board_adr,u8 *com_buf,u16 start_address,u16 lenth) //��������ɼ�����//
{
    unsigned char tmp[32], tmp_lenth;  
    tmp[0] = board_adr;
	tmp[1] = NEIZU_CAIJI;
	tmp[2] = HI(start_address);
	tmp[3] = LOW(start_address);
	tmp[4] = HI(lenth);
	tmp[5] = LOW(lenth);   
    tmp_lenth = 6;
    construct_rtu_frm ( com_buf,tmp,tmp_lenth);
	sendCount2=8;
	beginSend2();
    return 8;
}

u16  rtu_read_hldreg ( u8 board_adr,u8 *com_buf,u16 start_address,u16 lenth) //03
{

    unsigned char tmp[32], tmp_lenth;   
    tmp[0] = board_adr;
    tmp[1] = READ_HLD_REG;
    tmp[2] = HI(start_address);
    tmp[3] = LOW(start_address);
    tmp[4] = HI(lenth);
    tmp[5] = LOW(lenth);
    tmp_lenth = 6;
    construct_rtu_frm ( com_buf,tmp,tmp_lenth);
	sendCount2=8;
	beginSend2();
    return 8;//Ϊ�˷�ֹ����������//
}

u16 rtu_set_hldreg( u8 board_adr,u8 *com_buf, u16 start_address, u16 value )//06
{
    unsigned char tmp[32], tmp_lenth;   
    tmp[0] = board_adr;
    tmp[1] = SET_HLD_REG;
    tmp[2] = HI(start_address);
    tmp[3] = LOW(start_address);
    tmp[4] = HI(value);
    tmp[5] = LOW(value);  
    tmp_lenth = 6;
    construct_rtu_frm ( com_buf, tmp, tmp_lenth);
	sendCount2=8;
	beginSend2();
    return 8 ;

}

int rtu_data_anlys( u16  *dest_p, u8 *source_p, u16 data_start_address, u16 fr_lenth)//rtu ���շ���//
{
    u16 crc_result, crc_tmp;
    u8 tmp1, tmp2, shift; 
		 
	crc_tmp = *(source_p + fr_lenth-2); // crc  ��һ�ֽ�//
 	crc_tmp = crc_tmp * 256 + *( source_p+fr_lenth-1); // CRC ֵ// 
	crc_result = crc16(source_p, fr_lenth-2); // ����CRC ֵ
//  if ( crc_tmp != crc_result ) // CRC У����ȷ//
	if ( crc_tmp == crc_result ) // CRC У����ȷ//
    {
// 		hld_reg[0x31]++;
// 		return -1;
//	}
		switch ( *(source_p+1) ) // ������//
		{
		   case READ_COIL:                   //��ȡ�̵���״̬//
		   for ( tmp1=0; tmp1<*( source_p+2); tmp1++)
		   {
				shift = 1;
				for ( tmp2=0; tmp2<8; tmp2++)
				{ 
					 *(dest_p+data_start_address+tmp1*8+tmp2) = shift & *( source_p+3);
					 *( source_p+3) >>= 1;          
				}
		   }
		   break;
		   case READ_DI: //��ȡ����������//
		   for ( tmp1=0; tmp1<*( source_p+2); tmp1++)
		   {
				shift = 1;
				for (tmp2=0; tmp2<8; tmp2 ++)
				{ 
					*(dest_p+data_start_address+tmp1*8+tmp2) = shift & *( source_p+3);
					*( source_p+3)>>=1;             
				}
		   }
		   break;
		   case READ_HLD_REG:  //��ȡ���ּĴ���//
		   for ( tmp1=0; tmp1<*( source_p+2); tmp1+=2)
		   {
				//relay17_he;
				*(dest_p + data_start_address+ tmp1/2)= *( source_p+tmp1+3)*256 +  *( source_p+tmp1+4) ;
				//relay17_he;        
		   }
		   break ;
		   case 4:      //��ȡģ��������//
		   for ( tmp1=0; tmp1<*( source_p+2); tmp1+=2)
		   {
				*(dest_p + data_start_address+ tmp1/2) = *( source_p+tmp1+3)*256 +  *( source_p+tmp1+4) ;      
		   }
		   break;
		   case PROTOCOL_EXCEPTION:
		   return -1*PROTOCOL_ERR;     //����ת���������ͱ仯�ļ���
		   //break;
		   default:
		   return -1*PROTOCOL_ERR;		//����ת���������ͱ仯�ļ���
		   // break;
		}
		receCount2=0;
	}
	return 0;	
}


void modbus_rtu_dy(void) //��ѹ��ȡ����
{
	if(modbus_com2_over==2&&receBuf2[0]==11)
	{
		modbus_com2_over=0;
		rtu_data_anlys(nzval,receBuf2,100,25);		
	}
	else if(modbus_com2_over==2&&receBuf2[0]+1==slaveraddr)
	{
		modbus_com2_over=0;
		rtu_data_anlys(adcval,receBuf2,regstartaddr-10,25);		
	}
	else if(modbus_com2_over!=2 || receBuf[0]+1 != slaveraddr)
	{
		FLAG_comerr=1;
		receCount2=0;
	}
	rtu_read_hldreg(slaveraddr,sendBuf2,0,10);
	slaveraddr++;
	regstartaddr=regstartaddr+10;
	if(slaveraddr>boardnum)
	{
		FLAG_caiji=1;
		slaveraddr=1;
		regstartaddr=0;
	}
}

void modbus_rtu_nz(void)	//�����ȡ����
{
	if(modbus_com2_over==2&&receBuf2[0]==11)
	{
		modbus_com2_over=0;
		rtu_data_anlys(adcval,receBuf2,100,25);		
	}
	else if(modbus_com2_over==2&&receBuf2[0]+1==slaveraddr)
	{
		modbus_com2_over=0;
		rtu_data_anlys(nzval,receBuf2,regstartaddr-10,25); 
	}
	else if(modbus_com2_over!=2 || receBuf[0]+1 != slaveraddr)
	{
		FLAG_comerr=1;
		receCount2=0;
	}
	rtu_read_hldreg(slaveraddr,sendBuf2,16,10);
	slaveraddr++;
	regstartaddr=regstartaddr+10;
	if(slaveraddr>boardnum)
	{
		FLAG_caiji=0;
		slaveraddr=1;
		regstartaddr=0;
	}
}

void modbus_rtu(void)  //���ݲɼ�����
{
	if(FLAG_caiji==0)
	{modbus_rtu_dy();}
	else
	{modbus_rtu_nz();}
}

















