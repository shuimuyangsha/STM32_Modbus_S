/*
*
*	主机服务程序
*
*/

#include"master.h"
#include"usart.h"
#include"modbus.h"
#include"delay.h"
#include"desk.h"

u8 FLAG_caiji;	   //等于0时，采集电压，等于1时,采集内阻
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

u16  rtu_neizu_order ( u8 board_adr,u8 *com_buf,u16 start_address,u16 lenth) //开启内阻采集命令//
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
    return 8;//为了防止编译器报错//
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

int rtu_data_anlys( u16  *dest_p, u8 *source_p, u16 data_start_address, u16 fr_lenth)//rtu 接收分析//
{
    u16 crc_result, crc_tmp;
    u8 tmp1, tmp2, shift; 
		 
	crc_tmp = *(source_p + fr_lenth-2); // crc  第一字节//
 	crc_tmp = crc_tmp * 256 + *( source_p+fr_lenth-1); // CRC 值// 
	crc_result = crc16(source_p, fr_lenth-2); // 计算CRC 值
//  if ( crc_tmp != crc_result ) // CRC 校验正确//
	if ( crc_tmp == crc_result ) // CRC 校验正确//
    {
// 		hld_reg[0x31]++;
// 		return -1;
//	}
		switch ( *(source_p+1) ) // 功能码//
		{
		   case READ_COIL:                   //读取继电器状态//
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
		   case READ_DI: //读取开关量输入//
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
		   case READ_HLD_REG:  //读取保持寄存器//
		   for ( tmp1=0; tmp1<*( source_p+2); tmp1+=2)
		   {
				//relay17_he;
				*(dest_p + data_start_address+ tmp1/2)= *( source_p+tmp1+3)*256 +  *( source_p+tmp1+4) ;
				//relay17_he;        
		   }
		   break ;
		   case 4:      //读取模拟量输入//
		   for ( tmp1=0; tmp1<*( source_p+2); tmp1+=2)
		   {
				*(dest_p + data_start_address+ tmp1/2) = *( source_p+tmp1+3)*256 +  *( source_p+tmp1+4) ;      
		   }
		   break;
		   case PROTOCOL_EXCEPTION:
		   return -1*PROTOCOL_ERR;     //整数转换导致类型变化的迹象
		   //break;
		   default:
		   return -1*PROTOCOL_ERR;		//整数转换导致类型变化的迹象
		   // break;
		}
		receCount2=0;
	}
	return 0;	
}


void modbus_rtu_dy(void) //电压读取命令
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

void modbus_rtu_nz(void)	//内阻读取命令
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

void modbus_rtu(void)  //数据采集命令
{
	if(FLAG_caiji==0)
	{modbus_rtu_dy();}
	else
	{modbus_rtu_nz();}
}

















