#define LOG_TAG "Nand"
#include "../mtd/nand.h"

//#include "malloc.h"
#include "board.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//NAND FLASH ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/1/15
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved					  
//********************************************************************************
//����˵��
//V1.1 20160520
//1,����Ӳ��ECC֧��(������NAND_ECC_SECTOR_SIZE��СΪ��λ���ж�дʱ����)
//2,����NAND_Delay����,���ڵȴ�tADL/tWHR
//3,����NAND_WritePageConst����,������Ѱ����.
//V1.2 20160525
//1,ȥ��NAND_SEC_SIZE�궨�壬��NAND_ECC_SECTOR_SIZE���
//2,ȥ��nand_dev�ṹ�������secbufָ�룬�ò���
//V1.3 20160907
//1,����NAND_TADL_DELAY��,��������tADL���ӳ�ʱ��,�����޸�
//V1.4 20180321
//1,������H27U4G8F2EоƬ��֧��
//2,�޸�MEMSET/MEMHOLD/MEMHIZ����ʱ,��֧��H27U4G08F2E.
//V1.5 20180531
//1.����NAND_TWHR_DELAY/NAND_TRHW_DELAY/NAND_TPROG_DELAY/NAND_TBERS_DELAY�ĸ���ʱ,��ֹ����
//2.����ʱ����ms�������ʱ����ֹ����ʱ�䲻��������ж�״̬����ɳ��� 
////////////////////////////////////////////////////////////////////////////////// 	
#include "stm32f4xx_ll_fsmc.h"
u8 ecc_check=1;
u8 NAND_ReadPageComp(u32 PageNum,u16 ColNum,u32 CmpVal,u16 NumByteToRead,u16 *NumByteEqual);
NAND_HandleTypeDef NAND_Handler;    //NAND FLASH���
nand_attriute nand_dev;             //nand��Ҫ�����ṹ��
//NAND��ʱ
//һ��i++������Ҫ4ns
void NAND_Delay(u32 i)
{
  while(i>0)i--;
}
u32 NAND_ReadID1(void)
{
    NAND_IDTypeDef NAND_ID;

    HAL_NAND_Read_ID(&NAND_Handler,&NAND_ID);


    if ((NAND_ID.Maker_Id == 0xC2) && (NAND_ID.Device_Id == 0xDC)
           && (NAND_ID.Third_Id == 0x90) && (NAND_ID.Fourth_Id == 0x95))
     {

        LOG_I("NAND_found,size=4Gb\r\n");
     }else {
         LOG_I("NAND_Unidentifiable\r\n");

    }
     LOG_I("ID[%X,%X,%X,%X]\n",NAND_ID.Maker_Id,NAND_ID.Device_Id,NAND_ID.Third_Id,NAND_ID.Fourth_Id);
    return 0;
}

//��ʼ��NAND FLASH
u8 NAND_Init(void)
{
    FMC_NAND_PCC_TimingTypeDef ComSpaceTiming,AttSpaceTiming;
                                              
    NAND_Handler.Instance=FMC_NAND_DEVICE;
    NAND_Handler.Init.NandBank=FSMC_NAND_BANK2;                          //NAND����BANK3��
    NAND_Handler.Init.Waitfeature=FSMC_NAND_PCC_WAIT_FEATURE_DISABLE;//FMC_NAND_PCC_WAIT_FEATURE_DISABLE;    //�رյȴ�����
    NAND_Handler.Init.MemoryDataWidth=FSMC_NAND_PCC_MEM_BUS_WIDTH_8;//FMC_NAND_PCC_MEM_BUS_WIDTH_8;     //8λ���ݿ��
    NAND_Handler.Init.EccComputation=FSMC_NAND_ECC_DISABLE;              //��ʹ��ECC
    NAND_Handler.Init.ECCPageSize=FSMC_NAND_ECC_PAGE_SIZE_2048BYTE;//FMC_NAND_ECC_PAGE_SIZE_2048BYTE;      //ECCҳ��СΪ2k
    NAND_Handler.Init.TCLRSetupTime=0;                                  //����TCLR(tCLR=CLE��RE����ʱ)=(TCLR+TSET+2)*THCLK,THCLK=1/180M=5.5ns
    NAND_Handler.Init.TARSetupTime=1;                                   //����TAR(tAR=ALE��RE����ʱ)=(TAR+TSET+2)*THCLK,THCLK=1/180M=5.5n��   
   
    ComSpaceTiming.SetupTime=2;         //����ʱ��
    ComSpaceTiming.WaitSetupTime=3;     //�ȴ�ʱ��
    ComSpaceTiming.HoldSetupTime=2;     //����ʱ��
    ComSpaceTiming.HiZSetupTime=1;      //����̬ʱ��
    
    AttSpaceTiming.SetupTime=2;         //����ʱ��
    AttSpaceTiming.WaitSetupTime=3;     //�ȴ�ʱ��
    AttSpaceTiming.HoldSetupTime=2;     //����ʱ��
    AttSpaceTiming.HiZSetupTime=1;      //����̬ʱ��
    
    HAL_NAND_Init(&NAND_Handler,&ComSpaceTiming,&AttSpaceTiming); 
    if(NAND_Reset())
        LOG_I("NAND_RESET ERR");       		        //��λNAND
    stm32_udelay(100*1000);
//    delay_ms(100);
//    NAND_ReadID1();
    nand_dev.id=NAND_ReadID();	        //��ȡID
	NAND_ModeSet(4);			        //����ΪMODE4,����ģʽ 
    if(nand_dev.id==MT29F16G08ABABA)    //NANDΪMT29F16G08ABABA
    {
        nand_dev.page_totalsize=4320;  	//nandһ��page���ܴ�С������spare����     
        nand_dev.page_mainsize=4096;   	//nandһ��page����Ч��������С    
        nand_dev.page_sparesize=224;	//nandһ��page��spare����С
        nand_dev.block_pagenum=128;		//nandһ��block��������page��Ŀ
        nand_dev.plane_blocknum=2048;	//nandһ��plane��������block��Ŀ
        nand_dev.block_totalnum=4096;  	//nand����block��Ŀ  
    }
    else if((nand_dev.id==MT29F4G08ABADA)||(nand_dev.id==MX30LF4G))//NANDΪMT29F4G08ABADA
    {
        nand_dev.page_totalsize=2112;	//nandһ��page���ܴ�С������spare����
        nand_dev.page_mainsize=2048; 	//nandһ��page����Ч��������С
        nand_dev.page_sparesize=64;		//nandһ��page��spare����С
        nand_dev.block_pagenum=64;		//nandһ��block��������page��Ŀ
        nand_dev.plane_blocknum=2048;	//nandһ��plane��������block��Ŀ
        nand_dev.block_totalnum=4096; 	//nand����block��Ŀ
        LOG_I("nand_id:%x",nand_dev.id);
    }

  //      return 1;	//���󣬷���
    return 0;
}
//INIT_BOARD_EXPORT(NAND_Init);
////NAND FALSH�ײ�����,�������ã�ʱ��ʹ��
////�˺����ᱻHAL_NAND_Init()����
//void HAL_NAND_MspInit(NAND_HandleTypeDef *hnand)
//{
//    GPIO_InitTypeDef GPIO_Initure;
//
//    __HAL_RCC_FMC_CLK_ENABLE();             //ʹ��FMCʱ��
//    __HAL_RCC_GPIOD_CLK_ENABLE();           //ʹ��GPIODʱ��
//    __HAL_RCC_GPIOE_CLK_ENABLE();           //ʹ��GPIOEʱ��
//    __HAL_RCC_GPIOG_CLK_ENABLE();           //ʹ��GPIOGʱ��
//
//	//��ʼ��PD6 R/B����
//	GPIO_Initure.Pin=GPIO_PIN_6;
//    GPIO_Initure.Mode=GPIO_MODE_INPUT;          //����
//    GPIO_Initure.Pull=GPIO_PULLUP;    			//����
//    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //����
//    HAL_GPIO_Init(GPIOD,&GPIO_Initure);
//
//	//��ʼ��PG9 NCE3����
//    GPIO_Initure.Pin=GPIO_PIN_9;
//    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //����
//    GPIO_Initure.Pull=GPIO_NOPULL;    			//����
//    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //����
//	GPIO_Initure.Alternate=GPIO_AF12_FMC;       //����ΪFMC
//    HAL_GPIO_Init(GPIOG,&GPIO_Initure);
//
//    //��ʼ��PD0,1,4,5,11,12,14,15
//    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|\
//                     GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
//    GPIO_Initure.Pull=GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOD,&GPIO_Initure);
//
//    //��ʼ��PE7,8,9,10
//    GPIO_Initure.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
//    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
//}

//��ȡNAND FLASH��ID
//����ֵ:0,�ɹ�;
//    ����,ʧ��
u8 NAND_ModeSet(u8 mode)
{   
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_FEATURE;//����������������
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=0X01;		//��ַΪ0X01,����mode
 	*(vu8*)NAND_ADDRESS=mode;					//P1����,����mode
	*(vu8*)NAND_ADDRESS=0;
	*(vu8*)NAND_ADDRESS=0;
	*(vu8*)NAND_ADDRESS=0; 
    if(NAND_WaitForReady()==NSTA_READY)return 0;//�ɹ�
    else return 1;								//ʧ��
}

//��ȡNAND FLASH��ID
//��ͬ��NAND���в�ͬ��������Լ���ʹ�õ�NAND FALSH�����ֲ�����д����
//����ֵ:NAND FLASH��IDֵ
u32 NAND_ReadID(void)
{
    u8 deviceid[5]; 
    u32 id;  
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_READID; //���Ͷ�ȡID����
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=0X00;
	//IDһ����5���ֽ�
    deviceid[0]=*(vu8*)NAND_ADDRESS;      
    deviceid[1]=*(vu8*)NAND_ADDRESS;  
    deviceid[2]=*(vu8*)NAND_ADDRESS; 
    deviceid[3]=*(vu8*)NAND_ADDRESS; 
    deviceid[4]=*(vu8*)NAND_ADDRESS;  
    //þ���NAND FLASH��IDһ��5���ֽڣ�����Ϊ�˷�������ֻȡ4���ֽ����һ��32λ��IDֵ
    //����NAND FLASH�������ֲᣬֻҪ��þ���NAND FLASH����ôһ���ֽ�ID�ĵ�һ���ֽڶ���0X2C
    //�������ǾͿ����������0X2C��ֻȡ�������ֽڵ�IDֵ��
    id=((u32)deviceid[1])<<24|((u32)deviceid[2])<<16|((u32)deviceid[3])<<8|deviceid[4];
    return id;
}  
//��NAND״̬
//����ֵ:NAND״ֵ̬
//bit0:0,�ɹ�;1,����(���/����/READ)
//bit6:0,Busy;1,Ready
u8 NAND_ReadStatus(void)
{
    vu8 data=0; 
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_READSTA;//���Ͷ�״̬����
    NAND_Delay(NAND_TWHR_DELAY);	//�ȴ�tWHR,�ٶ�ȡ״̬�Ĵ���
 	data=*(vu8*)NAND_ADDRESS;			//��ȡ״ֵ̬
    return data;
}
//�ȴ�NAND׼����
//����ֵ:NSTA_TIMEOUT �ȴ���ʱ��
//      NSTA_READY    �Ѿ�׼����
u8 NAND_WaitForReady(void)
{
    u8 status=0;
    vu32 time=0; 
	while(1)						//�ȴ�ready
	{
		status=NAND_ReadStatus();	//��ȡ״ֵ̬
		if(status&NSTA_READY)break;
		time++;
		if(time>=0X1FFFFFFF)return NSTA_TIMEOUT;//��ʱ
	}  
    return NSTA_READY;//׼����
}  
//��λNAND
//����ֵ:0,�ɹ�;
//    ����,ʧ��
u8 NAND_Reset(void)
{ 
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_RESET;	//��λNAND
    if(NAND_WaitForReady()==NSTA_READY)return 0;//��λ�ɹ�
    else return 1;								//��λʧ��
} 
//�ȴ�RB�ź�Ϊĳ����ƽ
//rb:0,�ȴ�RB==0
//   1,�ȴ�RB==1
//����ֵ:0,�ɹ�
//       1,��ʱ
//u32 maxtesttime;
u8 NAND_WaitRB(vu8 rb)
{ u32 timeout;
    vu32 time=0;  
    if(0==rb)
        timeout=0xff;

    else
        timeout=0X1FFFFF;
	while(time<timeout)
	{
		time++;
		if(NAND_RB==rb)
		{

		/*    if (maxtesttime<time) {

                LOG_I("time%x rb:%x",time,rb);
                maxtesttime= MAX(maxtesttime,time);
            }*/

		    return 0;

	    }
	}
	if(1==rb)
	    LOG_I("WaitRB==1 timeout");
	if(0==rb)
	{
//	    LOG_I("WaitRB==0 timeout");
	}

	return 1;
}

//��ȡNAND Flash��ָ��ҳָ���е�����(main����spare��������ʹ�ô˺���)
//PageNum:Ҫ��ȡ��ҳ��ַ,��Χ:0~(block_pagenum*block_totalnum-1)
//ColNum:Ҫ��ȡ���п�ʼ��ַ(Ҳ����ҳ�ڵ�ַ),��Χ:0~(page_totalsize-1)
//*pBuffer:ָ�����ݴ洢��
//NumByteToRead:��ȡ�ֽ���(���ܿ�ҳ��)
//����ֵ:0,�ɹ� 
//    ����,�������
u8 NAND_ReadPage(u32 PageNum,u16 ColNum,u8 *pBuffer,u16 NumByteToRead)
{
    vu16 i=0;
	u8 res=0;
	u8 eccnum=0;		//��Ҫ�����ECC������ÿNAND_ECC_SECTOR_SIZE�ֽڼ���һ��ecc
	u8 eccstart=0;		//��һ��ECCֵ�����ĵ�ַ��Χ
	u8 errsta=0;
	u8 *p;

     *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_AREA_A;
    //���͵�ַ
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)ColNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(ColNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)PageNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(PageNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(PageNum>>16);

    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_AREA_TRUE1;
    //�������д����ǵȴ�R/B���ű�Ϊ�͵�ƽ����ʵ��Ҫ����ʱ���õģ��ȴ�NAND����R/B���š���Ϊ������ͨ��
    //��STM32��NWAIT����(NAND��R/B����)����Ϊ��ͨIO��������ͨ����ȡNWAIT���ŵĵ�ƽ���ж�NAND�Ƿ�׼��
    //�����ġ����Ҳ����ģ��ķ������������ٶȺܿ��ʱ���п���NAND��û���ü�����R/B��������ʾNAND��æ
    //��״̬��������ǾͶ�ȡ��R/B����,���ʱ��϶������ģ���ʵ��ȷʵ�ǻ����!���Ҳ���Խ���������
    //���뻻����ʱ����,ֻ������������Ϊ��Ч������û������ʱ������
	res=NAND_WaitRB(0);			//�ȴ�RB=0
//    if(res)
//       return NSTA_TIMEOUT;	//��ʱ�˳�

    //����2�д����������ж�NAND�Ƿ�׼���õ�
	res=NAND_WaitRB(1);			//�ȴ�RB=1

    if(res)return NSTA_TIMEOUT;	//��ʱ�˳�
	if(NumByteToRead%NAND_ECC_SECTOR_SIZE)//����NAND_ECC_SECTOR_SIZE����������������ECCУ��
	{ 
		//��ȡNAND FLASH�е�ֵ
		for(i=0;i<NumByteToRead;i++)
		{
			*(vu8*)pBuffer++ = *(vu8*)NAND_ADDRESS;
		}
	}else
	{
		eccnum=NumByteToRead/NAND_ECC_SECTOR_SIZE;			//�õ�ecc�������
		eccstart=ColNum/NAND_ECC_SECTOR_SIZE;
		p=pBuffer;
		for(res=0;res<eccnum;res++)
		{
			FSMC_Bank2_3->PCR2|=1<<6;						//ʹ��ECCУ��
			for(i=0;i<NAND_ECC_SECTOR_SIZE;i++)				//��ȡNAND_ECC_SECTOR_SIZE������
			{
				*(vu8*)pBuffer++ = *(vu8*)NAND_ADDRESS;
			}		
		//	while((FSMC_Bank2_3->SR2&(1<<6)));             //�ȴ�FIFO��
			while(!(FSMC_Bank2_3->SR2&(1<<6)));				//�ȴ�FIFO��
			nand_dev.ecc_hdbuf[res+eccstart]=FSMC_Bank2_3->ECCR2;//��ȡӲ��������ECCֵ
			FSMC_Bank2_3->PCR2&=~(1<<6);						//��ֹECCУ��
		} 
		i=nand_dev.page_mainsize+0X10+eccstart*4;			//��spare����0X10λ�ÿ�ʼ��ȡ֮ǰ�洢��eccֵ
		NAND_Delay(NAND_TRHW_DELAY);//�ȴ�tRHW 
		*(vu8*)(NAND_ADDRESS|NAND_CMD)=0X05;				//�����ָ��
		//���͵�ַ
		*(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)i;
		*(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(i>>8);
		*(vu8*)(NAND_ADDRESS|NAND_CMD)=0XE0;				//��ʼ������
		NAND_Delay(NAND_TWHR_DELAY);//�ȴ�tWHR 
		pBuffer=(u8*)&nand_dev.ecc_rdbuf[eccstart];
		for(i=0;i<4*eccnum;i++)								//��ȡ�����ECCֵ
		{
			*(vu8*)pBuffer++= *(vu8*)NAND_ADDRESS;
		}
if (ecc_check==1) {
                        for(i=0;i<eccnum;i++)								//����ECC
                        {
                            if(nand_dev.ecc_rdbuf[i+eccstart]!=nand_dev.ecc_hdbuf[i+eccstart])//�����,��ҪУ��
                            {
                //				printf("err hd,rd:0x%x,0x%x\r\n",nand_dev.ecc_hdbuf[i+eccstart],nand_dev.ecc_rdbuf[i+eccstart]);
                // 				printf("eccnum,eccstart:%d,%d\r\n",eccnum,eccstart);
                //				printf("PageNum,ColNum:%d,%d\r\n",PageNum,ColNum);
                                res=NAND_ECC_Correction(p+NAND_ECC_SECTOR_SIZE*i,nand_dev.ecc_rdbuf[i+eccstart],nand_dev.ecc_hdbuf[i+eccstart]);//ECCУ��
                                if(res)errsta=NSTA_ECC2BITERR;				//���2BIT������ECC����
                                else errsta=NSTA_ECC1BITERR;				//���1BIT ECC����
                            }
                        }
                   }

	}
    if(NAND_WaitForReady()!=NSTA_READY)errsta=NSTA_ERROR;	//ʧ��
    return errsta;	//�ɹ�   
} 
//��ȡNAND Flash��ָ��ҳָ���е�����(main����spare��������ʹ�ô˺���),���Ա�(FTL����ʱ��Ҫ)
//PageNum:Ҫ��ȡ��ҳ��ַ,��Χ:0~(block_pagenum*block_totalnum-1)
//ColNum:Ҫ��ȡ���п�ʼ��ַ(Ҳ����ҳ�ڵ�ַ),��Χ:0~(page_totalsize-1)
//CmpVal:Ҫ�Աȵ�ֵ,��u32Ϊ��λ
//NumByteToRead:��ȡ����(��4�ֽ�Ϊ��λ,���ܿ�ҳ��)
//NumByteEqual:�ӳ�ʼλ�ó�����CmpValֵ��ͬ�����ݸ���
//����ֵ:0,�ɹ�
//    ����,�������
extern u8 check_erase;

u8 NAND_ReadPageComp(u32 PageNum,u16 ColNum,u32 CmpVal,u16 NumByteToRead,u16 *NumByteEqual)
{

    u32 data;
#ifdef NAND_PAGE_COMPARE_DUMP
    u32 *test=rt_malloc(NumByteToRead*4);
#endif
u32 err_flag=0;
    u16 i=0;
	u8 res=0;

retry:
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_AREA_A;
    //���͵�ַ
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)ColNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(ColNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)PageNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(PageNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(PageNum>>16);
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_AREA_TRUE1;
    //�������д����ǵȴ�R/B���ű�Ϊ�͵�ƽ����ʵ��Ҫ����ʱ���õģ��ȴ�NAND����R/B���š���Ϊ������ͨ��
    //��STM32��NWAIT����(NAND��R/B����)����Ϊ��ͨIO��������ͨ����ȡNWAIT���ŵĵ�ƽ���ж�NAND�Ƿ�׼��
    //�����ġ����Ҳ����ģ��ķ������������ٶȺܿ��ʱ���п���NAND��û���ü�����R/B��������ʾNAND��æ
    //��״̬��������ǾͶ�ȡ��R/B����,���ʱ��϶������ģ���ʵ��ȷʵ�ǻ����!���Ҳ���Խ���������
    //���뻻����ʱ����,ֻ������������Ϊ��Ч������û������ʱ������
	res=NAND_WaitRB(0);			//�ȴ�RB=0 
//	if(res)return NSTA_TIMEOUT;	//��ʱ�˳�
    //����2�д����������ж�NAND�Ƿ�׼���õ�
	res=NAND_WaitRB(1);			//�ȴ�RB=1 
    if(res)return NSTA_TIMEOUT;	//��ʱ�˳�  
    for(i=0;i<NumByteToRead;i++)//��ȡ����,ÿ�ζ�4�ֽ�
    {
        data=*(vu32*)NAND_ADDRESS;
#ifdef NAND_PAGE_COMPARE_DUMP
        *(test+i)=data;
#endif
		if(data!=CmpVal)
		{
#ifdef NAND_PAGE_COMPARE_DUMP
		   if (err_flag++>0) {

           }
		   if (err_flag==1)
		   {
		    goto retry;
		   }
#else
	     //   LOG_I("PageCompErr PageNum %d, ColNum %d, NumByteToWrite %d",PageNum,ColNum,NumByteToRead*4);
		       break;
#endif


		}
    }

        	*NumByteEqual=i;					//��CmpValֵ��ͬ�ĸ���

#ifdef NAND_PAGE_COMPARE_DUMP
        //	if (i<NumByteToRead)

        	if (err_flag)
        	{
             //LOG_HEX("read nand",16,test,(i+1)*4);
        	    LOG_HEX("read nand",16,test, NumByteToRead*4);
        	    LOG_I("PageNum %d, ColNum %d, NumByteToWrite %d",PageNum,ColNum,NumByteToRead*4);
        	    *NumByteEqual=0;
            }

	rt_free(test);
#endif

    if(NAND_WaitForReady()!=NSTA_READY)
        return NSTA_ERROR;//ʧ��
    return 0;	//�ɹ�   
} 
//��NANDһҳ��д��ָ�����ֽڵ�����(main����spare��������ʹ�ô˺���)
//PageNum:Ҫд���ҳ��ַ,��Χ:0~(block_pagenum*block_totalnum-1)
//ColNum:Ҫд����п�ʼ��ַ(Ҳ����ҳ�ڵ�ַ),��Χ:0~(page_totalsize-1)
//pBbuffer:ָ�����ݴ洢��
//NumByteToWrite:Ҫд����ֽ�������ֵ���ܳ�����ҳʣ���ֽ���������
//����ֵ:0,�ɹ� 
//    ����,�������
u8 NAND_WritePage(u32 PageNum,u16 ColNum,u8 *pBuffer,u16 NumByteToWrite)
{
    vu16 i=0;  
	u8 res=0;
	u8 eccnum=0;		//��Ҫ�����ECC������ÿNAND_ECC_SECTOR_SIZE�ֽڼ���һ��ecc
	u8 eccstart=0;		//��һ��ECCֵ�����ĵ�ַ��Χ
	
	*(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_WRITE0;
    //���͵�ַ
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)ColNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(ColNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)PageNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(PageNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(PageNum>>16);
	NAND_Delay(NAND_TADL_DELAY);//�ȴ�tADL 
	if(NumByteToWrite%NAND_ECC_SECTOR_SIZE)//����NAND_ECC_SECTOR_SIZE����������������ECCУ��
//	if(
//	        ((ColNum+NumByteToWrite)%NAND_ECC_SECTOR_SIZE)||
//	        (NumByteToWrite%NAND_ECC_SECTOR_SIZE)
//	   )//����NAND_ECC_SECTOR_SIZE����������������ECCУ��
	{
		for(i=0;i<NumByteToWrite;i++)		//д������
		{
			*(vu8*)NAND_ADDRESS=*(vu8*)pBuffer++;
		}
	}else
	{
		eccnum=NumByteToWrite/NAND_ECC_SECTOR_SIZE;			//�õ�ecc�������
		eccstart=ColNum/NAND_ECC_SECTOR_SIZE; 
 		for(res=0;res<eccnum;res++)
		{
			FSMC_Bank2_3->PCR2|=1<<6;						//ʹ��ECCУ��
			for(i=0;i<NAND_ECC_SECTOR_SIZE;i++)				//д��NAND_ECC_SECTOR_SIZE������
			{
				*(vu8*)NAND_ADDRESS=*(vu8*)pBuffer++;
			}		
			while(!(FSMC_Bank2_3->SR2&(1<<6)));				//�ȴ�FIFO��
			nand_dev.ecc_hdbuf[res+eccstart]=FSMC_Bank2_3->ECCR2;	//��ȡӲ��������ECCֵ
  			FSMC_Bank2_3->PCR2&=~(1<<6);						//��ֹECCУ��
		}  
		i=nand_dev.page_mainsize+0X10+eccstart*4;			//����д��ECC��spare����ַ
		NAND_Delay(NAND_TADL_DELAY);//�ȴ�tADL 
		*(vu8*)(NAND_ADDRESS|NAND_CMD)=0X85;				//���дָ��
		//���͵�ַ
		*(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)i;
		*(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(i>>8);
		NAND_Delay(NAND_TADL_DELAY);//�ȴ�tADL 
		pBuffer=(u8*)&nand_dev.ecc_hdbuf[eccstart];
		for(i=0;i<eccnum;i++)					//д��ECC
		{ 
			for(res=0;res<4;res++)				 
			{
				*(vu8*)NAND_ADDRESS=*(vu8*)pBuffer++;
			}
		} 		
	}

    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_WRITE_TURE1; 

#ifdef PAGE_WRITE_DIS
    if(ColNum<2048)
 	LOG_I("PageNum %d, ColNum %d, NumByteToWrite %d",PageNum,ColNum, NumByteToWrite);

    #else

#endif
 	delay_us(NAND_TPROG_DELAY);	//�ȴ�tPROG
	if(NAND_WaitForReady()!=NSTA_READY)return NSTA_ERROR;//ʧ��
    return 0;//�ɹ�   
}
//��NANDһҳ�е�ָ����ַ��ʼ,д��ָ�����ȵĺ㶨����
//PageNum:Ҫд���ҳ��ַ,��Χ:0~(block_pagenum*block_totalnum-1)
//ColNum:Ҫд����п�ʼ��ַ(Ҳ����ҳ�ڵ�ַ),��Χ:0~(page_totalsize-1)
//cval:Ҫд���ָ������
//NumByteToWrite:Ҫд�������(��4�ֽ�Ϊ��λ)
//����ֵ:0,�ɹ� 
//    ����,�������
u8 NAND_WritePageConst(u32 PageNum,u16 ColNum,u32 cval,u16 NumByteToWrite)
{
    u16 i=0;  
	*(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_WRITE0;
    //���͵�ַ
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)ColNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(ColNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)PageNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(PageNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(PageNum>>16);
		NAND_Delay(NAND_TADL_DELAY);//�ȴ�tADL 
	for(i=0;i<NumByteToWrite;i++)		//д������,ÿ��д4�ֽ�
	{
		*(vu32*)NAND_ADDRESS=cval;
	} 
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_WRITE_TURE1; 
 	delay_us(NAND_TPROG_DELAY);	//�ȴ�tPROG
    if(NAND_WaitForReady()!=NSTA_READY)return NSTA_ERROR;//ʧ��
    return 0;//�ɹ�   
}
//��һҳ���ݿ�������һҳ,��д��������
//ע��:Դҳ��Ŀ��ҳҪ��ͬһ��Plane�ڣ�
//Source_PageNo:Դҳ��ַ,��Χ:0~(block_pagenum*block_totalnum-1)
//Dest_PageNo:Ŀ��ҳ��ַ,��Χ:0~(block_pagenum*block_totalnum-1)  
//����ֵ:0,�ɹ�
//    ����,�������
u8 NAND_CopyPageWithoutWrite(u32 Source_PageNum,u32 Dest_PageNum)
{
	u8 res=0;
    u16 source_block=0,dest_block=0;  
    //�ж�Դҳ��Ŀ��ҳ�Ƿ���ͬһ��plane��
    source_block=Source_PageNum/nand_dev.block_pagenum;
    dest_block=Dest_PageNum/nand_dev.block_pagenum;
    if((source_block%2)!=(dest_block%2))return NSTA_ERROR;	//����ͬһ��plane�� 
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_MOVEDATA_CMD0;	//��������0X00
    //����Դҳ��ַ
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)0;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)0;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)Source_PageNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(Source_PageNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(Source_PageNum>>16);
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_MOVEDATA_CMD1;//��������0X35 
    //�������д����ǵȴ�R/B���ű�Ϊ�͵�ƽ����ʵ��Ҫ����ʱ���õģ��ȴ�NAND����R/B���š���Ϊ������ͨ��
    //��STM32��NWAIT����(NAND��R/B����)����Ϊ��ͨIO��������ͨ����ȡNWAIT���ŵĵ�ƽ���ж�NAND�Ƿ�׼��
    //�����ġ����Ҳ����ģ��ķ������������ٶȺܿ��ʱ���п���NAND��û���ü�����R/B��������ʾNAND��æ
    //��״̬��������ǾͶ�ȡ��R/B����,���ʱ��϶������ģ���ʵ��ȷʵ�ǻ����!���Ҳ���Խ���������
    //���뻻����ʱ����,ֻ������������Ϊ��Ч������û������ʱ������
	res=NAND_WaitRB(0);			//�ȴ�RB=0 
	//if(res)return NSTA_TIMEOUT;	//��ʱ�˳�
    //����2�д����������ж�NAND�Ƿ�׼���õ�
	res=NAND_WaitRB(1);			//�ȴ�RB=1 
    if(res)return NSTA_TIMEOUT;	//��ʱ�˳� 
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_MOVEDATA_CMD2;  //��������0X85
    //����Ŀ��ҳ��ַ
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)0;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)0;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)Dest_PageNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(Dest_PageNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(Dest_PageNum>>16);
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_MOVEDATA_CMD3;	//��������0X10 
	delay_us(NAND_TPROG_DELAY);	//�ȴ�tPROG
    if(NAND_WaitForReady()!=NSTA_READY)return NSTA_ERROR;	//NANDδ׼���� 
    return 0;//�ɹ�   
}

//��һҳ���ݿ�������һҳ,���ҿ���д������
//ע��:Դҳ��Ŀ��ҳҪ��ͬһ��Plane�ڣ�
//Source_PageNo:Դҳ��ַ,��Χ:0~(block_pagenum*block_totalnum-1)
//Dest_PageNo:Ŀ��ҳ��ַ,��Χ:0~(block_pagenum*block_totalnum-1)  
//ColNo:ҳ���е�ַ,��Χ:0~(page_totalsize-1)
//pBuffer:Ҫд�������
//NumByteToWrite:Ҫд������ݸ���
//����ֵ:0,�ɹ� 
//    ����,�������
u8 NAND_CopyPageWithWrite(u32 Source_PageNum,u32 Dest_PageNum,u16 ColNum,u8 *pBuffer,u16 NumByteToWrite)
{
	u8 res=0;
    vu16 i=0;
	u16 source_block=0,dest_block=0;  
	u8 eccnum=0;		//��Ҫ�����ECC������ÿNAND_ECC_SECTOR_SIZE�ֽڼ���һ��ecc
	u8 eccstart=0;		//��һ��ECCֵ�����ĵ�ַ��Χ
    //�ж�Դҳ��Ŀ��ҳ�Ƿ���ͬһ��plane��
    source_block=Source_PageNum/nand_dev.block_pagenum;
    dest_block=Dest_PageNum/nand_dev.block_pagenum;
    if((source_block%2)!=(dest_block%2))return NSTA_ERROR;//����ͬһ��plane��
	*(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_MOVEDATA_CMD0;  //��������0X00
    //����Դҳ��ַ
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)0;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)0;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)Source_PageNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(Source_PageNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(Source_PageNum>>16);
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_MOVEDATA_CMD1;  //��������0X35
    
    //�������д����ǵȴ�R/B���ű�Ϊ�͵�ƽ����ʵ��Ҫ����ʱ���õģ��ȴ�NAND����R/B���š���Ϊ������ͨ��
    //��STM32��NWAIT����(NAND��R/B����)����Ϊ��ͨIO��������ͨ����ȡNWAIT���ŵĵ�ƽ���ж�NAND�Ƿ�׼��
    //�����ġ����Ҳ����ģ��ķ������������ٶȺܿ��ʱ���п���NAND��û���ü�����R/B��������ʾNAND��æ
    //��״̬��������ǾͶ�ȡ��R/B����,���ʱ��϶������ģ���ʵ��ȷʵ�ǻ����!���Ҳ���Խ���������
    //���뻻����ʱ����,ֻ������������Ϊ��Ч������û������ʱ������
	res=NAND_WaitRB(0);			//�ȴ�RB=0 
//	if(res)return NSTA_TIMEOUT;	//��ʱ�˳�
    //����2�д����������ж�NAND�Ƿ�׼���õ�
	res=NAND_WaitRB(1);			//�ȴ�RB=1 
    if(res)return NSTA_TIMEOUT;	//��ʱ�˳� 
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_MOVEDATA_CMD2;  //��������0X85
    //����Ŀ��ҳ��ַ
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)ColNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(ColNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)Dest_PageNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(Dest_PageNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(Dest_PageNum>>16); 
    //����ҳ���е�ַ
	NAND_Delay(NAND_TADL_DELAY);//�ȴ�tADL 
	if(NumByteToWrite%NAND_ECC_SECTOR_SIZE)//����NAND_ECC_SECTOR_SIZE����������������ECCУ��
	{  
		for(i=0;i<NumByteToWrite;i++)		//д������
		{
			*(vu8*)NAND_ADDRESS=*(vu8*)pBuffer++;
		}
	}else
	{
		eccnum=NumByteToWrite/NAND_ECC_SECTOR_SIZE;			//�õ�ecc�������
		eccstart=ColNum/NAND_ECC_SECTOR_SIZE; 
 		for(res=0;res<eccnum;res++)
		{
			FSMC_Bank2_3->PCR2|=1<<6;						//ʹ��ECCУ��
			for(i=0;i<NAND_ECC_SECTOR_SIZE;i++)				//д��NAND_ECC_SECTOR_SIZE������
			{
				*(vu8*)NAND_ADDRESS=*(vu8*)pBuffer++;
			}		
			while(!(FSMC_Bank2_3->SR2&(1<<6)));				//�ȴ�FIFO��
			nand_dev.ecc_hdbuf[res+eccstart]=FSMC_Bank2_3->ECCR2;	//��ȡӲ��������ECCֵ
 			FSMC_Bank2_3->PCR2&=~(1<<6);						//��ֹECCУ��
		}  
		i=nand_dev.page_mainsize+0X10+eccstart*4;			//����д��ECC��spare����ַ
		NAND_Delay(NAND_TADL_DELAY);//�ȴ�tADL 
		*(vu8*)(NAND_ADDRESS|NAND_CMD)=0X85;				//���дָ��
		//���͵�ַ
		*(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)i;
		*(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(i>>8);
		NAND_Delay(NAND_TADL_DELAY);//�ȴ�tADL 
		pBuffer=(u8*)&nand_dev.ecc_hdbuf[eccstart];
		for(i=0;i<eccnum;i++)					//д��ECC
		{ 
			for(res=0;res<4;res++)				 
			{
				*(vu8*)NAND_ADDRESS=*(vu8*)pBuffer++;
			}
		} 		
	}
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_MOVEDATA_CMD3;	//��������0X10 
 	delay_us(NAND_TPROG_DELAY);							//�ȴ�tPROG
    if(NAND_WaitForReady()!=NSTA_READY)return NSTA_ERROR;	//ʧ��
    return 0;	//�ɹ�   
} 
//��ȡspare���е�����
//PageNum:Ҫд���ҳ��ַ,��Χ:0~(block_pagenum*block_totalnum-1)
//ColNum:Ҫд���spare����ַ(spare�����ĸ���ַ),��Χ:0~(page_sparesize-1) 
//pBuffer:�������ݻ����� 
//NumByteToRead:Ҫ��ȡ���ֽ���(������page_sparesize)
//����ֵ:0,�ɹ�
//    ����,�������
u8 NAND_ReadSpare(u32 PageNum,u16 ColNum,u8 *pBuffer,u16 NumByteToRead)
{
    u8 temp=0;
    u8 remainbyte=0;
    remainbyte=nand_dev.page_sparesize-ColNum;
    if(NumByteToRead>remainbyte) NumByteToRead=remainbyte;  //ȷ��Ҫд����ֽ���������spareʣ��Ĵ�С
    temp=NAND_ReadPage(PageNum,ColNum+nand_dev.page_mainsize,pBuffer,NumByteToRead);//��ȡ����
    return temp;
} 
//��spare����д����
//PageNum:Ҫд���ҳ��ַ,��Χ:0~(block_pagenum*block_totalnum-1)
//ColNum:Ҫд���spare����ַ(spare�����ĸ���ַ),��Χ:0~(page_sparesize-1)  
//pBuffer:Ҫд��������׵�ַ 
//NumByteToWrite:Ҫд����ֽ���(������page_sparesize)
//����ֵ:0,�ɹ�
//    ����,ʧ��
u8 NAND_WriteSpare(u32 PageNum,u16 ColNum,u8 *pBuffer,u16 NumByteToWrite)
{
    u8 temp=0;
    u8 remainbyte=0;
    remainbyte=nand_dev.page_sparesize-ColNum;
    if(NumByteToWrite>remainbyte) NumByteToWrite=remainbyte;  //ȷ��Ҫ��ȡ���ֽ���������spareʣ��Ĵ�С
    temp=NAND_WritePage(PageNum,ColNum+nand_dev.page_mainsize,pBuffer,NumByteToWrite);//��ȡ
    return temp;
} 




//����һ����
//BlockNum:Ҫ������BLOCK���,��Χ:0-(block_totalnum-1)
//����ֵ:0,�����ɹ�
//    ����,����ʧ��
u8 NAND_EraseBlock(u32 BlockNum)
{
	if(nand_dev.id==MT29F16G08ABABA)
	    BlockNum<<=7;  	//�����ַת��Ϊҳ��ַ
    else if((nand_dev.id==MT29F4G08ABADA)||(nand_dev.id==MX30LF4G))
        BlockNum<<=6;//64
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_ERASE0;
    //���Ϳ��ַ
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)BlockNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>16);
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_ERASE1;
    //rt_thread_mdelay(1000);
    rt_thread_mdelay(NAND_TBERS_DELAY);		//�ȴ������ɹ�
	if(NAND_WaitForReady()!=NSTA_READY)
	    return NSTA_ERROR;//ʧ��
    return 0;	//�ɹ�   
} 
//ȫƬ����NAND FLASH
void NAND_EraseChip(void)
{
    u8 status;
    u16 i=0;
    for(i=0;i<nand_dev.block_totalnum;i++) //ѭ���������еĿ�
    {
        status=NAND_EraseBlock(i);
        if(status)printf("Erase %d block fail!!��������Ϊ%d\r\n",i,status);//����ʧ��
    }
}

//��ȡECC������λ/ż��λ
//oe:0,ż��λ
//   1,����λ
//eccval:�����eccֵ
//����ֵ:������eccֵ(���16λ)
u16 NAND_ECC_Get_OE(u8 oe,u32 eccval)
{
	u8 i;
	u16 ecctemp=0;
	for(i=0;i<24;i++)
	{
		if((i%2)==oe)
		{
			if((eccval>>i)&0X01)ecctemp+=1<<(i>>1); 
		}
	}
	return ecctemp;
} 
//ECCУ������
//eccrd:��ȡ����,ԭ�������ECCֵ
//ecccl:��ȡ����ʱ,Ӳ�������ECCֻ
//����ֵ:0,����������
//    ����,ECC����(�д���2��bit�Ĵ���,�޷��ָ�)
u8 NAND_ECC_Correction(u8* data_buf,u32 eccrd,u32 ecccl)
{
	u16 eccrdo,eccrde,eccclo,ecccle;
	u16 eccchk=0;
	u16 errorpos=0; 
	u32 bytepos=0;  
	eccrdo=NAND_ECC_Get_OE(1,eccrd);	//��ȡeccrd������λ
	eccrde=NAND_ECC_Get_OE(0,eccrd);	//��ȡeccrd��ż��λ
	eccclo=NAND_ECC_Get_OE(1,ecccl);	//��ȡecccl������λ
	ecccle=NAND_ECC_Get_OE(0,ecccl); 	//��ȡecccl��ż��λ
	eccchk=eccrdo^eccrde^eccclo^ecccle;
	if(eccchk==0XFFF)	//ȫ1,˵��ֻ��1bit ECC����
	{
		errorpos=eccrdo^eccclo; 
		printf("ECC error pos:%d\r\n",errorpos);
		bytepos=errorpos/8; 
		data_buf[bytepos]^=1<<(errorpos%8);
	}else				//����ȫ1,˵��������2bit ECC����,�޷��޸�
	{
//		printf("2bit ecc error or more\r\n");
	    printf("\r2bit ecc error or more\r\n");
		return 1;
	} 
	return 0;
}
 
void FSMC_NAND_Test_1(void)
{
  uint16_t index;
    uint16_t j;
    u8 onetick_len=128;
   u16 PageNum=0;
   u8 *pBuffer;//[2048];
   u8 *line_read;//[4096];
   u8 ret=0;
   u16 remainbyte=0;
   u16 ColNum=0;
   u16 NumByteToRead=2048;
   remainbyte=nand_dev.page_mainsize-ColNum;
   if(NumByteToRead>remainbyte) NumByteToRead=remainbyte;  //ȷ��Ҫд����ֽ���������spareʣ��Ĵ�С
   pBuffer=rt_malloc(nand_dev.page_mainsize*2);
   line_read=rt_malloc(nand_dev.page_mainsize*2);//[4096];
//  /* Erase the NAND first Block */
//  FSMC_NAND_EraseBlock(WriteReadAddr);
//
//  /* Write data to FSMC NOR memory */
//  /* Fill the buffer to send */
  for (index = 0; index < NumByteToRead; index++ )
  {
      pBuffer[index] = index;
  }
//
//  FSMC_NAND_WriteSmallPage(TxBuffer, WriteReadAddr, 1);
//   printf("\r\nWritten to the number of�� \r\n");
//   for(j = 0; j < 10; j++)
//    printf("%x  \r\n",TxBuffer[j]);
//  ret=NAND_EraseBlock(PageNum);
//  if(ret) //����ʧ��,��Ϊ����
//  {
//  printf("Bad block\r\n");
//  }
//   NAND_WritePage(PageNum,ColNum,pBuffer,NumByteToRead);




  /* Read back the written data */
  dev_fal_flash_Erase(e_nand_flash, (u8*)(pBuffer), 0, 128*1024);
//  for(j = 0; j < 4096; j+=onetick_len)
//  dev_fal_flash_Write(e_nand_flash, (u8*)(pBuffer+j), (u32)j, onetick_len);



j=4096;
  #if 1
        //д���У��line_read
        dev_fal_flash_Read(e_nand_flash, (u8*)line_read, (u32)0, j);
        LOG_HEX("",16,line_read,j);
//      if (memcmp((u8*)pBuffer,(u8*)line_read,j)) {
//          rt_kprintf("compare err");
//      }
  #endif





//   ret=NAND_ReadPage(PageNum,ColNum,pBuffer,NumByteToRead);//��ȡ����
     printf("\r\nRead several�� \r\n");
//    for(j = 0; j < 10; j++)
//    rt_kprintf("%x  \r\n",pBuffer[j]);
    rt_free(pBuffer);
    rt_free(line_read);
}





