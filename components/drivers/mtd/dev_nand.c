/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-02     25292       the first version
 */

#include "nand.h"
#include "stm32f4xx_hal_nand.h"
//u32 NAND_CMD_AREA;
//u32 NAND_ADDR_AREA;
//u32 NAND_DATA_AREA;


#define NAND_CMD_AREA                  *(vu8*)(NAND_ADDRESS|NAND_CMD) /* A17 = CLE  high */
#define NAND_ADDR_AREA                  *(vu8*)(NAND_ADDRESS|NAND_ADDR)  /* A16 = ALE high */
#define NAND_DATA_AREA                  (*(u8*)NAND_ADDRESS)

NAND_HandleTypeDef NAND_Handler;



static u8 FSMC_NAND_ReadStatus(void)
{
    u8 ucData;
    u8 ucStatus = NAND_BUSY;

    NAND_CMD_AREA = NAND_CMD_STATUS;
    ucData = *(__IO u8 *)(NAND_DEVICE1);//NAND_ADDRESS

    if((ucData & NAND_ERROR) == NAND_ERROR)
    {
        ucStatus = NAND_ERROR;
    }
    else if((ucData & NAND_READY) == NAND_READY)
    {
        ucStatus = NAND_READY;
    }
    else
    {
        ucStatus = NAND_BUSY;
    }

    return (ucStatus);
}

static u8 FSMC_NAND_GetStatus(void)
{
    rt_uint32_t ulTimeout = 0x10000;
    u8 ucStatus = NAND_READY;

    ucStatus = FSMC_NAND_ReadStatus();

    while ((ucStatus != NAND_READY) &&( ulTimeout != 0x00))
    {
        ucStatus = FSMC_NAND_ReadStatus();
        if(ucStatus == NAND_ERROR)
        {
            return (ucStatus);
        }
        ulTimeout--;
    }

    if(ulTimeout == 0x00)
    {
        ucStatus =  NAND_TIMEOUT_ERROR;
    }

    return (ucStatus);
}


#define LOG_TAG              "NAND"
//��ȡNAND FLASH��ID
//��ͬ��NAND���в�ͬ��������Լ���ʹ�õ�NAND FALSH�����ֲ�����д����
//����ֵ:NAND FLASH��IDֵ
u32 NAND_ReadID(void)
{
    NAND_IDTypeDef NAND_ID;

    HAL_NAND_Read_ID(&NAND_Handler,&NAND_ID);


    if ((NAND_ID.Maker_Id == 0xC2) && (NAND_ID.Device_Id == 0xDC)
           && (NAND_ID.Third_Id == 0x90) && (NAND_ID.Fourth_Id == 0x95))
     {

        LOG_I("NAND_found,size=4Gb\r\n");
     }else {
         LOG_I("NAND_Unidentifiable\r\n");
         LOG_I("ID[%X,%X,%X,%X]\n",NAND_ID.Maker_Id,NAND_ID.Device_Id,NAND_ID.Third_Id,NAND_ID.Fourth_Id);
    }
    return 0;
}
INIT_APP_EXPORT(NAND_ReadID);
//��λNAND
//����ֵ:0,�ɹ�;
//    ����,ʧ��
//u8 NAND_Reset(void);
//{
//    NAND_CMD_AREA = NAND_RESET; //��λNAND
//    if(FSMC_NAND_GetStatus()==NAND_READY)
//        return 0;           //��λ�ɹ�
//    else
//        return 1;           //��λʧ��
//}


void rt_hw_mtd_nand_deinit(void)
{
    HAL_NAND_DeInit(&NAND_Handler);
}


//��ʼ��NAND FLASH
u8 rt_hw_mtd_nand_init(void)
{
    if(&NAND_Handler != NULL){
        rt_hw_mtd_nand_deinit();
    }
    FMC_NAND_PCC_TimingTypeDef ComSpaceTiming,AttSpaceTiming;

    NAND_Handler.Instance               = FMC_NAND_DEVICE;
    NAND_Handler.Init.NandBank          = FSMC_NAND_BANK2;                             //NAND����BANK2��
    NAND_Handler.Init.Waitfeature       = FSMC_NAND_PCC_WAIT_FEATURE_DISABLE;           //�رյȴ�����
    NAND_Handler.Init.MemoryDataWidth   = FSMC_NAND_PCC_MEM_BUS_WIDTH_8;                //8λ���ݿ��
    NAND_Handler.Init.EccComputation    = FSMC_NAND_ECC_ENABLE;                        //��ʹ��ECC
    NAND_Handler.Init.ECCPageSize       = FSMC_NAND_ECC_PAGE_SIZE_2048BYTE;             //ECCҳ��СΪ2k
    NAND_Handler.Init.TCLRSetupTime     = 1;                                            //����TCLR(tCLR=CLE��RE����ʱ)=(TCLR+TSET+2)*THCLK,THCLK=1/180M=5.5ns
    NAND_Handler.Init.TARSetupTime      = 1;                                            //����TAR(tAR=ALE��RE����ʱ)=(TAR+TSET+2)*THCLK,THCLK=1/180M=5.5n��

    ComSpaceTiming.SetupTime        = 2;        //����ʱ��
    ComSpaceTiming.WaitSetupTime    = 5;        //�ȴ�ʱ��
    ComSpaceTiming.HoldSetupTime    = 3;        //����ʱ��
    ComSpaceTiming.HiZSetupTime     = 1;        //����̬ʱ��

    AttSpaceTiming.SetupTime        = 2;        //����ʱ��
    AttSpaceTiming.WaitSetupTime    = 5;        //�ȴ�ʱ��
    AttSpaceTiming.HoldSetupTime    = 3;        //����ʱ��
    AttSpaceTiming.HiZSetupTime     = 1;        //����̬ʱ��

    HAL_NAND_Init(&NAND_Handler,&ComSpaceTiming,&AttSpaceTiming);

 //   NAND_Reset();                       //��λNAND
//    rt_thread_mdelay(100);

   // NAND_ReadID();
    return 0;
}

u8 FSMC_NAND_ReadPage(u8 *_pBuffer, rt_uint32_t _ulPageNo, u16 _usAddrInPage, u16 NumByteToRead)
{
    rt_uint32_t i;

    NAND_CMD_AREA = NAND_AREA_A;
    //���͵�ַ
    NAND_ADDR_AREA = _usAddrInPage;
    NAND_ADDR_AREA = _usAddrInPage >> 8;
    NAND_ADDR_AREA = _ulPageNo;
    NAND_ADDR_AREA = (_ulPageNo & 0xFF00) >> 8;
    NAND_ADDR_AREA = (_ulPageNo & 0xFF0000) >> 16;

    NAND_CMD_AREA = NAND_AREA_TRUE1;

     /* ����ȴ���������������쳣, �˴�Ӧ���жϳ�ʱ */
    for (i = 0; i < 20; i++);
    while(rt_pin_read(NAND_RB)==0);

    /* �����ݵ�������pBuffer */
    for(i = 0; i < NumByteToRead; i++)
    {
        _pBuffer[i] = NAND_DATA_AREA;
    }

    return RT_EOK;
}


u8 FSMC_NAND_WritePage(u8 *_pBuffer, rt_uint32_t _ulPageNo, u16 _usAddrInPage, u16 NumByteToRead)
{
    rt_uint32_t i;
  u8 ucStatus;

    NAND_CMD_AREA = NAND_WRITE0;
  //���͵�ַ
    NAND_ADDR_AREA = _usAddrInPage;
    NAND_ADDR_AREA = _usAddrInPage >> 8;
    NAND_ADDR_AREA = _ulPageNo;
    NAND_ADDR_AREA = (_ulPageNo & 0xFF00) >> 8;
    NAND_ADDR_AREA = (_ulPageNo & 0xFF0000) >> 16;
    for (i = 0; i < 20; i++);

    for(i = 0; i < NumByteToRead; i++)
    {
        NAND_DATA_AREA = _pBuffer[i];
    }

    NAND_CMD_AREA = NAND_WRITE_TURE1;

    for (i = 0; i < 20; i++);

    ucStatus = FSMC_NAND_GetStatus();
    if(ucStatus == NAND_READY)
    {
   //     ucStatus = RTV_NOERR;
    }
    else if(ucStatus == NAND_ERROR)
    {
      //  ucStatus = ERR_NAND_PROG;
    }
    else if(ucStatus == NAND_TIMEOUT_ERROR)
    {
   //     ucStatus = ERR_NAND_HW_TOUT;
    }

    return (ucStatus);
}

//����һ����
//BlockNum:Ҫ������BLOCK���,��Χ:0-(block_totalnum-1)
//����ֵ:0,�����ɹ�
//    ����,����ʧ��
u8 NAND_EraseBlock(rt_uint32_t _ulBlockNo)
{
    u8 ucStatus;

    NAND_CMD_AREA = NAND_ERASE0;

    _ulBlockNo <<= 6;

    NAND_ADDR_AREA = _ulBlockNo;
    NAND_ADDR_AREA = _ulBlockNo >> 8;
    NAND_ADDR_AREA = _ulBlockNo >> 16;

    NAND_CMD_AREA = NAND_ERASE1;

    ucStatus = FSMC_NAND_GetStatus();
    if(ucStatus == NAND_READY)
    {
    //    ucStatus = RTV_NOERR;
    }
    else if(ucStatus == NAND_ERROR)
    {
      //  ucStatus = ERR_NAND_PROG;
    }
    else if(ucStatus == NAND_TIMEOUT_ERROR)
    {
     //   ucStatus = ERR_NAND_HW_TOUT;
    }

    return (ucStatus);
}

//ȫƬ����NAND FLASH
void NAND_EraseChip(void)
{
    u8 status;
    u16 i=0;
    for(i=0;i<2048;i++)     //ѭ���������еĿ�
    {
        status=NAND_EraseBlock(i);
        if(status)
            NAND_DEBUG("Erase %d block fail!!��ERRORCODE %d\r\n",i,status);//����ʧ��
    }
}
