/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-12-05     Bernard      the first version
 */

/*
 * COPYRIGHT (C) 2012, Shanghai Real Thread
 */
#include "board.h"
#include <drivers/mtd_nand.h>
#include "nand.h"
#include "ftl.h"
#ifdef RT_USING_MTD_NAND

/**
 * RT-Thread Generic Device Interface
 */
static rt_err_t _mtd_init(rt_device_t dev)
{
    if(FTL_Init())               //检测NAND FLASH,并初始化FTL
        {
        LOG_I("nand_check_err");
        return RT_ERROR;
        }else {
          return RT_EOK;
        }
//    rt_hw_mtd_nand_init();

}
//INIT_DEVICE_EXPORT(_mtd_init);
static rt_err_t _mtd_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t _mtd_close(rt_device_t dev)
{
    return RT_EOK;
}
u8 FTL_ReadData(u8 *pBuffer,u32 SectorNo,u16 SectorSize,u32 Count,u32 PageOffset1);
static rt_size_t _mtd_read(rt_device_t dev,
                           rt_off_t    pos,
                           void       *buffer,
                           rt_size_t   size)
{

    u32    sec_offset=pos%NAND_ECC_SECTOR_SIZE;
      //FTL_ReadSectors(buffer,pos/NAND_ECC_SECTOR_SIZE,NAND_ECC_SECTOR_SIZE,size/NAND_ECC_SECTOR_SIZE);//读取扇区
    FTL_ReadData(buffer,pos/NAND_ECC_SECTOR_SIZE,NAND_ECC_SECTOR_SIZE,size,sec_offset);
    return size;
}
 rt_size_t _mtd_erase(rt_device_t dev,
                            rt_off_t    pos,
                            rt_size_t   size)
{
    u32 LBNNo=pos/(128*1024);
    u32 BlockNum=size/(128*1024);
     FTL_EraseBlock(LBNNo,BlockNum);
     return size;
}


static rt_size_t _mtd_write(rt_device_t dev,
                            rt_off_t    pos,
                            const void *buffer,
                            rt_size_t   size)
{
u32    sec_index=pos/NAND_ECC_SECTOR_SIZE;
u32    sec_offset=pos%NAND_ECC_SECTOR_SIZE;
u32   SectorCount=  size/NAND_ECC_SECTOR_SIZE;
u32   SectorCountOffset=  size%NAND_ECC_SECTOR_SIZE;
#if 0
if(FTL_WriteData(buffer,sec_index,NAND_ECC_SECTOR_SIZE,size,sec_offset)==0)
    return size;
      else {
          return 0;
      }
#else
if ((sec_offset==0)&&(SectorCountOffset==0)) {
    if(FTL_WriteSectors((u8*)buffer,sec_index,NAND_ECC_SECTOR_SIZE,SectorCount)==0)//写入扇区
    return size;
    else {
        return 0;
    }
}else {
    FTL_WriteData((u8*)buffer,sec_index,NAND_ECC_SECTOR_SIZE,size,sec_offset);
  // LOG_I("SECTOR_SIZE: %d",SectorCountOffset);
    return size;
}
#endif
}

static rt_err_t _mtd_control(rt_device_t dev, int cmd, void *args)
{
    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops mtd_nand_ops =
{
    _mtd_init,
    _mtd_open,
    _mtd_close,
    _mtd_read,
    _mtd_write,
    _mtd_control
};
#endif

rt_err_t rt_mtd_nand_register_device(const char                *name,
                                     struct rt_mtd_nand_device *device)
{
    rt_device_t dev;

    dev = RT_DEVICE(device);
    RT_ASSERT(dev != RT_NULL);

    /* set device class and generic device interface */
    dev->type        = RT_Device_Class_MTD;
#ifdef RT_USING_DEVICE_OPS
    dev->ops         = &mtd_nand_ops;
#else
    dev->init        = _mtd_init;
    dev->open        = _mtd_open;
    dev->read        = _mtd_read;
    dev->write       = _mtd_write;
    dev->close       = _mtd_close;
    dev->control     = _mtd_control;
#endif

    dev->rx_indicate = RT_NULL;
    dev->tx_complete = RT_NULL;
//    dev->init(0);
    /* register to RT-Thread device system */
    return rt_device_register(dev, name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

}

rt_err_t read_id(struct rt_mtd_nand_device *device)
{
   return NAND_ReadID();
}


// rt_err_t (*read_page)(struct rt_mtd_nand_device *device,
//                       rt_off_t page,
//                       rt_uint8_t *data, rt_uint32_t data_len,
//                       rt_uint8_t *spare, rt_uint32_t spare_len)
// {
//     _mtd_read(device,
//             page*2048,
//                                data,
//                                data_len)
// }
// rt_err_t (*write_page)(struct rt_mtd_nand_device *device,
//                        rt_off_t page,
//                        const rt_uint8_t *data, rt_uint32_t data_len,
//                        const rt_uint8_t *spare, rt_uint32_t spare_len)
// {
//
//
// }
 // rt_err_t (*move_page)(struct rt_mtd_nand_device *device, rt_off_t src_page, rt_off_t dst_page);
//
// rt_err_t (*erase_block)(struct rt_mtd_nand_device *device, rt_uint32_t block)
// {
//
//
// }
// rt_err_t (*check_block)(struct rt_mtd_nand_device *device, rt_uint32_t block);
// rt_err_t (*mark_badblock)(struct rt_mtd_nand_device *device, rt_uint32_t block);





struct rt_mtd_nand_driver_ops rt_mtd_nand_driver_ops_struct;
struct rt_mtd_nand_device mtd_device;
void rt_mtd_nand_init(void)
{

    mtd_device.ops=&rt_mtd_nand_driver_ops_struct;
    rt_mtd_nand_driver_ops_struct.read_id=read_id;
//    rt_mtd_nand_driver_ops_struct.read_page=0;
//    rt_mtd_nand_driver_ops_struct.write_page=0;
//    rt_mtd_nand_driver_ops_struct.move_page=0;
//    rt_mtd_nand_driver_ops_struct.erase_block=0;
//    rt_mtd_nand_driver_ops_struct.check_block=0;
//    rt_mtd_nand_driver_ops_struct.mark_badblock=0;


    rt_mtd_nand_register_device("nand1",
            &mtd_device);

}
INIT_BOARD_EXPORT(rt_mtd_nand_init);
#if defined(RT_MTD_NAND_DEBUG) && defined(RT_USING_FINSH)
#include <finsh.h>
#define __is_print(ch) ((unsigned int)((ch) - ' ') < 127u - ' ')

static void mtd_dump_hex(const rt_uint8_t *ptr, rt_size_t buflen)
{
    unsigned char *buf = (unsigned char *)ptr;
    int i, j;
    for (i = 0; i < buflen; i += 16)
    {
        rt_kprintf("%06x: ", i);
        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%02x ", buf[i + j]);
            else
                rt_kprintf("   ");
        rt_kprintf(" ");
        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%c", __is_print(buf[i + j]) ? buf[i + j] : '.');
        rt_kprintf("\n");
    }
}

int mtd_nandid(const char *name)
{
    struct rt_mtd_nand_device *nand;
    nand = RT_MTD_NAND_DEVICE(rt_device_find(name));
    if (nand == RT_NULL)
    {
        rt_kprintf("no nand device found!\n");
        return -RT_ERROR;
    }

    return rt_mtd_nand_read_id(nand);
}

int mtd_nand_read(const char *name, int block, int page)
{
    rt_err_t result;
    rt_uint8_t *page_ptr;
    rt_uint8_t *oob_ptr;
    struct rt_mtd_nand_device *nand;

    nand = RT_MTD_NAND_DEVICE(rt_device_find(name));
    if (nand == RT_NULL)
    {
        rt_kprintf("no nand device found!\n");
        return -RT_ERROR;
    }

    page_ptr = rt_malloc(nand->page_size + nand->oob_size);
    if (page_ptr == RT_NULL)
    {
        rt_kprintf("out of memory!\n");
        return -RT_ENOMEM;
    }

    oob_ptr = page_ptr + nand->page_size;
    rt_memset(page_ptr, 0xff, nand->page_size + nand->oob_size);

    /* calculate the page number */
    page = block * nand->pages_per_block + page;
    result = rt_mtd_nand_read(nand, page, page_ptr, nand->page_size,
                              oob_ptr, nand->oob_size);

    rt_kprintf("read page, rc=%d\n", result);
    mtd_dump_hex(page_ptr, nand->page_size);
    mtd_dump_hex(oob_ptr, nand->oob_size);

    rt_free(page_ptr);
    return 0;
}

int mtd_nand_readoob(const char *name, int block, int page)
{
    struct rt_mtd_nand_device *nand;
    rt_uint8_t *oob_ptr;

    nand = RT_MTD_NAND_DEVICE(rt_device_find(name));
    if (nand == RT_NULL)
    {
        rt_kprintf("no nand device found!\n");
        return -RT_ERROR;
    }

    oob_ptr = rt_malloc(nand->oob_size);
    if (oob_ptr == RT_NULL)
    {
        rt_kprintf("out of memory!\n");
        return -RT_ENOMEM;
    }

    /* calculate the page number */
    page = block * nand->pages_per_block + page;
    rt_mtd_nand_read(nand, page, RT_NULL, nand->page_size,
                     oob_ptr, nand->oob_size);
    mtd_dump_hex(oob_ptr, nand->oob_size);

    rt_free(oob_ptr);
    return 0;
}

int mtd_nand_write(const char *name, int block, int page)
{
    rt_err_t result;
    rt_uint8_t *page_ptr;
    rt_uint8_t *oob_ptr;
    rt_uint32_t index;
    struct rt_mtd_nand_device *nand;

    nand = RT_MTD_NAND_DEVICE(rt_device_find(name));
    if (nand == RT_NULL)
    {
        rt_kprintf("no nand device found!\n");
        return -RT_ERROR;
    }

    page_ptr = rt_malloc(nand->page_size + nand->oob_size);
    if (page_ptr == RT_NULL)
    {
        rt_kprintf("out of memory!\n");
        return -RT_ENOMEM;
    }

    oob_ptr = page_ptr + nand->page_size;
    /* prepare page data */
    for (index = 0; index < nand->page_size; index ++)
    {
        page_ptr[index] = index & 0xff;
    }
    /* prepare oob data */
    for (index = 0; index < nand->oob_size; index ++)
    {
        oob_ptr[index] = index & 0xff;
    }

    /* calculate the page number */
    page = block * nand->pages_per_block + page;
    result = rt_mtd_nand_write(nand, page, page_ptr, nand->page_size,
                               oob_ptr, nand->oob_size);
    if (result != RT_MTD_EOK)
    {
        rt_kprintf("write page failed!, rc=%d\n", result);
    }

    rt_free(page_ptr);
    return 0;
}

int mtd_nand_erase(const char *name, int block)
{
    struct rt_mtd_nand_device *nand;
    nand = RT_MTD_NAND_DEVICE(rt_device_find(name));
    if (nand == RT_NULL)
    {
        rt_kprintf("no nand device found!\n");
        return -RT_ERROR;
    }
    return rt_mtd_nand_erase_block(nand, block);
}

int mtd_nand_erase_all(const char *name)
{
    rt_uint32_t index = 0;
    struct rt_mtd_nand_device *nand;

    nand = RT_MTD_NAND_DEVICE(rt_device_find(name));
    if (nand == RT_NULL)
    {
        rt_kprintf("no nand device found!\n");
        return -RT_ERROR;
    }

    for (index = 0; index < (nand->block_end - nand->block_start); index ++)
    {
        rt_mtd_nand_erase_block(nand, index);
    }

    return 0;
}
//void nand_test(void)
//{const *nand_device="nand1";
//u32 block=0;
//u32 page=0;
//
//    mtd_nand_erase(nand_device, block);
//    mtd_nand_write(nand_device, block, page);
//    mtd_nand_read(nand_device, block, page);
//
//}

#ifdef FINSH_USING_MSH
static void mtd_nand(int argc, char **argv)
{
    /* If the number of arguments less than 2 */
    if (argc < 3)
    {
help:
        rt_kprintf("\n");
        rt_kprintf("mtd_nand [OPTION] [PARAM ...]\n");
        rt_kprintf("         id       <name>            Get nandid by given name\n");
        rt_kprintf("         read     <name> <bn> <pn>  Read data on page <pn> of block <bn> of device <name>\n");
        rt_kprintf("         readoob  <name> <bn> <pn>  Read oob  on page <pn> of block <bn> of device <name>\n");
        rt_kprintf("         write    <name> <bn> <pn>  Run write test on page <pn> of block <bn> of device <name>\n");
        rt_kprintf("         erase    <name> <bn>       Erase on block <bn> of device <name>\n");
        rt_kprintf("         eraseall <name>            Erase all block on device <name>\n");
        return ;
    }
    else if (!strcmp(argv[1], "id"))
    {
        mtd_nandid(argv[2]);
    }
    else if (!strcmp(argv[1], "read"))
    {
        if (argc < 5)
        {
            rt_kprintf("The input parameters are too few!\n");
            goto help;
        }
        mtd_nand_read(argv[2], atoi(argv[3]), atoi(argv[4]));
    }
    else if (!strcmp(argv[1], "readoob"))
    {
        if (argc < 5)
        {
            rt_kprintf("The input parameters are too few!\n");
            goto help;
        }
        mtd_nand_readoob(argv[2], atoi(argv[3]), atoi(argv[4]));
    }
    else if (!strcmp(argv[1], "write"))
    {
        if (argc < 5)
        {
            rt_kprintf("The input parameters are too few!\n");
            goto help;
        }
        mtd_nand_write(argv[2], atoi(argv[3]), atoi(argv[4]));
    }
    else if (!strcmp(argv[1], "erase"))
    {
        if (argc < 4)
        {
            rt_kprintf("The input parameters are too few!\n");
            goto help;
        }
        mtd_nand_erase(argv[2], atoi(argv[3]));
    }
    else if (!strcmp(argv[1], "eraseall"))
    {
        mtd_nand_erase_all(argv[2]);
    }
    else
    {
        rt_kprintf("Input parameters are not supported!\n");
        goto help;
    }
}
//MSH_CMD_EXPORT(mtd_nand, MTD nand device test function);
#endif /* FINSH_USING_MSH */

#ifndef FINSH_USING_MSH_ONLY
FINSH_FUNCTION_EXPORT_ALIAS(mtd_nandid, nand_id, read ID - nandid(name));
FINSH_FUNCTION_EXPORT_ALIAS(mtd_nand_read, nand_read, read page in nand - nand_read(name, block, page));
FINSH_FUNCTION_EXPORT_ALIAS(mtd_nand_readoob, nand_readoob, read spare data in nand - nand_readoob(name, block, page));
FINSH_FUNCTION_EXPORT_ALIAS(mtd_nand_write, nand_write, write dump data to nand - nand_write(name, block, page));
FINSH_FUNCTION_EXPORT_ALIAS(mtd_nand_erase, nand_erase, nand_erase(name, block));
FINSH_FUNCTION_EXPORT_ALIAS(mtd_nand_erase_all, nand_erase_all, erase all of nand device - nand_erase_all(name, block));
#endif /* FINSH_USING_MSH_ONLY */

#endif /* defined(RT_MTD_NAND_DEBUG) && defined(RT_USING_FINSH) */

#endif /* RT_USING_MTD_NAND */
