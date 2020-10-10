/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2018-12-07     balanceTWK        first version
 */

#include <board.h>

#ifdef RT_USING_WDT

//#define DRV_DEBUG
#define LOG_TAG             "drv.wdt"
#include <drv_log.h>
#include "stm32f4xx_hal_iwdg.h"
struct stm32_wdt_obj
{
    IWDG_HandleTypeDef hiwdg;
    rt_uint16_t is_start;
};
static struct stm32_wdt_obj stm32_wdt;
static struct rt_watchdog_ops ops;
static rt_watchdog_t watchdog;
#define IWDG_DEVICE_NAME    "wdt"//"iwg"    /* 看门狗设备名称 */
static rt_device_t wdg_dev;         /* 看门狗设备句柄 */

 void wdt_idle_hook(void)
{
    /* 在空闲线程的回调函数里喂狗 */
    rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
    //LOG_I("feed the dog!\n ");
}

static int iwdg_sample(int argc, char *argv[])
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t timeout =5;    /* 溢出时间  4096/125 */
    char device_name[RT_NAME_MAX];

    /* 判断命令行参数是否给定了设备名称 */
    if (argc == 2)
    {
        rt_strncpy(device_name, argv[1], RT_NAME_MAX);
    }
    else
    {
        rt_strncpy(device_name, IWDG_DEVICE_NAME, RT_NAME_MAX);
    }
    /* 根据设备名称查找看门狗设备，获取设备句柄 */
    wdg_dev = rt_device_find(device_name);
    if (!wdg_dev)
    {
        rt_kprintf("find %s failed!\n", device_name);
        return RT_ERROR;
    }
    /* 初始化设备 */
    ret = rt_device_init(wdg_dev);
    if (ret != RT_EOK)
    {
        rt_kprintf("initialize %s failed!\n", device_name);
        return RT_ERROR;
    }
    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_START, &timeout);
    /* 设置看门狗溢出时间 */
    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);
    if (ret != RT_EOK)
    {
        rt_kprintf("set %s timeout failed!\n", device_name);
        return RT_ERROR;
    }
    /* 设置空闲线程回调函数 */
    //rt_thread_idle_sethook(wdt_idle_hook);

    return ret;
}
/* 导出到 msh 命令列表中 */
//MSH_CMD_EXPORT(iwdg_sample, iwdg sample);



static rt_err_t wdt_init(rt_watchdog_t *wdt)
{
    return RT_EOK;
}

static rt_err_t wdt_control(rt_watchdog_t *wdt, int cmd, void *arg)
{
    switch (cmd)
    {
        /* feed the watchdog */
    case RT_DEVICE_CTRL_WDT_KEEPALIVE:
        if(HAL_IWDG_Refresh(&stm32_wdt.hiwdg) != HAL_OK)
        {
            LOG_E("watch dog keepalive fail.");
        }
        break;
        /* set watchdog timeout */
    case RT_DEVICE_CTRL_WDT_SET_TIMEOUT:
#if defined(LSI_VALUE)
        if(LSI_VALUE)
        {
            stm32_wdt.hiwdg.Init.Reload = (*((rt_uint32_t*)arg)) * LSI_VALUE / 256 ;
        }
        else
        {
            LOG_E("Please define the value of LSI_VALUE!");
        }
        if(stm32_wdt.hiwdg.Init.Reload > 0xFFF)
        {
            LOG_E("wdg set timeout parameter too large, please less than %ds",0xFFF * 256 / LSI_VALUE);
            return -RT_EINVAL;
        }
#else
  #error "Please define the value of LSI_VALUE!"
#endif
        if(stm32_wdt.is_start)
        {
            if (HAL_IWDG_Init(&stm32_wdt.hiwdg) != HAL_OK)
            {
                LOG_E("wdg set timeout failed.");
                return -RT_ERROR;
            }
        }
        break;
    case RT_DEVICE_CTRL_WDT_GET_TIMEOUT:
#if defined(LSI_VALUE)
        if(LSI_VALUE)
        {
            (*((rt_uint32_t*)arg)) = stm32_wdt.hiwdg.Init.Reload * 256 / LSI_VALUE;
        }
        else
        {
            LOG_E("Please define the value of LSI_VALUE!");
        }
#else
  #error "Please define the value of LSI_VALUE!"
#endif
        break;
    case RT_DEVICE_CTRL_WDT_START:
        if (HAL_IWDG_Init(&stm32_wdt.hiwdg) != HAL_OK)
        {
            LOG_E("wdt start failed.");
            return -RT_ERROR;
        }
        stm32_wdt.is_start = 1;
        break;
    default:
        LOG_W("This command is not supported.");
        return -RT_ERROR;
    }
    return RT_EOK;
}

int rt_wdt_init(void)
{
#if defined(SOC_SERIES_STM32H7)
    stm32_wdt.hiwdg.Instance = IWDG1;
#else
    stm32_wdt.hiwdg.Instance = IWDG;
#endif
    stm32_wdt.hiwdg.Init.Prescaler = IWDG_PRESCALER_256;

    stm32_wdt.hiwdg.Init.Reload = 0x00000FFF;
#if defined(SOC_SERIES_STM32F0) || defined(SOC_SERIES_STM32L4) || defined(SOC_SERIES_STM32F7) \
    || defined(SOC_SERIES_STM32H7)
    stm32_wdt.hiwdg.Init.Window = 0x00000FFF;
#endif
    stm32_wdt.is_start = 0;

    ops.init = &wdt_init;
    ops.control = &wdt_control;
    watchdog.ops = &ops;
    /* register watchdog device */
    if (rt_hw_watchdog_register(&watchdog, "wdt", RT_DEVICE_FLAG_DEACTIVATE, RT_NULL) != RT_EOK)
    {
        LOG_E("wdt device register failed.");
        return -RT_ERROR;
    }
    LOG_I("wdt device register success.");
    iwdg_sample(0,0);
    return RT_EOK;
}
//INIT_PREV_EXPORT(rt_wdt_init);
#ifdef   WDT_ENABLE
INIT_BOARD_EXPORT(rt_wdt_init);
#endif

#endif /* RT_USING_WDT */
