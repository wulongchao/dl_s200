/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date         Author        Notes
 * 2018-12-04   balanceTWK    first version
 */

#include "board.h"
#include "stm32f4xx_hal_rtc_ex.h"
#ifdef BSP_USING_ONCHIP_RTC


//#ifndef HAL_RTCEx_BKUPRead
//#define HAL_RTCEx_BKUPRead(x1, x2) (~BKUP_REG_DATA)
//#endif
//#ifndef HAL_RTCEx_BKUPWrite
//#define HAL_RTCEx_BKUPWrite(x1, x2, x3)
//#endif

#ifndef RTC_BKP_DR1
#define RTC_BKP_DR1 RT_NULL
#endif

//#define DRV_DEBUG
#define LOG_TAG             "drv.rtc"
#include <drv_log.h>

#define BKUP_REG_DATA 0xA5A5

static struct rt_device rtc;

 RTC_HandleTypeDef RTC_Handler;

static time_t get_rtc_timestamp(void)
{
    RTC_TimeTypeDef RTC_TimeStruct = {0};
    RTC_DateTypeDef RTC_DateStruct = {0};
    struct tm tm_new;

    HAL_RTC_GetTime(&RTC_Handler, &RTC_TimeStruct, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&RTC_Handler, &RTC_DateStruct, RTC_FORMAT_BIN);

    tm_new.tm_sec  = RTC_TimeStruct.Seconds;
    tm_new.tm_min  = RTC_TimeStruct.Minutes;
    tm_new.tm_hour = RTC_TimeStruct.Hours;
    tm_new.tm_mday = RTC_DateStruct.Date;
    tm_new.tm_mon  = RTC_DateStruct.Month - 1;
    tm_new.tm_year = RTC_DateStruct.Year + 100;

    LOG_D("get rtc time.");
    return mktime(&tm_new);
}

static rt_err_t set_rtc_time_stamp(time_t time_stamp)
{
    RTC_TimeTypeDef RTC_TimeStruct = {0};
    RTC_DateTypeDef RTC_DateStruct = {0};
    struct tm *p_tm;

    p_tm = localtime(&time_stamp);
    if (p_tm->tm_year < 100)
    {
        return -RT_ERROR;
    }

    RTC_TimeStruct.Seconds = p_tm->tm_sec ;
    RTC_TimeStruct.Minutes = p_tm->tm_min ;
    RTC_TimeStruct.Hours   = p_tm->tm_hour;
    RTC_DateStruct.Date    = p_tm->tm_mday;
    RTC_DateStruct.Month   = p_tm->tm_mon + 1 ;
    RTC_DateStruct.Year    = p_tm->tm_year - 100;
    RTC_DateStruct.WeekDay = p_tm->tm_wday + 1;

    if (HAL_RTC_SetTime(&RTC_Handler, &RTC_TimeStruct, RTC_FORMAT_BIN) != HAL_OK)
    {
        return -RT_ERROR;
    }
    if (HAL_RTC_SetDate(&RTC_Handler, &RTC_DateStruct, RTC_FORMAT_BIN) != HAL_OK)
    {
        return -RT_ERROR;
    }

    LOG_E("set rtc time.");
    HAL_RTCEx_BKUPWrite(&RTC_Handler, RTC_BKP_DR1, BKUP_REG_DATA);
    return RT_EOK;
}

static void rt_rtc_init(void)
{
#ifndef SOC_SERIES_STM32H7
    __HAL_RCC_PWR_CLK_ENABLE();
#endif

    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
#ifdef BSP_RTC_USING_LSI
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
#else
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
#endif
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
}

static rt_err_t rt_rtc_config(struct rt_device *dev)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    HAL_PWR_EnableBkUpAccess();
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
#ifdef BSP_RTC_USING_LSI
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
#else
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
#endif
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /* Enable RTC Clock */
    __HAL_RCC_RTC_ENABLE();
    LOG_I("rt_rtc_config");
    RTC_Handler.Instance = RTC;
    if (HAL_RTCEx_BKUPRead(&RTC_Handler, RTC_BKP_DR1) != BKUP_REG_DATA)
    {
        LOG_I("RTC hasn't been configured, please use <date> command to config.");


#if defined(SOC_SERIES_STM32F1)
        RTC_Handler.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
        RTC_Handler.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
#elif defined(SOC_SERIES_STM32F0)

        /* set the frequency division */
#ifdef BSP_RTC_USING_LSI
        RTC_Handler.Init.AsynchPrediv = 0XA0;
        RTC_Handler.Init.SynchPrediv = 0xFA;
#else
        RTC_Handler.Init.AsynchPrediv = 0X7F;
        RTC_Handler.Init.SynchPrediv = 0x0130;
#endif /* BSP_RTC_USING_LSI */

        RTC_Handler.Init.HourFormat = RTC_HOURFORMAT_24;
        RTC_Handler.Init.OutPut = RTC_OUTPUT_DISABLE;
        RTC_Handler.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
        RTC_Handler.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
#elif defined(SOC_SERIES_STM32F2) || defined(SOC_SERIES_STM32F4) || defined(SOC_SERIES_STM32F7) || defined(SOC_SERIES_STM32L4) || defined(SOC_SERIES_STM32H7)

        /* set the frequency division */
#ifdef BSP_RTC_USING_LSI
        RTC_Handler.Init.AsynchPrediv = 0X7D;
#else
        RTC_Handler.Init.AsynchPrediv = 0X7F;
#endif /* BSP_RTC_USING_LSI */
        RTC_Handler.Init.SynchPrediv = 0XFF;

        RTC_Handler.Init.HourFormat = RTC_HOURFORMAT_24;
        RTC_Handler.Init.OutPut = RTC_OUTPUT_DISABLE;
        RTC_Handler.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
        RTC_Handler.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
#endif
        if (HAL_RTC_Init(&RTC_Handler) != HAL_OK)
        {
            return -RT_ERROR;
        }


    }
    return RT_EOK;
}

static rt_err_t rt_rtc_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    RT_ASSERT(dev != RT_NULL);
    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
        *(rt_uint32_t *)args = get_rtc_timestamp();
        LOG_D("RTC: get rtc_time %x\n", *(rt_uint32_t *)args);
        break;

    case RT_DEVICE_CTRL_RTC_SET_TIME:
        if (set_rtc_time_stamp(*(rt_uint32_t *)args))
        {
            result = -RT_ERROR;
        }
        LOG_D("RTC: set rtc_time %x\n", *(rt_uint32_t *)args);
        break;
    }

    return result;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops rtc_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    rt_rtc_control
};
#endif

static rt_err_t rt_hw_rtc_register(rt_device_t device, const char *name, rt_uint32_t flag)
{
    RT_ASSERT(device != RT_NULL);
    LOG_I("rt_rtc_init");
    rt_rtc_init();
    if (rt_rtc_config(device) != RT_EOK)
    {
        return -RT_ERROR;
    }
#ifdef RT_USING_DEVICE_OPS
    device->ops         = &rtc_ops;
#else
    device->init        = RT_NULL;
    device->open        = RT_NULL;
    device->close       = RT_NULL;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = rt_rtc_control;
#endif
    device->type        = RT_Device_Class_RTC;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->user_data   = RT_NULL;

    /* register a character device */
    return rt_device_register(device, name, flag);


}
char* time_month_str[12]=
{
        "Jan",
        "Feb",
        "Mar",
        "Apr",
        "May",

        "Jun",
        "Jul",
        "Aug",
        "Sep",
        "Oct",

        "Nov",
        "Dec",
};
void convert_time(char *str,uint16_t* year,u16 *month,u16* day, u16* hour, u16* min ,u16* sec )
{
            char* buf[6];
             u8 i;
              char str1[16]= {0};
                  buf[0] = strtok(str," ");
                  buf[1] = strtok(0," ");
                  buf[2] = strtok(0," ");
              strcpy(str1,__TIME__);
                  buf[3] = strtok(str1,":");
                  buf[4] = strtok(0,":");
                  buf[5] = strtok(0,":");
              for (i = 0; i < 12; ++i) {
                  if (memcmp(buf[0],time_month_str[i],strlen(buf[0]))==0) {
                      *month=i+1;
                      break;
                  }
              }
              *day=atoi(buf[1]);
              *year=atoi(buf[2]);
              *hour = atoi(buf[3]);
              *min =atoi(buf[4]);
              *sec =atoi(buf[5]);
}



//����  set_rtc_time_stamp�� ����
    //HAL_RTCEx_BKUPWrite(&RTC_Handler, RTC_BKP_DR1, BKUP_REG_DATA);
int rt_hw_rtc_init(void)
{
    rt_err_t result;
  volatile  u32 daback=0;
    result = rt_hw_rtc_register(&rtc, "rtc", RT_DEVICE_FLAG_RDWR);
    if (result != RT_EOK)
    {
        LOG_E("rtc register err code: %d", result);
        return result;
    }
    daback=HAL_RTCEx_BKUPRead(&RTC_Handler, RTC_BKP_DR1);
    if(daback!= BKUP_REG_DATA)
       {    char str[16]= {0};
            char str1[16]= {0};
            char* buf[6];

             uint16_t year = 2020;
             u16 month = 1,day = 2,  hour = 12, min = 0, sec = 0;
             strcpy(str,__DATE__)  ;

             convert_time(str,&year, &month,&day,  &hour, &min, &sec);
               // LOG_I("%s,%s,%s",buf[0],buf[1],buf[2]);
             set_time(hour, min, sec);
             set_date(year, month, day);
             daback=HAL_RTCEx_BKUPRead(&RTC_Handler, RTC_BKP_DR1);
             LOG_E("BKUPRead:%x",daback);
     }
    {


    }
    LOG_D("rtc init success");
    return RT_EOK;
}
//INIT_DEVICE_EXPORT(rt_hw_rtc_init);
INIT_BOARD_EXPORT(rt_hw_rtc_init);
#endif /* BSP_USING_ONCHIP_RTC */
