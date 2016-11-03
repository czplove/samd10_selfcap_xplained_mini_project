/* This source file is part of the ATMEL QTouch Library 5.0.6 */

/*****************************************************************************
*
* \file
*
* \brief  This file contains the SAMD QTouch library sample user application.
*
*
* - Userguide:          QTouch Library Peripheral Touch Controller User Guide.
* - Support email:      www.atmel.com/design-support/
*
*
* Copyright (c) 2013-2015 Atmel Corporation. All rights reserved.
*
* \asf_license_start
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. The name of Atmel may not be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* 4. This software may only be redistributed and used in connection with an
*    Atmel microcontroller product.
*
* THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \asf_license_stop
*
******************************************************************************/

/**
* Include header files for all drivers that have been imported from
* Atmel Software Framework (ASF).
*/
#include <asf.h>
#include "touch_api_ptc.h"
/**
* Discrete LED output pin definition
*/
//#define LED_1_PIN       PIN_PA08
//#define LED_2_PIN       PIN_PA09
#define LED_3_PIN       PIN_PA25

#define TIME_PERIOD_1MSEC 33u
#define TC_COUNT_VALUE 5535

#define POWER_ON    0X01    /* 开机 */
#define POWER_OFF   0X02    /* 关机 */
#define UP          0X03    /* UP */
#define MENU        0X04    /* 菜单 */
#define DOWN        0X05    /* DOWN */
#define WIND        0X06    /* 调风 */
#define NETIN       0X07    /* 加网 */
#define NETOUT      0X08    /* 退网 */

//#define TC_COUNT_VALUE 60000

volatile uint16_t touch_time_counter = 0u;

struct rtc_module rtc_instance;
static struct tc_module tc_instance;

uint8_t touch_buffer[3];
uint8_t GroupKeyFlag = 0;
uint8_t TimerOverFlowFlag = 0;
uint8_t ClickCount = 0;
uint8_t TouchFlag = 0;
uint8_t TouchFlag1 = 0;
uint8_t TouchFlag2 = 0;
uint8_t TouchFlag3 = 0;
uint8_t TouchFlag4 = 0;
uint8_t PowerCount = 0;


/**
* Prototypes
*/

/*! \brief Configure Port pins
*
*/
void configure_port_pins(void);

/*! \brief Initialize timer
*
*/
void timer_init( void );
void SendDate(uint8_t dat);

void set_timer_period(void);

/*! \brief RTC timer overflow callback
*
*/
void rtc_overflow_callback(void);

/*! \brief Configure the RTC timer callback
*
*/
void configure_rtc_callbacks(void);

/*! \brief Configure the RTC timer count after which interrupts comes
*
*/
void configure_rtc_count(void);

/*! \brief Configure the RTC timer overflow callback
*
*/
void rtc_overflow_callback(void)
{
  /* Do something on RTC overflow here */
  if (touch_time_counter == touch_time.measurement_period_ms) {
    touch_time.time_to_measure_touch = 1;
    touch_time.current_time_ms = touch_time.current_time_ms +
      touch_time.measurement_period_ms;
    touch_time_counter = 0u;
  } else {
    touch_time_counter++;
  }
}
void set_timer_period(void)
{
  
}
/*! \brief Configure the RTC timer callback
*
*/
void configure_rtc_callbacks(void)
{
  /* register callback */
  rtc_count_register_callback(&rtc_instance,
                              rtc_overflow_callback, RTC_COUNT_CALLBACK_OVERFLOW);
  /* Enable callback */
  rtc_count_enable_callback(&rtc_instance, RTC_COUNT_CALLBACK_OVERFLOW);
}

/*! \brief Configure the RTC timer count after which interrupts comes
*
*/
void configure_rtc_count(void)
{
  struct rtc_count_config config_rtc_count;
  rtc_count_get_config_defaults(&config_rtc_count);
  
  config_rtc_count.prescaler           = RTC_MODE0_CTRL_PRESCALER_DIV1;
  config_rtc_count.mode                = RTC_COUNT_MODE_16BIT;
  config_rtc_count.continuously_update = true;
  /* initialize rtc */
  rtc_count_init(&rtc_instance, RTC, &config_rtc_count);
  
  /* enable rtc */
  rtc_count_enable(&rtc_instance);
}


/*! \brief Initialize timer
*
*/
void timer_init(void)
{
  /* Configure and enable RTC */
  configure_rtc_count();
  
  /* Configure and enable callback */
  configure_rtc_callbacks();
  
  /* Set Timer Period */
  
  rtc_count_set_period(&rtc_instance, TIME_PERIOD_1MSEC);
}


/** TC Callback function.
*/
static void tc_callback_to_counter(struct tc_module *const module_inst)
{
  static uint32_t count = 0;
  count ++;
  if(count%1000 == 0)/* 定时时间10s */
  {
    TimerOverFlowFlag = 1; 
    tc_disable(&tc_instance);
    tc_stop_counter(&tc_instance);
  }
  
  tc_set_count_value(module_inst,TC_COUNT_VALUE);
}

/** Configures  TC function with the  driver.
*/
static void configure_tc(void)
{
  struct tc_config config_tc;
  
  tc_get_config_defaults(&config_tc);
  config_tc.counter_size    = TC_COUNTER_SIZE_16BIT;
  config_tc.counter_16_bit.value = TC_COUNT_VALUE;
  
  tc_init(&tc_instance, CONF_TC_INSTANCE, &config_tc);
//  tc_enable(&tc_instance);
//  tc_disable(&tc_instance);
//  tc_stop_counter(&tc_instance);
}

/** Registers TC callback function with the  driver.
*/
static void configure_tc_callbacks(void)
{
  tc_register_callback(
                       &tc_instance,
                       tc_callback_to_counter,
                       TC_CALLBACK_OVERFLOW);
  tc_enable_callback(&tc_instance, TC_CALLBACK_OVERFLOW);
}


void configure_port_pins(void)
{
  struct port_config config_port_pin;
  port_get_config_defaults(&config_port_pin);
  
  config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
  port_pin_set_config(LED_3_PIN, &config_port_pin);
}

struct usart_module usart_instance;

#define MAX_RX_BUFFER_LENGTH   3

volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];
void usart_read_callback(struct usart_module *const usart_module)
{
  usart_write_buffer_job(&usart_instance,
                         (uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
}

void usart_write_callback(struct usart_module *const usart_module)
{
  
  usart_read_buffer_job(&usart_instance,
                        (uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
  //port_pin_toggle_output_level(LED_0_PIN);
}
//! [callback_funcs]

//! [setup]
void configure_usart(void)
{
  struct usart_config config_usart;
  usart_get_config_defaults(&config_usart);
  config_usart.baudrate    = 9600;
  config_usart.mux_setting = USART_RX_3_TX_2_XCK_3 ;
//  config_usart.pinmux_pad0 = PINMUX_PA14C_SERCOM0_PAD0;
//  config_usart.pinmux_pad1 = PINMUX_PA15C_SERCOM0_PAD1;
  config_usart.pinmux_pad2 = PINMUX_PA08C_SERCOM1_PAD2;
  config_usart.pinmux_pad3 = PINMUX_PA09C_SERCOM1_PAD3;
  config_usart.pinmux_pad0 = PINMUX_UNUSED;
  config_usart.pinmux_pad1 = PINMUX_UNUSED;
  while (usart_init(&usart_instance,
                    SERCOM1, &config_usart) != STATUS_OK) {	}
  usart_enable(&usart_instance);
  usart_register_callback(&usart_instance,
                          usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
  usart_register_callback(&usart_instance,
                          usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
  usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
  usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}

volatile uint8_t button_state0;
volatile uint8_t button_state1;
volatile uint8_t button_state2;
volatile uint8_t button_state3;
volatile uint8_t button_state4;

/**
* MAIN
*/
int main(void)
{
  /**
  * Initialize and configure system and generic clocks.
  * Use conf_clocks.h to configure system and generic clocks.
  */
  system_init();
  
  system_interrupt_enable_global();
  
  /**
  * Initialize delay service.
  */
  delay_init();
  
  /**
  * Initialize timer.
  */
  timer_init();
  
  /*Configures  TC driver*/
  configure_tc();
  
  /*Configures TC callback*/
  configure_tc_callbacks();
  
  configure_usart();
  /**
  * Initialize QTouch library and configure touch sensors.
  */
  touch_sensors_init();
  
  /**
  * Configure port pins
  */
  configure_port_pins();

  port_pin_set_output_level(LED_3_PIN,1);
  
  /* Configure System Sleep mode to STANDBY MODE. */
  system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
  
  //  uint8_t string[] = "Hello World!\r\n";
  //  usart_write_buffer_wait(&usart_instance, string, sizeof(string));
  //  
  //  usart_read_buffer_job(&usart_instance,
  //                        (uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
  
  while (1) {
    //system_sleep();
    touch_sensors_measure();
    
    if ((p_selfcap_measure_data->measurement_done_touch == 1u)) 
    {
      p_selfcap_measure_data->measurement_done_touch = 0u;
      
      button_state0 = GET_SELFCAP_SENSOR_STATE(0);
      button_state1 = GET_SELFCAP_SENSOR_STATE(1);
      button_state2 = GET_SELFCAP_SENSOR_STATE(2);
      button_state3 = GET_SELFCAP_SENSOR_STATE(3);
      button_state4 = GET_SELFCAP_SENSOR_STATE(4);
      ClickCount = 0;
      GroupKeyFlag = 0;
      tc_disable(&tc_instance);
      tc_stop_counter(&tc_instance);
      
      while(button_state0)
      {
        touch_sensors_measure();
        button_state0 = GET_SELFCAP_SENSOR_STATE(0);
        button_state1 = GET_SELFCAP_SENSOR_STATE(1);
        button_state2 = GET_SELFCAP_SENSOR_STATE(2);
        if(button_state1 && !TouchFlag1)
        {
          GroupKeyFlag = 1;
          TouchFlag1 = 1;
          ClickCount++;
          if(ClickCount == 4)
          {
            ClickCount = 0;
            SendDate(NETIN);
          }
        }
        if(!button_state1 && TouchFlag1)
        {
          TouchFlag1 = 0;
        }
        
        while(button_state2&&button_state0)
        {
          GroupKeyFlag = 1;
          touch_sensors_measure();
          button_state2 = GET_SELFCAP_SENSOR_STATE(2);
          button_state0 = GET_SELFCAP_SENSOR_STATE(0);
          tc_enable(&tc_instance);
          if(TimerOverFlowFlag == 1)
          {
            SendDate(NETOUT);
            tc_disable(&tc_instance);
            tc_stop_counter(&tc_instance);
            TimerOverFlowFlag =0;
            
          }
        }                             
        if(!button_state2 && GroupKeyFlag == 1)
        {
          TouchFlag2 = 0;
        }
        
        if(!GroupKeyFlag && !button_state0)
        {
          GroupKeyFlag = 1;
          SendDate(MENU);
        }
      }
      
      if(button_state3 && !TouchFlag3)
      {
        TouchFlag3 = 1;
        PowerCount++;
        if(PowerCount%2)
        {
          SendDate(POWER_ON);
        }
        else 
        {
          SendDate(POWER_OFF);
        }
      }
      if(!button_state3 && TouchFlag3)
      {
        TouchFlag3 = 0;
      }
      
      if(button_state2 && !TouchFlag2)
      {
        TouchFlag2 = 1;
        SendDate(DOWN);
         
      }
      if(!button_state2 && TouchFlag2)
      {
        TouchFlag2 = 0;
      }
      
      if(button_state1 && !TouchFlag1)
      {
        TouchFlag1 = 1;
        SendDate(WIND);
      }
      if(!button_state1 && TouchFlag1)
      {
        TouchFlag1 = 0;
      }
      
      if(button_state4 && !TouchFlag4)
      {
        TouchFlag4 = 1;
        SendDate(UP);
      }
      if(!button_state4 && TouchFlag4)
      {
        TouchFlag4 = 0;
      }
      
      
    } /* measurement done touch */
  } /* while(1) */
} /* main */


/**
* @fun    void SendDate
* @brief  串口发送数据函数
*         2015/12/29 星期二,Administrator
* @param  uint8 data
*
* @retval 
*/
void SendDate(uint8_t dat)
{
  touch_buffer[0] = 0x90;
  touch_buffer[1] = dat;
  touch_buffer[2] = 0x97;
  usart_write_buffer_job(&usart_instance,
                         (uint8_t *)touch_buffer, MAX_RX_BUFFER_LENGTH);
}



