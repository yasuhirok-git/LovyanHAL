/*----------------------------------------------------------------------------/
  Lovyan HAL library - Hardware Abstraction Layer library .

Original Source:
 https://github.com/lovyan03/LovyanHAL/

Licence:
 [BSD](https://github.com/lovyan03/LovyanHAL/blob/master/license.txt)

Author:
 [lovyan03](https://twitter.com/lovyan03)
/----------------------------------------------------------------------------*/
#pragma once

#include "env.hpp"
#include "../LovyanHAL_MCU.hpp"
#include <hardware/clocks.h>

namespace lhal
{
 namespace v0
 {

  class LovyanHAL : public LovyanHAL_MCU
  {
  public:
    class GPIO_HAL : public GPIO_HAL_Base
    {
    public:
      class RAW
      {
        static volatile uint32_t* const _set_reg[];
        static volatile uint32_t* const _clr_reg[];
        static volatile uint32_t* const _set_oe_reg[];
        static volatile uint32_t* const _clr_oe_reg[];
        static const volatile uint32_t* const _read_reg[];

      public:
        static inline volatile uint32_t* getSetReg(gpio::port_num_t port = 0) { return _set_reg[port]; };
        static inline volatile uint32_t* getClrReg(gpio::port_num_t port = 0) { return _clr_reg[port]; };
        static inline volatile uint32_t* getOeSetReg(gpio::port_num_t port = 0) { return _set_oe_reg[port]; };
        static inline volatile uint32_t* getOeClrReg(gpio::port_num_t port = 0) { return _clr_oe_reg[port]; };
        static inline const volatile uint32_t* getReadReg(gpio::port_num_t port = 0) { return _read_reg[port]; };
      };
      static RAW Raw;
      static inline void writeMaskHigh(gpio::port_num_t port, gpio::pin_mask_t bitmask) { *RAW::getSetReg(port) = bitmask; };
      static inline void writeMaskLow(gpio::port_num_t port, gpio::pin_mask_t bitmask) { *RAW::getClrReg(port) = bitmask; };
      static inline void writeMask(gpio::port_num_t port, gpio::pin_mask_t bitmask, bool value)
      {
        if (value) {
          *RAW::getSetReg(port) = bitmask;
        } else {
          *RAW::getClrReg(port) = bitmask;
        }
      }
      static inline void writeMaskOeSet(gpio::port_num_t port, gpio::pin_mask_t bitmask) { *RAW::getOeSetReg(port) = bitmask; };
      static inline void writeMaskOeClr(gpio::port_num_t port, gpio::pin_mask_t bitmask) { *RAW::getOeClrReg(port) = bitmask; };
      static inline void writeMaskOe(gpio::port_num_t port, gpio::pin_mask_t bitmask, bool value)
      {
        if (value) {
          *RAW::getOeSetReg(port) = bitmask;
        } else {
          *RAW::getOeClrReg(port) = bitmask;
        }
      }

      static inline gpio::pin_mask_t readMask(gpio::port_num_t port, gpio::pin_mask_t bitmask) { return *RAW::getReadReg(port) & bitmask; };
      
      static inline void writeHigh(gpio_port_pin_t pp)
      {
        const gpio::pin_mask_t mask = pp.getMask();
        if ((mask & _is_od[pp.port]) == 0)
        {
          writeMaskHigh(pp.port, mask);
        }
        else
        {
          writeMaskOeClr(pp.port, mask);
        }
      }
      static inline void writeLow(gpio_port_pin_t pp)
      {
        const gpio::pin_mask_t mask = pp.getMask();
        if ((mask & _is_od[pp.port]) == 0)
        {
          writeMaskLow(pp.port, mask);
        }
        else
        {
          writeMaskOeSet(pp.port, mask);
        }
      }
      static inline void write(gpio_port_pin_t pp, bool value)
      {
        const gpio::pin_mask_t mask = pp.getMask();
        if ((mask & _is_od[pp.port]) == 0)
        {
          writeMask(pp.port, mask, value); 
        }
        else
        {
          writeMaskOe(pp.port, mask, !value); 
        }
      }
      static inline bool read(gpio_port_pin_t pp) { return readMask(pp.port, pp.getMask()); }

      static void setMode(gpio_port_pin_t pp, LovyanHAL::GPIO_HAL::mode_t mode);
      static void setFunction(gpio_port_pin_t pp, enum gpio_function f);

    protected:
      static gpio::pin_mask_t _is_od[];  // open drain port
    };

    LovyanHAL(void) : LovyanHAL_MCU() {}

    static GPIO_HAL Gpio;

    static constexpr error_t init(void) { return error_t::err_ok; }
    static inline void delay(uint32_t msec) { ::delay(msec); }

    static inline uint32_t getCpuFrequency(void) { return clock_get_hz(clk_sys); }
    /// Arduino環境のピン番号からMCUのポート+ピン番号に変換する…ESP32はMCUピン番号がそのままArduinoのピン番号となっているので変換が不要;
    static constexpr gpio_port_pin_t convertArduinoPinNumber(int arduino_pin_number)
    {
      return (gpio_port_pin_t)(NUM_BANK0_GPIOS > (uint32_t)arduino_pin_number ? arduino_pin_number : ~0u);
    }
  
  };

  class GPIO_host
  {
    volatile uint32_t* _reg_clr;
    volatile uint32_t* _reg_set;
    const volatile uint32_t* _reg_input;
    gpio::pin_mask_t _pin_mask;
    gpio_port_pin_t _gpio_pin;

  public:

    GPIO_host(gpio_port_pin_t pp) :
      _reg_clr { LovyanHAL::GPIO_HAL::RAW::getClrReg(pp.port) },
      _reg_set { LovyanHAL::GPIO_HAL::RAW::getSetReg(pp.port) },
      _reg_input { LovyanHAL::GPIO_HAL::RAW::getReadReg(pp.port) },
      _pin_mask { pp.getMask() },
      _gpio_pin { pp }
    {};

    void setMode(LovyanHAL::GPIO_HAL::mode_t mode) { LovyanHAL::Gpio.setMode(_gpio_pin, mode); }


    void writeHigh(void) { *LovyanHAL::GPIO_HAL::RAW::getSetReg() = _pin_mask; }
    void writeLow(void) { *LovyanHAL::GPIO_HAL::RAW::getClrReg() = _pin_mask; }
    void write(bool value) { *(value ? LovyanHAL::GPIO_HAL::RAW::getSetReg() : LovyanHAL::GPIO_HAL::RAW::getClrReg()) = _pin_mask; }
    bool read(void) { return *LovyanHAL::GPIO_HAL::RAW::getReadReg() & _pin_mask; }

//*
    inline void writeI2CHigh(void) { writeHigh(); }
    inline void writeI2CLow(void) { writeLow(); }
    inline void writeI2C(bool value) { write(value); }
/*/
    inline void writeI2CHigh(void) { setMode(LovyanHAL::GPIO_HAL::input_pullup); }
    inline void writeI2CLow(void) { setMode(LovyanHAL::GPIO_HAL::output_low); }
    inline void writeI2C(bool value) { if (value) { writeI2CHigh(); } else { writeI2CLow(); }; }
//*/
  };

  // TODO:
  // static inline uint32_t millis(void) { return (uint32_t) (esp_timer_get_time() / 1000ULL); }
  // static inline uint64_t micros(void) { return (esp_timer_get_time()); }
  // static inline void yield(void) { vPortYield(); }
  // static inline void delay(uint32_t msec) { vTaskDelay(msec / portTICK_PERIOD_MS); }
  // void delayMicroseconds(uint32_t usec);

 }
}

