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
        static inline volatile uint32_t* getOESetReg(gpio::port_num_t port = 0) { return _set_oe_reg[port]; };
        static inline volatile uint32_t* getOEClrReg(gpio::port_num_t port = 0) { return _clr_oe_reg[port]; };
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
      static inline void writeMaskOESet(gpio::port_num_t port, gpio::pin_mask_t bitmask) { *RAW::getOESetReg(port) = bitmask; };
      static inline void writeMaskOEClr(gpio::port_num_t port, gpio::pin_mask_t bitmask) { *RAW::getOEClrReg(port) = bitmask; };
      static inline void writeMaskOE(gpio::port_num_t port, gpio::pin_mask_t bitmask, bool value)
      {
        if (value) {
          *RAW::getOESetReg(port) = bitmask;
        } else {
          *RAW::getOEClrReg(port) = bitmask;
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
          writeMaskOEClr(pp.port, mask);
        }
      }

      static inline void writeLow(gpio_port_pin_t pp)
      {
        const gpio::pin_mask_t mask =  pp.getMask();
        if ((mask & _is_od[pp.port]) == 0)
        {
          writeMaskLow(pp.port, mask);
        }
        else
        {
          writeMaskOESet(pp.port, mask);
        }
      }

      static inline void write(gpio_port_pin_t pp, bool value, gpio::pin_mask_t mask)
      {
        if ((mask & _is_od[pp.port]) == 0)
        {
          writeMask(pp.port, mask, value); 
        }
        else
        {
          writeMaskOE(pp.port, mask, !value); 
        }
      }

      static inline void write(gpio_port_pin_t pp, bool value)
      {
        write(pp, value, pp.getMask());
      }

      static inline bool read(gpio_port_pin_t pp) { return readMask(pp.port, pp.getMask()); }

      static void setMode(gpio_port_pin_t pp, LovyanHAL::GPIO_HAL::mode_t mode);
      static void setFunction(gpio_port_pin_t pp, enum gpio_function f);

    protected:
      static gpio::pin_mask_t _is_od[];  // open drain port
    };

    LovyanHAL(void) : LovyanHAL_MCU() {}

    static GPIO_HAL Gpio;

    static constexpr error_t init() { return error_t::err_ok; }
    static inline void delay(uint32_t msec) { ::delay(msec); }

    static inline uint32_t getCpuFrequency() { return clock_get_hz(clk_sys); }
    /// Arduino環境のピン番号からMCUのポート+ピン番号に変換する…ESP32はMCUピン番号がそのままArduinoのピン番号となっているので変換が不要;
    static constexpr gpio_port_pin_t convertArduinoPinNumber(int arduino_pin_number)
    {
      return (gpio_port_pin_t)(NUM_BANK0_GPIOS > (uint32_t)arduino_pin_number ? arduino_pin_number : ~0u);
    }
  
  };

  class GPIO_host
  {
  private:
    inline void writeNormalHigh()
    {
      *_reg_set = _pin_mask;
    }

    inline void writeNormalLow()
    {
      *_reg_clr = _pin_mask;
    }

    inline void writeODHigh()
    {
      *_reg_oe_clr = _pin_mask;
    }

    inline void writeODLow()
    {
      *_reg_oe_set = _pin_mask;
    }

    inline void writeOD(bool value)
    {
      if (value)
      {
        writeODHigh();
      }
      else
      {
        writeODLow();
      }
    }

    volatile uint32_t* _reg_clr;
    volatile uint32_t* _reg_set;
    volatile uint32_t* _reg_oe_clr;
    volatile uint32_t* _reg_oe_set;
    const volatile uint32_t* _reg_input;
    gpio::pin_mask_t _pin_mask;
    gpio_port_pin_t _gpio_pin;
    bool _is_od;

  public:

    GPIO_host(const gpio_port_pin_t pp);
    GPIO_host();

    void setMode(LovyanHAL::GPIO_HAL::mode_t mode);

    void writeHigh()
    {
      if (_is_od)
      {
        writeODHigh();
      }
      else
      {
        writeNormalHigh();
      }
    }

    void writeLow()
    {
      if (_is_od)
      {
        writeODLow();
      }
      else
      {
        writeNormalLow();
      }
    }

    void write(bool value)
    {
      if (value)
      {
        writeHigh();
      }
      else
      {
        writeLow();
      }
    }

    inline bool read()
    {
      return ((*_reg_input & _pin_mask) != 0);
    }
 
//*
    inline void writeI2CHigh() { writeODHigh(); }
    inline void writeI2CLow() { writeODLow(); }
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

