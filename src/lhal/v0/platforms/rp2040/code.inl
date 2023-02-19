/*----------------------------------------------------------------------------/
  Lovyan HAL library - Hardware Abstraction Layer library .

Original Source:
 https://github.com/lovyan03/LovyanHAL/

Licence:
 [BSD](https://github.com/lovyan03/LovyanHAL/blob/master/license.txt)

Author:
 [lovyan03](https://twitter.com/lovyan03)
/----------------------------------------------------------------------------*/
#if defined ( LHAL_TARGET_PLATFORM )
#include "../mcu_impl.inl"

#include <Arduino.h>
#include "LHAL.hpp"
#include <hardware/structs/sio.h>

#include <hardware/structs/iobank0.h>

namespace lhal
{
 namespace v0
 {

  LovyanHAL::GPIO_HAL LovyanHAL::Gpio;

  GPIO_host LovyanHAL::GPIO_HAL_Base::getHost(gpio_port_pin_t pin) { return GPIO_host { pin }; }

  gpio::pin_mask_t LovyanHAL::GPIO_HAL::GPIO_HAL::_is_od[1];

  volatile uint32_t* const LovyanHAL::GPIO_HAL::RAW::_set_reg[] =
  {
    (static_cast<volatile uint32_t* >(&sio_hw->gpio_set)),
  };
  volatile uint32_t* const LovyanHAL::GPIO_HAL::RAW::_clr_reg[] =
  {
    (static_cast<volatile uint32_t* >(&sio_hw->gpio_clr)),
  };

  volatile uint32_t* const LovyanHAL::GPIO_HAL::RAW::_set_oe_reg[] =
  {
    (static_cast<volatile uint32_t* >(&sio_hw->gpio_oe_set)),
  };
  volatile uint32_t* const LovyanHAL::GPIO_HAL::RAW::_clr_oe_reg[] =
  {
    (static_cast<volatile uint32_t* >(&sio_hw->gpio_oe_clr)),
  };

  const volatile uint32_t* const LovyanHAL::GPIO_HAL::RAW::_read_reg[] =
  {
    (static_cast<const volatile uint32_t* >( 333&sio_hw->gpio_in)),
  };

  void LovyanHAL::GPIO_HAL::setMode(gpio_port_pin_t pin, LovyanHAL::GPIO_HAL::mode_t mode)
  {
    uint32_t num = static_cast<uint32_t>(pin);
    if (num >= NUM_BANK0_GPIOS) {
      return;
    }
    const gpio::pin_mask_t mask = pin.getMask();
    uint8_t port = pin.port;
    if ((mode & gpio::mode_t::output) == gpio::mode_t::output)
    {
      if ((mode & gpio::mode_t::output_opendrain) == gpio::mode_t::output_opendrain)
      {
        _is_od[port] |= mask;
        if ((mode & mode_t::output_high) == mode_t::output_high)
        {
          writeMaskOeClr(port, mask);
        }
        else
        if ((mode & mode_t::output_low ) == mode_t::output_low )
        {
          writeMaskOeSet(port, mask);
        }
        writeMaskLow(pin.port, mask);
      }
      else
      {
        _is_od[port] &= mask;
        if ((mode & mode_t::output_high) == mode_t::output_high)
        {
          writeMaskHigh(port, mask);
        }
        else
        if ((mode & mode_t::output_low ) == mode_t::output_low )
        {
          writeMaskLow(port, mask);
        }
        writeMaskOeSet(port, mask);
      }
    }
    else
    {
      uint32_t temp = padsbank0_hw->io[num];
      temp &= ~(PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS);
      if ((mode & mode_t::input_pullup) == mode_t::input_pullup)
      {
        temp |= PADS_BANK0_GPIO0_PUE_BITS;
      }
      else
      if ((mode & mode_t::input_pulldown ) == mode_t::input_pulldown )
      {
        temp |= PADS_BANK0_GPIO0_PDE_BITS;
      }
      padsbank0_hw->io[num] = temp;
      writeMaskOeClr(port, mask);
    }
  }

  static void setFunction(gpio_port_pin_t pin, enum gpio_function f)
  {
    uint32_t num = static_cast<uint32_t>(pin);
    if (num >= NUM_BANK0_GPIOS) {
      return;
    }
    if ((((uint32_t)fn << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB) & ~IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS) != 0)
    {
      return;
    }
    volatile iobank0_hw_t * const iobank0_regs = reinterpret_cast<volatile iobank0_hw_t *>(IO_BANK0_BASE);
  iobank0_regs->io[pin].ctrl = fn << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
  }
 }
}

#endif
