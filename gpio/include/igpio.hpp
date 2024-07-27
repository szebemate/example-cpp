// SPDX-FileCopyrightText: 2024 Ferenc Nandor Janky <ferenj@effective-range.com>
// SPDX-FileCopyrightText: 2024 Attila Gombos <attila.gombos@effective-range.com>
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <memory>

extern "C"{
typedef void (*gpioISRFunc_t)  (int gpio, int level, uint32_t tick);
}
struct IGPIO {
  using Ptr = std::shared_ptr<IGPIO>;

  struct Interrupted : std::exception {
    const char *what() const noexcept override { return "GPIO Interrupted"; }
  };

  enum class Modes {
    INPUT,
    OUTPUT,
    ALT0,
    ALT1,
    ALT2,
    ALT3,
    ALT4,
    ALT5,
    UNDEFINED
  };
  using port_id_t = unsigned;
  using val_t = unsigned;

  virtual void set_gpio_mode(port_id_t port, Modes mode) = 0;
  virtual void gpio_write(port_id_t gpio, val_t val) = 0;
  virtual val_t gpio_read(port_id_t gpio) = 0;
  virtual void gpio_set_isr_func(port_id_t gpio, unsigned edge, int timeout, gpioISRFunc_t f) =0;

  virtual void delay(std::chrono::microseconds) = 0;

  static Ptr Create();

  virtual ~IGPIO() = default;
};