// SPDX-FileCopyrightText: 2024 Ferenc Nandor Janky <ferenj@effective-range.com>
// SPDX-FileCopyrightText: 2024 Attila Gombos
// <attila.gombos@effective-range.com> SPDX-License-Identifier: MIT

#include <iostream>

#include <chrono>
#include <ostream>
#include <thread>

#include <fmt/format.h>
#include <igpio.hpp>

extern "C" {
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/reboot.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
}
template <typename F> struct [[nodiscard]] Finally {
  F f;
  ~Finally() { f(); }
};
extern "C" {}
extern "C" {
int i2c_read_reg(__u8 reg_addr) {
  int file;
  int adapter_nr = 1; /* probably dynamically determined */
  char filename[20];

  snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
  file = open(filename, O_RDWR);
  if (file < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    return -1;
  }
  int addr = 0x33; /* The I2C address */

  if (ioctl(file, I2C_SLAVE, addr) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    std::cout << "Err open" << std::endl;
    close(file);
    return -1;
  }

  __s32 res;

  res = i2c_smbus_read_byte_data(file, reg_addr);
  if (res < 0) {
    std::cout << "Err read" << res << std::endl;
  } else {
    std::cout << "OK read" << res << std::endl;
  }
  close(file);
  return res;
}
int i2c_write_reg(__u8 reg_addr, __u8 value) {
  int file;
  int adapter_nr = 1; /* probably dynamically determined */
  char filename[20];

  snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
  file = open(filename, O_RDWR);
  if (file < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    return -1;
  }
  int addr = 0x33; /* The I2C address */

  if (ioctl(file, I2C_SLAVE, addr) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    std::cout << "Err open" << std::endl;
    close(file);
    return -1;
  }

  __s32 res;

  res = i2c_smbus_write_byte_data(file, reg_addr, value);
  if (res < 0) {
    std::cout << "Err write" << res << std::endl;

  } else {
    std::cout << "OK write" << res << std::endl;
  }
  close(file);
  return res;
}
std::atomic_bool falling_edge;
void ISRCallback(int gpio, int level, uint32_t tick) {
  std::cout << "ISRCallback gpio" << gpio << " level:" << level
            << " tick:" << tick << std::endl;
  falling_edge = true;
}
}

#define MCU_INT_PIN 27
#define PI_RUN_PIN 25
#define PIC_SHUTDOWN_REG_ADDR 0x4
#define PIC_SHUTDOWN_ACKED_REG_ADDR 0x5
#define PI_SHUDTOWN_REQ_VAL 0x55
#define PI_SHUDTOWN_ACK_VAL 0xAA
int main() {

  bool shutdown_requested = false;
  std::chrono::steady_clock::time_point shutdown_begin;
  std::cout << "HELLO" << std::endl;
  auto gpio = IGPIO::Create();


  // std::cout << "HELLODs" << std::endl;
  // gpio->set_gpio_mode(16, IGPIO::Modes::OUTPUT);
  // Finally restore_gpio{[&] { gpio->set_gpio_mode(16, IGPIO::Modes::INPUT);
  // }}; gpio->gpio_write(16, 0); gpio->gpio_write(16, 1);
  // gpio->delay(std::chrono::seconds{5});
  // gpio->gpio_write(16, 0);

  // PI RUN pin
  gpio->set_gpio_mode(PI_RUN_PIN, IGPIO::Modes::OUTPUT);
  gpio->gpio_write(PI_RUN_PIN, 1);

  // init MCU_INT pin
  gpio->set_gpio_mode(MCU_INT_PIN, IGPIO::Modes::INPUT);

  // set interrupt pin
  gpio->gpio_set_isr_func(MCU_INT_PIN, 0, 0, &ISRCallback);
  auto mcu_int = gpio->gpio_read(MCU_INT_PIN);
  auto mcu_prev = mcu_int;

  std::cout << "Startin wit" << mcu_int << std::endl;
  while (1) {
    //toggle PI_RUN pin
    auto act = gpio->gpio_read(PI_RUN_PIN);
    auto act2 = !act;
    gpio->gpio_write(PI_RUN_PIN, act2);

    
    if (falling_edge && (shutdown_requested == false)) {
      std::cout << "MCU INT PIN falling_edge atomic bool" << std::endl;
      falling_edge = false;

      // read shutdown reg
      auto res = i2c_read_reg(PIC_SHUTDOWN_REG_ADDR);
      if (res == PI_SHUDTOWN_REQ_VAL) {
        std::cout << "SHUTDOWN requested" << std::endl;
        shutdown_requested = true;
        shutdown_begin = std::chrono::steady_clock::now();

        // acknowledge it
        i2c_write_reg(PIC_SHUTDOWN_ACKED_REG_ADDR, PI_SHUDTOWN_ACK_VAL);
        std::cout << "Start shutdown" << std::endl;

        gpio->delay(std::chrono::milliseconds{300});
        auto reboot_res = reboot(RB_POWER_OFF);
        std::cout << "REBOOT res" << reboot_res << std::endl;
      } else {
        std::cout << "ERR not set reg" << std::endl;
      }
    }

    gpio->delay(std::chrono::milliseconds{300});
    //just for test
    auto reg = i2c_read_reg(PIC_SHUTDOWN_REG_ADDR);
    std::cout << "REG " << reg << std::endl;
  }
}