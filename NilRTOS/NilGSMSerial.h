/* Arduino NilRTOS Library
 * Copyright (C) 2013 by William Greiman
 *
 * This file is part of the Arduino NilRTOS Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino NilRTOS Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
 /**
 * @file    NilGSMSerial.h
 * @brief   Nil RTOS serial library header.
 *
 * @defgroup Serial NilGSMSerial
 * @details Nil RTOS serial library.
 * @{
 */
#ifndef NilGSMSerial_h
#define NilGSMSerial_h
#include <Arduino.h>
/**
 * @class NilGSMSerialClass
 * @brief Mini serial class derived from the Arduino Print class.
 */
class NilGSMSerialClass : public Print {
 public:
  int available();
  void begin(unsigned long);
  int read();
  size_t write(uint8_t b);
  using Print::write;
};
#ifdef UDR0
extern NilGSMSerialClass NilGSMSerial;
#endif  // UDR0
#endif  // NilGSMSerial_h

/** @} */