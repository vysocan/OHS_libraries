// A library for handling real-time clocks, dates, etc.
// 2010-02-04 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// 2012-11-08 RAM methods - idreammicro.com
// 2012-11-14 SQW/OUT methods - idreammicro.com
// 2012-01-12 DS1388 support
// 2013-08-29 ENERGIA MSP430 support

#include <TwiMaster.h>
//#include <Wire.h>
// Energia support
#ifndef ENERGIA
#include <avr/pgmspace.h>
#else
#define pgm_read_word(data) *data
#define pgm_read_byte(data) *data
#define PROGMEM
#endif
#include "RTClib.h"
#include <Arduino.h>

// Wire style address for DS1307
const int DS1307_ADDRESS = (0XD0 >>1);
//#define DS1307_ADDRESS          0x68
#define DS1307_CONTROL_REGISTER 0x07
#define DS1307_RAM_REGISTER     0x08

// DS1307 Control register bits.
#define RTC_DS1307__RS0         0x00
#define RTC_DS1307__RS1         0x01
#define RTC_DS1307__SQWE        0x04
#define RTC_DS1307__OUT         0x07

#define SECONDS_PER_DAY         86400L
#define SECONDS_IN_CENTURY      (3155760039UL)

////////////////////////////////////////////////////////////////////////////////
// utility code, some of this could be exposed in the DateTime API if needed

static const uint8_t daysInMonth [] PROGMEM = {
  31,28,31,30,31,30,31,31,30,31,30,31
};

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

static uint32_t time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

////////////////////////////////////////////////////////////////////////////////
// DateTime implementation - ignores time zones and DST changes
// NOTE: also ignores leap seconds, see http://en.wikipedia.org/wiki/Leap_second

DateTime::DateTime (uint32_t t) {
    //Serial.print("t:");
    //Serial.println(t);
    if (t >= SECONDS_IN_CENTURY) t = 0; // Safety belts
    ss = t % 60;
    t /= 60;
    mm = t % 60;
    t /= 60;
    hh = t % 24;
    //uint16_t days = t / 24;
    days = t / 24;
    uint8_t leap;
    for (yOff = 0; ; ++yOff) {
        leap = yOff % 4 == 0;
        if (days < 365 + leap)
            break;
        days -= 365 + leap;
    }
    for (m = 1; ; ++m) {
        uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
        if (leap && m == 2)
            ++daysPerMonth;
        if (days < daysPerMonth)
            break;
        days -= daysPerMonth;
    }
    d = days + 1;
}

DateTime::DateTime (uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    if (year >= 2000)
        year -= 2000;
    yOff = year;
    m = month;
    d = day;
    hh = hour;
    mm = min;
    ss = sec;
}

static uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

// A convenient constructor for using "the compiler's time":
//   DateTime now (__DATE__, __TIME__);
// NOTE: using PSTR would further reduce the RAM footprint
DateTime::DateTime (const char* date, const char* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    yOff = conv2d(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (date[0]) {
        case 'J': m = date[1] == 'a' ? 1 : m = date[2] == 'n' ? 6 : 7; break;
        case 'F': m = 2; break;
        case 'A': m = date[2] == 'r' ? 4 : 8; break;
        case 'M': m = date[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(date + 4);
    hh = conv2d(time);
    mm = conv2d(time + 3);
    ss = conv2d(time + 6);
}

uint8_t DateTime::dayOfWeek() const {
    uint16_t day = get() / SECONDS_PER_DAY;
    return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

uint32_t DateTime::get() const {
    uint16_t days = date2days(yOff, m, d);
    return time2long(days, hh, mm, ss);
}

void DateTime::set(uint32_t t) {
    //Serial.print("t:");
    //Serial.println(t);
    if (t >= SECONDS_IN_CENTURY) t = 0; // Safety belts
    ss = t % 60;
    t /= 60;
    mm = t % 60;
    t /= 60;
    hh = t % 24;
    //uint16_t days = t / 24;
    days = t / 24;
    uint8_t leap;
    for (yOff = 0; ; ++yOff) {
        leap = yOff % 4 == 0;
        if (days < 365 + leap)
            break;
        days -= 365 + leap;
    }
    for (m = 1; ; ++m) {
        uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
        if (leap && m == 2)
            ++daysPerMonth;
        if (days < daysPerMonth)
            break;
        days -= daysPerMonth;
    }
    d = days + 1;
}

//
char* DateTime::timestamp() {
  char _tmp_itoa[5];
  _tmp_ts[0] = 0;
  
  if (yOff < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(yOff, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);

  if (m < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(m, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  
  if (d < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(d, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  
  if (hh < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(hh, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  if (mm < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(mm, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  if (ss < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(ss, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  
  return _tmp_ts;  
}

//
char* DateTime::formatedDateTime() {
  char _tmp_itoa[5];
  _tmp_ts[0] = 0;
  
  if (d < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(d, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  strcat_P (_tmp_ts, RTC_text_dot);
  if (m < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(m, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  strcat_P (_tmp_ts, RTC_text_dot_tt);
  if (yOff < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(yOff, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  strcat_P (_tmp_ts, RTC_text_space);
  if (hh < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(hh, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  strcat_P (_tmp_ts, RTC_text_semicol);
  if (mm < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(mm, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  strcat_P (_tmp_ts, RTC_text_semicol);
  if (ss < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(ss, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  
  return _tmp_ts;  
}

//
char* DateTime::formatedUpTime() {
  char _tmp_itoa[5];
  _tmp_ts[0] = 0;
  
  uint16_t days = date2days(yOff, m, d);
  itoa(days, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  strcat_P (_tmp_ts, RTC_text_days);
  
  if (hh < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(hh, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  strcat_P (_tmp_ts, RTC_text_semicol);
  if (mm < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(mm, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  strcat_P (_tmp_ts, RTC_text_semicol);
  if (ss < 10) strcat_P (_tmp_ts, RTC_text_0);
  itoa(ss, _tmp_itoa, 10);
  strcat (_tmp_ts, _tmp_itoa);
  
  return _tmp_ts;  
}

////////////////////////////////////////////////////////////////////////////////
// RTC_DS1307 implementation

void RTC_DS1307::adjust(const DateTime& dt) {
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write((byte) 0);
    Wire.write(bin2bcd(dt.second()));
    Wire.write(bin2bcd(dt.minute()));
    Wire.write(bin2bcd(dt.hour()));
    Wire.write(bin2bcd(0));
    Wire.write(bin2bcd(dt.day()));
    Wire.write(bin2bcd(dt.month()));
    Wire.write(bin2bcd(dt.year() - 2000));
    Wire.write((byte) 0);
    Wire.endTransmission();
}

DateTime RTC_DS1307::now() {
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write((byte) 0);
    Wire.endTransmission();

    Wire.requestFrom(DS1307_ADDRESS, 7);
    uint8_t ss = bcd2bin(Wire.read());
    uint8_t mm = bcd2bin(Wire.read());
    uint8_t hh = bcd2bin(Wire.read());
    Wire.read();
    uint8_t d = bcd2bin(Wire.read());
    uint8_t m = bcd2bin(Wire.read());
    uint16_t y = bcd2bin(Wire.read()) + 2000;

    return DateTime (y, m, d, hh, mm, ss);
}

void RTC_DS1307::setSqwOutLevel(uint8_t level) {
    uint8_t value = (level == LOW) ? 0x00 : (1 << RTC_DS1307__OUT);
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(DS1307_CONTROL_REGISTER);
    Wire.write(value);
    Wire.endTransmission();
}

void RTC_DS1307::setSqwOutSignal(Frequencies frequency) {
    uint8_t value = (1 << RTC_DS1307__SQWE);
    switch (frequency)
    {
        case Frequency_1Hz:
            // Nothing to do.
        break;
        case Frequency_4096Hz:
            value |= (1 << RTC_DS1307__RS0);
        break;
        case Frequency_8192Hz:
            value |= (1 << RTC_DS1307__RS1);
        break;
        case Frequency_32768Hz:
        default:
            value |= (1 << RTC_DS1307__RS1) | (1 << RTC_DS1307__RS0);
        break;
    }
    Wire.beginTransmission(DS1307_ADDRESS);
  	Wire.write(DS1307_CONTROL_REGISTER);
  	Wire.write(value);
    Wire.endTransmission();
}

uint8_t RTC_DS1307::readByteInRam(uint8_t address) {
    Wire.beginTransmission(DS1307_ADDRESS);
  	Wire.write(address);
    Wire.endTransmission();

    Wire.requestFrom(DS1307_ADDRESS, 1);
    uint8_t data = Wire.read();
    Wire.endTransmission();

    return data;
}

void RTC_DS1307::readBytesInRam(uint8_t address, uint8_t length, uint8_t* p_data) {
    Wire.beginTransmission(DS1307_ADDRESS);
  	Wire.write(address);
    Wire.endTransmission();

    Wire.requestFrom(DS1307_ADDRESS, (int)length);
    for (uint8_t i = 0; i < length; i++) {
        p_data[i] = Wire.read();
    }
    Wire.endTransmission();
}

void RTC_DS1307::writeByteInRam(uint8_t address, uint8_t data) {
    Wire.beginTransmission(DS1307_ADDRESS);
  	Wire.write(address);
  	Wire.write(data);
    Wire.endTransmission();
}

void RTC_DS1307::writeBytesInRam(uint8_t address, uint8_t length, uint8_t* p_data) {
    Wire.beginTransmission(DS1307_ADDRESS);
  	Wire.write(address);
  	for (uint8_t i = 0; i < length; i++) {
  	  	Wire.write(p_data[i]);
  	}
    Wire.endTransmission();
}

uint8_t RTC_DS1307::isrunning(void) {
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write((byte) 0);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 1);
  uint8_t ss = Wire.read();
  return !(ss>>7);
}
