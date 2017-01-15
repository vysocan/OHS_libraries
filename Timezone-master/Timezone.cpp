/*----------------------------------------------------------------------*
 * Arduino Timezone Library v1.0                                        *
 * Jack Christensen Mar 2012                                            *
 *                                                                      *
 * This work is licensed under the Creative Commons Attribution-        *
 * ShareAlike 3.0 Unported License. To view a copy of this license,     *
 * visit http://creativecommons.org/licenses/by-sa/3.0/ or send a       *
 * letter to Creative Commons, 171 Second Street, Suite 300,            *
 * San Francisco, California, 94105, USA.                               *
 *----------------------------------------------------------------------*/

#include "Timezone.h"

#define SECS_PER_MIN 60
#define SECS_PER_DAY 86400

#ifdef __AVR__
	#include <avr/eeprom.h>
#endif

/*----------------------------------------------------------------------*
 * Create a Timezone object from the given time change rules.           *
 *----------------------------------------------------------------------*/
Timezone::Timezone(TimeChangeRule dstStart, TimeChangeRule stdStart)
{
    _dst = dstStart;
    _std = stdStart;
}

#ifdef __AVR__
/*----------------------------------------------------------------------*
 * Create a Timezone object from time change rules stored in EEPROM     *
 * at the given address.                                                *
 *----------------------------------------------------------------------*/
Timezone::Timezone(int address)
{
    readRules(address);
}
#endif

/*----------------------------------------------------------------------*
 * Convert the given UTC time to local time, standard or                *
 * daylight time, as appropriate.                                       *
 *----------------------------------------------------------------------*/
DateTime Timezone::toLocal(DateTime utc)
{
    //recalculate the time change points if needed
    if (utc.year() != _dstUTC.year()) calcTimeChanges(utc.year());

    if (utcIsDST(utc))
        return utc.get() + _dst.offset * SECS_PER_MIN;
    else
        return utc.get() + _std.offset * SECS_PER_MIN;
}

/*----------------------------------------------------------------------*
 * Convert the given UTC time to local time, standard or                *
 * daylight time, as appropriate, and return a pointer to the time      *
 * change rule used to do the conversion. The caller must take care     *
 * not to alter this rule.                                              *
 *----------------------------------------------------------------------*/
DateTime Timezone::toLocal(DateTime utc, TimeChangeRule **tcr)
{
    //recalculate the time change points if needed
    if (utc.year() != _dstUTC.year()) calcTimeChanges(utc.year());

    if (utcIsDST(utc)) {
        *tcr = &_dst;
        return utc.get() + _dst.offset * SECS_PER_MIN;
    }
    else {
        *tcr = &_std;
        return utc.get() + _std.offset * SECS_PER_MIN;
    }
}

/*----------------------------------------------------------------------*
 * Convert the given local time to UTC time.                            *
 *                                                                      *
 * WARNING:                                                             *
 * This function is provided for completeness, but should seldom be     *
 * needed and should be used sparingly and carefully.                   *
 *                                                                      *
 * Ambiguous situations occur after the Standard-to-DST and the         *
 * DST-to-Standard time transitions. When changing to DST, there is     *
 * one hour of local time that does not exist, since the clock moves    *
 * forward one hour. Similarly, when changing to standard time, there   *
 * is one hour of local times that occur twice since the clock moves    *
 * back one hour.                                                       *
 *                                                                      *
 * This function does not test whether it is passed an erroneous time   *
 * value during the Local -> DST transition that does not exist.        *
 * If passed such a time, an incorrect UTC time value will be returned. *
 *                                                                      *
 * If passed a local time value during the DST -> Local transition      *
 * that occurs twice, it will be treated as the earlier time, i.e.      *
 * the time that occurs before the transistion.                         *
 *                                                                      *
 * Calling this function with local times during a transition interval  *
 * should be avoided!                                                   *
 *----------------------------------------------------------------------*/
DateTime Timezone::toUTC(DateTime local)
{
    //recalculate the time change points if needed
    if (local.year() != _dstLoc.year()) calcTimeChanges(local.year());

    if (locIsDST(local))
        return local.get() - _dst.offset * SECS_PER_MIN;
    else
        return local.get() - _std.offset * SECS_PER_MIN;
}

/*----------------------------------------------------------------------*
 * Determine whether the given UTC DateTime is within the DST interval    *
 * or the Standard time interval.                                       *
 *----------------------------------------------------------------------*/
boolean Timezone::utcIsDST(DateTime utc)
{
    //recalculate the time change points if needed
    if (utc.year() != _dstUTC.year()) calcTimeChanges(utc.year());

    if (_stdUTC.get() > _dstUTC.get())    //northern hemisphere
        return (utc.get() >= _dstUTC.get() && utc.get() < _stdUTC.get());
    else                      //southern hemisphere
        return !(utc.get() >= _stdUTC.get() && utc.get() < _dstUTC.get());
}

/*----------------------------------------------------------------------*
 * Determine whether the given Local DateTime is within the DST interval  *
 * or the Standard time interval.                                       *
 *----------------------------------------------------------------------*/
boolean Timezone::locIsDST(DateTime local)
{
    //recalculate the time change points if needed
    if (local.year() != _dstLoc.year()) calcTimeChanges(local.year());

    if (_stdLoc.get() > _dstLoc.get())    //northern hemisphere
        return (local.get() >= _dstLoc.get() && local.get() < _stdLoc.get());
    else                      //southern hemisphere
        return !(local.get() >= _stdLoc.get() && local.get() < _dstLoc.get());
}

/*----------------------------------------------------------------------*
 * Calculate the DST and standard time change points for the given      *
 * given year as local and UTC DateTime values.                           *
 *----------------------------------------------------------------------*/
void Timezone::calcTimeChanges(int yr)
{
    _dstLoc = toDateTime(_dst, yr);
    _stdLoc = toDateTime(_std, yr);
    _dstUTC = _dstLoc.get() - _std.offset * SECS_PER_MIN;
    _stdUTC = _stdLoc.get() - _dst.offset * SECS_PER_MIN;
}

/*----------------------------------------------------------------------*
 * Convert the given DST change rule to a DateTime value                  *
 * for the given year.                                                  *
 *----------------------------------------------------------------------*/
DateTime Timezone::toDateTime(TimeChangeRule r, int yr)
{
    uint8_t _yr, _m, _d, _hh;
    DateTime t;
    uint8_t m, w;            //temp copies of r.month and r.week

    m = r.month;
    w = r.week;
    if (w == 0) {            //Last week = 0
        if (++m > 12) {      //for "Last", go to the next month
            m = 1;
            yr++;
        }
        w = 1;               //and treat as first week of next month, subtract 7 days later
    }

    t=(yr - 1970, m, 1, r.hour);       //first day of the month, or first day of next month for "Last" rules
    t = t.get() + (7 * (w - 1) + (r.dow - t.dayOfWeek()+1 + 7) % 7) * SECS_PER_DAY;
    if (r.week == 0) t = t.get() - (7 * SECS_PER_DAY);    //back up a week if this is a "Last" rule
    return t;
}

#ifdef __AVR__
/*----------------------------------------------------------------------*
 * Read the daylight and standard time rules from EEPROM at				*
 * the given address.                                                   *
 *----------------------------------------------------------------------*/
void Timezone::readRules(int address)
{
    eeprom_read_block((void *) &_dst, (void *) address, sizeof(_dst));
    address += sizeof(_dst);
    eeprom_read_block((void *) &_std, (void *) address, sizeof(_std));
}

/*----------------------------------------------------------------------*
 * Write the daylight and standard time rules to EEPROM at				*
 * the given address.                                                   *
 *----------------------------------------------------------------------*/
void Timezone::writeRules(int address)
{
    eeprom_write_block((void *) &_dst, (void *) address, sizeof(_dst));
    address += sizeof(_dst);
    eeprom_write_block((void *) &_std, (void *) address, sizeof(_std));
}

#endif