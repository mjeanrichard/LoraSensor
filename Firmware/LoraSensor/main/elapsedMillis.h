/* Elapsed time types - for easy-to-use measurements of elapsed time
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2011 PJRC.COM, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef elapsedMillis_h
#define elapsedMillis_h
#ifdef __cplusplus

#include "esp_timer.h"

class elapsedMicros
{
private:
    unsigned long us;
public:
    elapsedMicros(void) { us = esp_timer_get_time(); }
    elapsedMicros(unsigned long val) { us = esp_timer_get_time() - val; }
    elapsedMicros(const elapsedMicros& orig) { us = orig.us; }
    operator unsigned long() const { return esp_timer_get_time() - us; }
    elapsedMicros& operator = (const elapsedMicros& rhs) { us = rhs.us; return *this; }
    elapsedMicros& operator = (unsigned long val) { us = esp_timer_get_time() - val; return *this; }
    elapsedMicros& operator -= (unsigned long val) { us += val; return *this; }
    elapsedMicros& operator += (unsigned long val) { us -= val; return *this; }
    elapsedMicros operator - (int val) const { elapsedMicros r(*this); r.us += val; return r; }
    elapsedMicros operator - (unsigned int val) const { elapsedMicros r(*this); r.us += val; return r; }
    elapsedMicros operator - (long val) const { elapsedMicros r(*this); r.us += val; return r; }
    elapsedMicros operator - (unsigned long val) const { elapsedMicros r(*this); r.us += val; return r; }
    elapsedMicros operator + (int val) const { elapsedMicros r(*this); r.us -= val; return r; }
    elapsedMicros operator + (unsigned int val) const { elapsedMicros r(*this); r.us -= val; return r; }
    elapsedMicros operator + (long val) const { elapsedMicros r(*this); r.us -= val; return r; }
    elapsedMicros operator + (unsigned long val) const { elapsedMicros r(*this); r.us -= val; return r; }
};

class elapsedMillis
{
private:
    unsigned long ms;
public:
    elapsedMillis(void) { ms = (unsigned long)(esp_timer_get_time() / 1000ULL); }
    elapsedMillis(unsigned long val) { ms = (unsigned long)(esp_timer_get_time() / 1000ULL) - val; }
    elapsedMillis(const elapsedMillis& orig) { ms = orig.ms; }
    operator unsigned long() const { return (unsigned long)(esp_timer_get_time() / 1000ULL) - ms; }
    elapsedMillis& operator = (const elapsedMillis& rhs) { ms = rhs.ms; return *this; }
    elapsedMillis& operator = (unsigned long val) { ms = (unsigned long)(esp_timer_get_time() / 1000ULL) - val; return *this; }
    elapsedMillis& operator -= (unsigned long val) { ms += val; return *this; }
    elapsedMillis& operator += (unsigned long val) { ms -= val; return *this; }
    elapsedMillis operator - (int val) const { elapsedMillis r(*this); r.ms += val; return r; }
    elapsedMillis operator - (unsigned int val) const { elapsedMillis r(*this); r.ms += val; return r; }
    elapsedMillis operator - (long val) const { elapsedMillis r(*this); r.ms += val; return r; }
    elapsedMillis operator - (unsigned long val) const { elapsedMillis r(*this); r.ms += val; return r; }
    elapsedMillis operator + (int val) const { elapsedMillis r(*this); r.ms -= val; return r; }
    elapsedMillis operator + (unsigned int val) const { elapsedMillis r(*this); r.ms -= val; return r; }
    elapsedMillis operator + (long val) const { elapsedMillis r(*this); r.ms -= val; return r; }
    elapsedMillis operator + (unsigned long val) const { elapsedMillis r(*this); r.ms -= val; return r; }
};

class elapsedSeconds
{
private:
    unsigned long s;
public:
    elapsedSeconds(void) { s = esp_timer_get_time() / 1000000ULL; }
    elapsedSeconds(unsigned long val) { s = esp_timer_get_time() / 1000000ULL - val; }
    elapsedSeconds(const elapsedSeconds& orig) { s = orig.s; }
    operator unsigned long() const { return esp_timer_get_time() / 1000000ULL - s; }
    elapsedSeconds& operator = (const elapsedSeconds& rhs) { s = rhs.s; return *this; }
    elapsedSeconds& operator = (unsigned long val) { s = esp_timer_get_time() / 1000000ULL - val; return *this; }
    elapsedSeconds& operator -= (unsigned long val) { s += val; return *this; }
    elapsedSeconds& operator += (unsigned long val) { s -= val; return *this; }
    elapsedSeconds operator - (int val) const { elapsedSeconds r(*this); r.s += val; return r; }
    elapsedSeconds operator - (unsigned int val) const { elapsedSeconds r(*this); r.s += val; return r; }
    elapsedSeconds operator - (long val) const { elapsedSeconds r(*this); r.s += val; return r; }
    elapsedSeconds operator - (unsigned long val) const { elapsedSeconds r(*this); r.s += val; return r; }
    elapsedSeconds operator + (int val) const { elapsedSeconds r(*this); r.s -= val; return r; }
    elapsedSeconds operator + (unsigned int val) const { elapsedSeconds r(*this); r.s -= val; return r; }
    elapsedSeconds operator + (long val) const { elapsedSeconds r(*this); r.s -= val; return r; }
    elapsedSeconds operator + (unsigned long val) const { elapsedSeconds r(*this); r.s -= val; return r; }
};

#endif // __cplusplus
#endif // elapsedMillis_h
