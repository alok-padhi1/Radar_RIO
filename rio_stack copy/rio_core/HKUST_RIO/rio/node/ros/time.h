#pragma once
#include <cstdint>
namespace ros {
  struct Time {
    uint32_t sec;
    uint32_t nsec;
    Time() : sec(0), nsec(0) {}
    Time(uint32_t s, uint32_t ns) : sec(s), nsec(ns) {}
    Time(double t) {
      sec = (uint32_t)t;
      nsec = (uint32_t)((t - sec) * 1e9);
    }
    double toSec() const { return sec + nsec * 1e-9; }
    bool operator==(const Time& rhs) const { return sec == rhs.sec && nsec == rhs.nsec; }
    bool operator!=(const Time& rhs) const { return !(*this == rhs); }
    bool operator<(const Time& rhs) const {
      if (sec < rhs.sec) return true;
      if (sec == rhs.sec && nsec < rhs.nsec) return true;
      return false;
    }
    bool operator<=(const Time& rhs) const { return *this < rhs || *this == rhs; }
    bool operator>(const Time& rhs) const { return !(*this <= rhs); }
    bool operator>=(const Time& rhs) const { return !(*this < rhs); }
    Time operator-(const Time& rhs) const {
      return Time(toSec() - rhs.toSec());
    }
  };
}
