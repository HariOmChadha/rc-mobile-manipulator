#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>

// Manual behavior is preserved until an explicit host ARM command.
struct RemoteControl {
  enum Mode { MANUAL, AUTO, HOLD } mode = MANUAL;
  uint32_t session = 0, lastSeq = 0, lastCommand = 0;
  bool hasSeq = false;
  int steer = 0, throttle = 50, neutralSteer = 0, neutralThrottle = 50;
  static constexpr uint32_t timeoutMs = 350;
  static bool valid(int s, int t) { return s >= 0 && s <= 255 && t >= 50 && t <= 145; }
  void stop() { steer = neutralSteer; throttle = neutralThrottle; mode = HOLD; session = 0; }
  void tick(uint32_t now) {
    if (mode == AUTO && uint32_t(now - lastCommand) >= timeoutMs) stop();
  }
  void command(const char* line, uint32_t now, char* reply, size_t size) {
    tick(now);
    unsigned long id = 0, seq = 0;
    int s = 0, t = 0;
    char extra = 0;
    std::snprintf(reply, size, "ERR");
    if (!std::strcmp(line, "HELLO")) {
      std::snprintf(reply, size, "RC_READY,1");
    } else if (!std::strcmp(line, "MANUAL")) {
      mode = MANUAL; session = 0;
      std::snprintf(reply, size, "ACK,MANUAL");
    } else if (std::sscanf(line, "ARM,%lu,%d,%d%c", &id, &s, &t, &extra) == 3 &&
               id > 0 && id <= 2147483647UL && valid(s,t) && mode != AUTO) {
      session = uint32_t(id); neutralSteer = steer = s; neutralThrottle = throttle = t;
      mode = AUTO; hasSeq = false; lastCommand = now;
      std::snprintf(reply, size, "ACK,ARM,%lu", id);
    } else if (std::sscanf(line, "DRIVE,%lu,%lu,%d,%d%c", &id, &seq, &s, &t, &extra) == 4 &&
               mode == AUTO && id == session && seq <= 2147483647UL &&
               (!hasSeq || seq > lastSeq) && valid(s,t)) {
      steer = s; throttle = t; lastSeq = uint32_t(seq); hasSeq = true; lastCommand = now;
      std::snprintf(reply, size, "ACK,DRIVE,%lu", seq);
    } else if (std::sscanf(line, "STOP,%lu%c", &id, &extra) == 1 && mode == AUTO && id == session) {
      stop(); std::snprintf(reply, size, "ACK,STOP,%lu", id);
    }
  }
};
