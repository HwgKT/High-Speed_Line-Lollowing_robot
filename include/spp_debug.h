#pragma once
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "bt_telemetry.h"
// Low-volume SPP diagnostic replies only, never periodic USB output.
inline void sppLogLine(const char *line) { if(btConnected()) btSendLine(String(line)); }
inline void sppLogLine(const String &line) { if(btConnected()) btSendLine(line); }
inline void sppLogPrintf(const char *fmt, ...) {
  if(!btConnected()) return;
  char line[320];
  va_list args;
  va_start(args,fmt);
  vsnprintf(line,sizeof(line),fmt,args);
  va_end(args);
  size_t n=strlen(line);
  while(n && (line[n-1]=='\n'||line[n-1]=='\r')) line[--n]='\0';
  if(n) btSendLine(String(line));
}
