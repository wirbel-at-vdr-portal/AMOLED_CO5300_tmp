#pragma once

#define DEBUG_FUNC
#define DEBUG_ARG

#define xstr(a) str(a)
#define str(a) #a

#ifdef DEBUG_FUNC
#define dbg_func do { \
  /*Serial.print(__FILE__);   Serial.print(":");  Serial.print(__LINE__);   Serial.print(" "); */  Serial.println(__PRETTY_FUNCTION__); \
  } while(0)
#else
#define dbg_func
#endif

#ifdef DEBUG_ARG
#define dbg_arg(Name) do { \
  Serial.print( xstr(Name) ); Serial.print(" = 0x"); Serial.println(Name,HEX); \
  } while(0)
#else
#define dbg_func
#endif

