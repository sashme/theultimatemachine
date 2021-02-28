#pragma once

namespace Debug
{

char buf[256];

#define DEBUG( params... ) \
  snprintf( Debug::buf, sizeof( Debug::buf ), params ); \
  Serial.println( Debug::buf )

}
