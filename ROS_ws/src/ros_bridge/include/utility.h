#ifndef SPRINTER_UTILITY_H
#define SPRINTER_UTILITY_H
#include <cmath>
namespace ROSbridge
{
typedef uint8_t* bytePtr;
typedef const uint8_t* constBytePtr;

template <class T, class U>
static constexpr inline T deg2rad(U degrees)
{
  return degrees * (M_PI / 180);
}

template <class T, class U>
static constexpr inline T rad2deg(U radians)
{
  return radians * (180 / M_PI);
}
}

#endif  // SPRINTER_UTILITY_H