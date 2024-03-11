#include "pid.h"
pid::pid(float _h, float _K, float b_,
         float Ti_, float Td_, float N_)
    // member variable initialization list
    : h{_h}, K{_K}, b{b_}, Ti{Ti_}, Td{Td_},
      N{N_}, I{0.0}, D{0.0}, Tt{2}
{
}

float pid::compute_control(float r, float y, int anti_windup, float h)
{
  D = 0;
  float bi = K * h / Ti; // integral gain
  float ao = h / Tt;
  I = I + Kold * (bold * r - y) - K * (b * r - y); // bumpless
  float P = K * (b * r - y);
  float v = P + I + D;
  float u = v;
  if (u < 0)
    u = 0;
  if (u > 4095)
    u = 4095;
  if (anti_windup == 1)
  {
    I = I + bi * (r - y) + ao * (u - v); // Anti-windup
  }
  Kold = K;
  bold = b;
  return u;
}
