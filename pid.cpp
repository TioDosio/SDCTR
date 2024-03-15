#include "pid.h"
pid::pid(float _h, float _K, float b_,
         float Ti_, float Td_, float N_)
    // member variable initialization list
    : h{_h}, K{_K}, b{b_}, Ti{Ti_}, Td{Td_},
      N{N_}, I{0.0}, D{0.0}, Tt{0.1}
{
}

float pid::compute_control(float K, float b, float Ti, float Td, float r, float y, int anti_windup, float h, int feedback)
{
  if (feedback == 0)
  {
    float u = K * b * r;
    if (u < 0)
      u = 0;
    else if (u > 4095)
      u = 4095;
    return u;
  }
  else
  {
    D = 0;
    I = I + Kold * (bold * r - y) - K * (b * r - y); // bumpless
    float P = K * (b * r - y);
    float v = P + I + D;
    float u = v;
    if (u < 0) // Saturation
      u = 0;
    else if (u > 1)
      u = 1;
    if (anti_windup == 1)
    {
      float bi = K * h / Ti; // integral gain
      float ao = h / Tt;
      I = I + bi * (r - y) + ao * (u - v); // Anti-windup
    }
    else{
    float e = r - y;
        I += K * h / Ti * e;
    }
    Kold = K;
    bold = b;
    return u;
  }
}
