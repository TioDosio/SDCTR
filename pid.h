#ifndef PID_H
#define PID_H
class pid
{
    float I, D, K, Ti, Td, b, h, N, Kold, bold, Tt;

public:
    explicit pid(float _h, float _K = 1, float b_ = 1, float Ti_ = 1, float Td_ = 0, float N_ = 10);

    ~pid(){};
    float compute_control(float r, float y, int anti_windup, float h);

    void housekeep(float r, float y, int anti_windup);
};
inline void pid::housekeep(float r, float y, int anti_windup)
{
    float e = r - y;
    if (anti_windup == 0)
    {
        I += K * h / Ti * e;
    }
}
#endif // PID_H