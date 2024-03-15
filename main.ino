#include "pid.h"

float K = 15, B, Ti, Td = 0;
const float m = -0.8, b = 6, h = 0.01;
const int LED_PIN = 15, ADC_PIN = A0, DAC_RANGE = 4096, R = 10e3;
pid my_pid{h, K, B, Ti};                           // Create a pid controller
unsigned long previousTime = 0, sampInterval = 10; // milliseconds
double ganho, duty_cycle = 0;
float H = 0, y = 0, u = 0, r{10}, run_time = 0, run_init = 0, energy = 0, power_max = ((0.29 * 0.29) / 47) + (0.29 * 2.63), vcc = 3.3;
int occupied = 1, anti_windup = 1, feedback = 1, flag_stream_lux = 0, flag_stream_duty_c = 0, Led = 1;
void setup()
{
  analogReadResolution(12);
  analogWriteFreq(30000); // 30KHz
  analogWriteRange(4095); // Max PWM // pid*4095  
  Serial.begin();

  ganho = calibration();
  Serial.print("Calibration Completed, gain = ");
  Serial.println(ganho);
  float resistance = pow(10, (m * log10(r) + b));
  Ti = 10e-6 * ((10000 * resistance) / (10000 + resistance));
}

void loop()
{
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= sampInterval)
  {
    if (Serial.available())
    {
      String command = Serial.readStringUntil('\n');
      processCommand(command);
    }
    /*if (occupied == 1 and r < 2) // meter bem
    {
      r = 15;
    }
    else if (occupied == 0 and r >= 8)
    {
      r = 5;
    }*/
    float adc_value = analogRead(ADC_PIN);                                                                              // read adc value
    y = dac_lux(adc_value);                                                                                             // convert adc value to lux
    saveToBuffer_lux(y);                                                                                                // save lux value to buffer
    u = my_pid.compute_control(K, B, Ti*0.5, Td, lux_volt(r, b, m, vcc), lux_volt(y, b, m, vcc), anti_windup, h, feedback); // compute control
    u *= 4095;
    duty_cycle = u / DAC_RANGE;
    saveToBuffer_dc(duty_cycle); // save duty cycle value to buffer
    int pwm = (int)u;
    analogWrite(LED_PIN, pwm); // write pwm value to led
    if (feedback == 1)
    {
      my_pid.housekeep(lux_volt(r, b, m, vcc), lux_volt(y, b, m, vcc), anti_windup);
    }

    if (flag_stream_lux != 0) // strem lux values of the desk
    {
      Serial.print(0);
      Serial.print(" ");
      Serial.print(30);
      Serial.print(" ");
      Serial.print(r);
      Serial.print(" ");
      Serial.print(pwm);
      Serial.print(" ");
      /*Serial.print("s");
      Serial.print(" ");
      Serial.print("l");
      Serial.print(" ");*/
      Serial.print(y);
      Serial.print(" ");
      /*Serial.print(currentTime);
      Serial.println();
      Serial.print(" ");*/
      Serial.print(duty_cycle);
      Serial.print(" ");
      Serial.print(B);
      Serial.print(" ");
      Serial.print(K);
      Serial.print(" ");
      Serial.println(Ti);
    }
    if (flag_stream_duty_c != 0) // stream duty cycle values of the desk
    {
      Serial.print("s");
      Serial.print(" ");
      Serial.print("d");
      Serial.print(" ");
      Serial.print(duty_cycle);
      Serial.print(" ");
      Serial.print(currentTime);
      Serial.println();
    }
    average_visibility_error(r, y);
    average_flicker_error();
    previousTime = currentTime;
  }
}
