#include "pid.h"

const float m = -0.8, b = 6.15, h = 0.01;
const int LED_PIN = 15, ADC_PIN = A0, DAC_RANGE = 4096, R = 10e3;
pid my_pid{h, 100, 0.1, 2};                        // Create a pid controller // pid my_pid{10, 3, 10, 7};
unsigned long previousTime = 0, sampInterval = 10; // milliseconds
double box_light, duty_cycle = 0;
int Led = 1;
float y = 0, u = 0, r{10}, external_lux = 0, run_time = 0, run_init = 0, energy = 0, power_max = ((0.28 * 0.28) / 47) + 100e-3, vcc = 3.3; // P = V^2 / R = 0.28^2 / 47
int occupied = 1, anti_windup = 1, feedback = 1, flag_stream_lux = 1, flag_stream_duty_c = 0;

void setup()
{
  analogReadResolution(12);
  analogWriteFreq(30000); // 30KHz
  analogWriteRange(4095); // Max PWM
  Serial.begin();

  box_light = calibration();
  Serial.print("Calibration Completed, box light= ");
  Serial.println(box_light);
  delay(1000);
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
    if (occupied == 1 and r < 2)
    {
      r = 15;
    }
    else if (occupied == 0 and r >= 8)
    {
      r = 5;
    }
    float adc_value = analogRead(ADC_PIN);
    y = dac_lux(adc_value);
    saveToBuffer_lux(y);
    if (feedback == 1)
    {
      u = my_pid.compute_control(lux_volt(r, b, m, vcc), lux_volt(mean_lux(), b, m, vcc), anti_windup, h);
    }
    else
    {
      u = duty_cycle * DAC_RANGE;
    }
    duty_cycle = u / DAC_RANGE;
    saveToBuffer_dc(duty_cycle);
    int pwm = (int)u;
    analogWrite(LED_PIN, pwm);
    my_pid.housekeep(r, y, anti_windup);

    if (flag_stream_lux != 0) // Stream the led values (need to be implemented sending messages to other desks)
    {
      Serial.print(0);
      Serial.print(" ");
      Serial.print(30);
      Serial.print(" ");
      Serial.print(r);
      Serial.print(" ");
      Serial.print(pwm);
      Serial.print(" ");
      Serial.print("s");
      Serial.print(" ");
      Serial.print("l");
      Serial.print(" ");
      Serial.print(y);
      Serial.print(" ");
      Serial.print(currentTime);
      Serial.println();
    }
    if (flag_stream_duty_c != 0)
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
