
struct info
{
    double lux;
    double d_cycle;
};
const int bufferSize = 6000;
info circularBuffer[bufferSize];
int currentIndex = 0;

/*
 * Function to compute the illuminance from the LDR sensor
 * @param adc_value: the value read from the ADC
 * @return the illuminance in lux
 */
double dac_lux(float adc_value)
{
    double R_LDR = R * (DAC_RANGE / ((double)adc_value) - 1); // Convert from adc valu to resistance  V_ADC = adc_value / DAC_RANGE * VCC; R_LDR = VCC / V_ADC * R - R
    return pow(10, (log10(R_LDR) - b) / m);                   // Convert from resistance to lux
}
/*
 * Function to compute the volts from Lux
 * @param lux: the illuminance in lux
 * @param b: the y-intercept of the LDR sensor
 * @param m: the slope of the LDR sensor
 * @param vcc: the voltage of the LDR sensor
 * @return the voltage
 */
float lux_volt(float lux, float b, float m, float vcc)
{
    float resistance = pow(10, (m * log10(lux) + b));
    return (vcc * 10000.0) / (resistance + 10000.0);
}

/*
 * Function to calibrate the LDR sensor
 * @return the gain of the LDR sensor
 */
double calibration()
{
    analogWrite(LED_PIN, 0);
    delay(2000);
    float external_lux = dac_lux(analogRead(ADC_PIN));
    analogWrite(LED_PIN, 4095);
    delay(4000);
    float y_max = dac_lux(analogRead(ADC_PIN));
    return (y_max - external_lux) / 1 - 0;
}

/*
 * Function to compute the power consumption of the luminaire
 * @return the power consumption in watts
 */
float imediate_power_cons()
{
    return (power_max * duty_cycle);
}
/*
 * Function to compute the time since the last restart
 * @return the time in seconds
 */
float time_since_restart()
{
    return (millis() / 1000);
}
/*
 * Function to compute the average energy consumption of the luminaire
 * @return the energy consumption in joules
 */
float average_energy_consumption(int desk) // do it in joule
{
  Serial.println((currentTime - previousTime));
    energy += power_max * duty_cycle * 0.01;
    return energy;
}
/*
 *   Function to compute the average visibility error
 * @param r: illuminance reference in lux
 * @param y: illuminance measured in lux
 *
 */
void average_visibility_error(float r, float y) // accumulated visibility error in lux.
{
    double aux = r - y;
    if (aux < 0)
    {
        aux = 0;
    }
    sum_visibility_error += aux;
    mean_visibility = sum_visibility_error / counter;
}
/*
 *   Function to compute the average flicker error
 */
void average_flicker_error(double dc) // accumulated flicker error in Hz (s^-1)
{
    if (((dc - dc_1) * (dc_1 - dc_2)) < 0)
    {
        sum_flicker += abs(dc - dc_1) + abs(dc_1 - dc_2);
    }
    mean_flicker = sum_flicker / counter * 1000; // ver que está em milis

    dc_2 = dc_1;
    dc_1 = dc;
}

/*
 * Function to save the illuminance to the buffer
 */
void saveToBuffer_lux(double val)
{
    circularBuffer[currentIndex].lux = val;
}
/*
 * Function to save the duty cycle to the buffer
 */
void saveToBuffer_dc(double val)
{
    circularBuffer[currentIndex].d_cycle = val;
    currentIndex = (currentIndex + 1) % bufferSize;
}
/*
 * Function to print the buffer
 */
void printBuffer(char c)
{
    if (c == 'l')
    {
        for (int i = 0; i < bufferSize; i++)
        {
            Serial.print(circularBuffer[i].lux);
            Serial.print(" ");
        }
    }
    else
    {
        for (int i = 0; i < bufferSize; i++)
        {
            Serial.print(circularBuffer[i].d_cycle);
            Serial.print(" ");
        }
    }
    Serial.println();
}
/*
 * Function to process the commands received from the serial port
 * @param command: the command received from the serial port
 */
void processCommand(const String &command)
{
    int i = Led; // desk number (just one desk for now) Led = 1
    float val;
    char x;
    double time;

    if (command.startsWith("d ")) // Set directly the duty cycle of luminaire i.
    {
        sscanf(command.c_str(), "d %d %f", &i, &val);
        if (i == Led and val >= 0 and val <= 1) // não dá
        {
            duty_cycle = val;
            flag_dc=1;
            Serial.println("ack");
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("g d ")) // Get current duty cycle of luminaire i
    {
        sscanf(command.c_str(), "g d %d", &i);
        if (i == Led)
        {
            Serial.print("d ");
            Serial.print(i);
            Serial.print(" ");
            Serial.println(duty_cycle);
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("r ")) // Set the illuminance reference of luminaire i
    {
        sscanf(command.c_str(), "r %d %f", &i, &val);
        if (i == Led and val >= 0)
        {
            flag_dc=0;
            r = val;
            B = 1 / (K * ganho * (lux_volt(r, b, m, vcc) / r));
            float resistance = pow(10, (m * log10(r) + b));
            Ti = 10e-6 * ((10000 * resistance) / (10000 + resistance));
            Serial.println("ack");
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("g r ")) // Get current illuminance reference of luminaire i
    {
        sscanf(command.c_str(), "g r %d", &i);
        if (i == Led)
        {
            Serial.print("r ");
            Serial.print(i);
            Serial.print(" ");
            Serial.println(r);
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("g l ")) // Measure the illuminance of luminaire i
    {
        sscanf(command.c_str(), "g l %d", &i);
        if (i == Led)
        {
            Serial.print("l ");
            Serial.print(i);
            Serial.print(" ");
            Serial.println(y);
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("o ")) // Set the current occupancy state of desk <i>
    {
        sscanf(command.c_str(), "o %d %f", &i, &val);
        if (i == Led and (val == 0 or val == 1))
        {
            occupied = val;
            Serial.println("ack");
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("g o ")) // Get the current occupancy state of desk <i>
    {
        sscanf(command.c_str(), "g o %d", &i);
        if (i == Led)
        {
            Serial.print("o ");
            Serial.print(i);
            Serial.print(" ");
            Serial.println(occupied);
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("a ")) // Set anti-windup state of desk <i>
    {
        sscanf(command.c_str(), "a %d %f", &i, &val);

        if (i == Led and (val == 0 or val == 1))
        {
            anti_windup = val;
            Serial.println("ack");
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("g a ")) // Get anti-windup state of desk <i>
    {
        sscanf(command.c_str(), "g a %d", &i);
        if (i == Led)
        {
            Serial.print("a ");
            Serial.print(i);
            Serial.print(" ");
            Serial.println(anti_windup);
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("k ")) // Set feedback on/off of desk <i>
    {
        sscanf(command.c_str(), "k %d %f", &i, &val);
        if (i == Led and (val == 0 or val == 1))
        {
            Serial.println("ack");
            feedback = val;
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("g k ")) // Get feedback state of desk <i>
    {
        sscanf(command.c_str(), "g k %d", &i);
        if (i == Led)
        {
            Serial.print("k ");
            Serial.print(i);
            Serial.print(" ");
            Serial.println(feedback);
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("g x ")) // Get current external illuminance of desk <i>
    {
        sscanf(command.c_str(), "g x %d", &i);
        if (i == Led)
        {
            float ext = y - ganho * duty_cycle;
            if (ext < 0)
            {
                ext = 0;
            }
            Serial.print("x ");
            Serial.print(i);
            Serial.print(" ");
            Serial.println(ext);
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("g p ")) // Get instantaneous power consumption of desk <i>
    {
        if (i == Led)
        {
            Serial.print("p ");
            Serial.print(i);
            Serial.print(" ");
            Serial.println(imediate_power_cons()); // ver
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("g t ")) // Get the elapsed time since the last restart
    {
        sscanf(command.c_str(), "g t %d", &i);
        if (i == Led)
        {
            Serial.print("t ");
            Serial.print(i);
            Serial.print(" ");
            Serial.println(time_since_restart());
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("s ")) // Start the stream of the real-time variable <x> of desk <i>. <x> can be 'l' or 'd'.
    {
        sscanf(command.c_str(), "s %c %d", &x, &i);
        if (i == Led and x == 'l')
        {
            flag_stream_lux = i;
        }
        else if (i == Led and x == 'd')
        {
            flag_stream_duty_c = i;
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("S ")) // Stop the stream of the real-time variable <x> of desk <i>. <x> can be 'l' or 'd'.
    {
        sscanf(command.c_str(), "S %c %d", &x, &i);
        if (i == Led and x == 'l')
        {
            flag_stream_lux = 0;
            Serial.println("ack");
        }
        else if (i == Led and x == 'd')
        {
            flag_stream_duty_c = 0;
            Serial.println("ack");
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("g b ")) // Get the last minute buffer of the variable <x> of the desk <i>. <x> can be 'l' or 'd'.
    {
        sscanf(command.c_str(), "g b %c %d", &x, &i);
        if (i == Led and x == 'l')
        {
            Serial.print("b ");
            Serial.print(x);
            Serial.print(" ");
            Serial.print(i);
            Serial.print(" ");
            printBuffer('l');
        }
        else if (i == Led and x == 'd') // nao da
        {
            Serial.print("b ");
            Serial.print(x);
            Serial.print(" ");
            Serial.print(i);
            Serial.print(" ");
            printBuffer('d');
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("g e ")) // Get the average energy consumption at the desk <i> since the last system restart.
    {
        sscanf(command.c_str(), "g e %d", &i); // nao da
        if (i == Led)
        {
            Serial.print("e ");
            Serial.print(i);
            Serial.print(" ");
            Serial.println(average_energy_consumption(i));
        }
        else
        {
            Serial.println("err");
        }
    }
    else if (command.startsWith("g v ")) // Get the average visibility error at desk <i> since the last system restart.
    {
        sscanf(command.c_str(), "g v %d", &i);

        Serial.print("v ");
        Serial.print(i);
        Serial.print(" ");
        Serial.println(mean_visibility);
    }
    else if (command.startsWith("g f ")) // Get the average flicker error on desk <i> since the last system restart.
    {
        sscanf(command.c_str(), "g f %d", &i);

        Serial.print("f ");
        Serial.print(i);
        Serial.print(" ");
        Serial.println(mean_flicker);
    }
    else if (command.startsWith("mudar "))
    {
        sscanf(command.c_str(), "mudar %f", &K);
        B = 1 / (K * ganho * (lux_volt(r, b, m, vcc) / r));
    }
    else if (command.startsWith("m "))
    {
        sscanf(command.c_str(), "m %f", &m);
    }
    else
    {
        Serial.println("err"); // Command not recognized
    }
}