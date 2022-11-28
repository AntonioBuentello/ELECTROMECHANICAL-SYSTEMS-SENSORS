/*
 *  Antonio Buentello
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "wait.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "clock.h"
#include "i2c0.h"

/*
 * Device: Registers
 *  Register address pointer
 *      00 : Conversion register 16 bit
 *      01 : Config register 16 bit
 *      10 : Lo_thresh register 16 bit
 *      11 : Hi_thresh register 16 bit
 *
 */

#define ADS1115_I2C_ADDRESS_DEFAULT 0x48
#define CONVERSION 0x00
#define CONFIG 0x1

/*
 * 14:12 MUX[2:0] R/W 0h Input multiplexer configuration
 *
 * @legend AINp and AINn denote the selected Positive and Negative inputs. AINx denotes one of the four available analog inputs.
 * These bits configure the input multiplexer. These bits serve no function on the ADS1113 and ADS1114.
 *  000 : AINp = AIN0  and AINn = AIN1 (default)
 *  001 : AINp = AIN0  and AINn = AIN3
 *  010 : AINp = AIN1  and AINn = AIN3
 *  011 : AINp = AIN2  and AINn = AIN3
 *  100 : AINp = AIN0  and AINn = GND  => The setting if you want to measure voltage on the AIN0 input pin (compared to GND).
 *  101 : AINp = AIN1  and AINn = GND
 *  110 : AINp = AIN2  and AINn = GND
 *  111 : AINp = AIN3  and AINn = GND
 * @default 0b0
 */

/*
 * 11:9 PGA[2:0] R/W 2h Programmable gain amplifier configuration
 *
 * These bits set the FSR of the programmable gain amplifier. These bits serve no function on the ADS1113.
 *  000 : FSR = ±6.144 V            => 1 bit = 3mV
 *  001 : FSR = ±4.096 V            => 1 bit = 2mV
 *  010 : FSR = ±2.048 V (default)  => 1 bit = 1mV
 *  011 : FSR = ±1.024 V            => 1 bit = 0.5mV
 *  100 : FSR = ±0.512 V            => 1 bit = 0.25mV
 *  101 : FSR = ±0.256 V            => 1 bit = 0.125mV
 *
 */
#define FSR_6_144V 0x0
#define FSR_4_096V 0x1
#define FSR_2_048V 0x2
#define FSR_1_024V 0x3
#define FSR_0_256V 0x4

float negative[101] =
{-3.554, -3.523, -3.492, -3.462, -3.431, -3.400, -3.368, -3.337, -3.306, -3.274,
-3.243, -3.211, -3.179, -3.147, -3.115, -3.083, -3.050, -3.018, -2.986, -2.953,
-2.920, -2.887, -2.854, -2.821, -2.788, -2.755, -2.721, -2.688, -2.654, -2.620,
-2.587, -2.553, -2.519, -2.485, -2.450, -2.416, -2.382, -2.347, -2.312, -2.278,
-2.243, -2.208, -2.173, -2.138, -2.103, -2.067, -2.032, -1.996, -1.961, -1.925,
-1.889, -1.854, -1.818, -1.782, -1.745, -1.709, -1.673, -1.637, -1.600, -1.564,
-1.527, -1.490, -1.453, -1.417, -1.380, -1.343, -1.305, -1.268, -1.231, -1.194,
-1.156, -1.119, -1.081, -1.043, -1.006, -0.968, -0.930, -0.892, -0.854, -0.816,
-0.778, -0.739, -0.701, -0.663, -0.624, -0.586, -0.547, -0.508, -0.470, -0.431,
-0.392, -0.353, -0.314, -0.275, -0.236, -0.197, -0.157, -0.118, -0.079, -0.039, 0.000};

float positive[301] =
{
 0.000, 0.039, 0.079, 0.119, 0.158, 0.198, 0.238, 0.277, 0.317, 0.357,
 0.397, 0.437, 0.477, 0.517, 0.557, 0.597, 0.637, 0.677, 0.718, 0.758,
 0.798, 0.838, 0.879, 0.919, 0.960, 1.000, 1.041, 1.081, 1.122, 1.163,
 1.203, 1.244, 1.285, 1.326, 1.366, 1.407, 1.448, 1.489, 1.530, 1.571,
 1.612, 1.653, 1.694, 1.735, 1.776, 1.817, 1.858, 1.899, 1.941, 1.982,
 2.023, 2.064, 2.106, 2.147, 2.188, 2.230, 2.271, 2.312, 2.354, 2.395,
 2.436, 2.478, 2.519, 2.561, 2.602, 2.644, 2.685, 2.727, 2.768, 2.810,
 2.851, 2.893, 2.934, 2.976, 3.017, 3.059, 3.100, 3.142, 3.184, 3.225,
 3.267, 3.308, 3.350, 3.391, 3.433, 3.474, 3.516, 3.557, 3.599, 3.640,
 3.682, 3.723, 3.765, 3.806, 3.848, 3.889, 3.931, 3.972, 4.013, 4.055,
 4.096, 4.138, 4.179, 4.220, 4.262, 4.303, 4.344, 4.385, 4.427, 4.468,
 4.509, 4.550, 4.591, 4.633, 4.674, 4.715, 4.756, 4.797, 4.838, 4.879,
 4.920, 4.961, 5.002, 5.043, 5.084, 5.124, 5.165, 5.206, 5.247, 5.288,
 5.328, 5.369, 5.410, 5.450, 5.491, 5.532, 5.572, 5.613, 5.653, 5.694,
 5.735, 5.775, 5.815, 5.856, 5.896, 5.937, 5.977, 6.017, 6.058, 6.098,
 6.138, 6.179, 6.219, 6.259, 6.299, 6.339, 6.380, 6.420, 6.460, 6.500,
 6.540, 6.580, 6.620, 6.660, 6.701, 6.741, 6.781, 6.821, 6.861, 6.901,
 6.941, 6.981, 7.021, 7.060, 7.100, 7.140, 7.180, 7.220, 7.260, 7.300,
 7.340, 7.380, 7.420, 7.460, 7.500, 7.540, 7.579, 7.619, 7.659, 7.699,
 7.739, 7.779, 7.819, 7.859, 7.899, 7.939, 7.979, 8.019, 8.059, 8.099,
 8.138, 8.178, 8.218, 8.258, 8.298, 8.338, 8.378, 8.418, 8.458, 8.499,
 8.539, 8.579, 8.619, 8.659, 8.699, 8.739, 8.779, 8.819, 8.860, 8.900,
 8.940, 8.980, 9.020, 9.061, 9.101, 9.141, 9.181, 9.222, 9.262, 9.302,
 9.343, 9.383, 9.423, 9.464, 9.504, 9.545, 9.585, 9.626, 9.666, 9.707,
 9.747, 9.788, 9.828, 9.869, 9.909, 9.950, 9.991, 10.031, 10.072, 10.113,
 10.153, 10.194, 10.235, 10.276, 10.316, 10.357, 10.398, 10.439, 10.480, 10.520,
 10.561, 10.602, 10.643, 10.684, 10.725, 10.766, 10.807, 10.848, 10.889, 10.930,
 10.971, 11.012, 11.053, 11.094, 11.135, 11.176, 11.217, 11.259, 11.300, 11.341,
 11.382, 11.423, 11.465, 11.506, 11.547, 11.588, 11.630, 11.671, 11.712, 11.753,
 11.795, 11.836, 11.877, 11.919, 11.960, 12.001, 12.043, 12.084, 12.126, 12.167, 12.209
};

void initHw(){
    initSystemClockTo40Mhz();
}
float int16ToC(int16_t value){
    float voltage = value * 2.048;
    voltage /= 32767.0;
    return (voltage - 0.5) * 100.0;
}
float int16ToScaledVoltsTMP36(int16_t value){
    float voltage = value * 2.048;
    voltage /= 32767.0;
    return voltage;
}
float int16ToScaledVoltsThermocouple(int16_t value){
    float voltage = value * 0.256;
    voltage /= 32767.0;
    return voltage * 1000.0;
}
float tmpMeasure(void){
    char str[10];
    int16_t raw = 0;
    float scaled = 0;
    float scaledMV = 0;

    uint8_t rawData[2] = {0,0};
    uint8_t configData[2] = {0,0};

    configData[1] = 0x63;
    configData[0] = 0x44;

    writeI2c0Registers(ADS1115_I2C_ADDRESS_DEFAULT, CONFIG, configData, 2);
    waitMicrosecond(100000);
    readI2c0Registers(ADS1115_I2C_ADDRESS_DEFAULT, CONVERSION, rawData, 2);

    raw = ((rawData[0] << 8) | rawData[1]);

    // 100 : AINP = AIN0 and AINN = GND

    putsUart0("RAW TMP36: ");
    sprintf(str, "%d", raw);
    putsUart0(str);
    putsUart0("\n");

    putsUart0("Scaled TMP36 (mV): ");
    scaledMV = int16ToScaledVoltsTMP36(raw);
    sprintf(str, "%f", scaledMV);
    putsUart0(str);
    putsUart0("\n");

    putsUart0("TMP36 Temperature (C): ");
    scaled = int16ToC(raw);
    sprintf(str, "%f", scaled);
    putsUart0(str);
    putsUart0("\n");

    return scaledMV;

}
float thermoMeasure(void){
    char str[10];
    int16_t raw2 = 0;
    float scaled2 = 0;
    uint8_t rawData2[2] = {0,0};
    uint8_t configData2[2] = {0,0};

    configData2[1] = 0x63;
    configData2[0] = 0x2E;

    writeI2c0Registers(ADS1115_I2C_ADDRESS_DEFAULT, CONFIG, configData2, 2);
    waitMicrosecond(100000);
    readI2c0Registers(ADS1115_I2C_ADDRESS_DEFAULT, CONVERSION, rawData2, 2);

    raw2 = ((rawData2[0] << 8) | rawData2[1]);

    // 010 : AINP = AIN1 and AINN = AIN3

    putsUart0("RAW Thermo-couple: ");
    sprintf(str, "%d", raw2);
    putsUart0(str);
    putsUart0("\n");

    putsUart0("Scaled Thermo-couple (mV): ");
    scaled2 = int16ToScaledVoltsThermocouple(raw2);
    sprintf(str, "%f", scaled2);
    putsUart0(str);
    putsUart0("\n");

    return scaled2;

}
// y = y1 + ((x - x1) / (x2 - x1)) * (y2 - y1), where x is the known value, y is the unknown value,
// x1 and y1 are the coordinates that are below the known x value,
//and x2 and y2 are the coordinates that are above the x value.

float interpolate(bool isNegative, bool isPositive, int32_t index, float summedVoltage){
    float y = 0.0;
    float x = summedVoltage;
    float y1, y2, x1, x2;

    if (isNegative){
        y1 = index - 1;
        x1 = negative[index - 1];
        y2 = index + 1;
        x2 = negative[index + 1];
        y = y1 + ((x - x1) / (x2 - x1)) * (y2 - y1);
        return y-100.00;

    }
    else{
        y1 = index - 1;
        x1 = positive[index - 1];
        y2 = index + 1;
        x2 = positive[index + 1];

        y = y1 + ((x - x1) / (x2 - x1)) * (y2 - y1);
        return y;
    }
}
int main(void){
    initHw();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    initI2c0();
    char str[10];

    float coldJunctionVoltage;
    float thermocoupleVoltage;
    float summedVoltage;
    float temperature;              // (Vcj + Vtc = index into table) -> temperature of thermocouple

    int32_t i = 0;
    bool isNegative = false;
    bool isPositive = false;


    while(1){
        i = 0;

        coldJunctionVoltage = tmpMeasure();
        thermocoupleVoltage = thermoMeasure();
        summedVoltage = coldJunctionVoltage + thermocoupleVoltage;
        putsUart0("Summed Voltage: ");
        sprintf(str, "%f", summedVoltage);
        putsUart0(str);
        putsUart0("\n");

        if (summedVoltage < 0.0){
            isNegative = true;
            isPositive = false;
            while (negative[i] < summedVoltage){
                i++;
            }
            temperature = interpolate(isNegative, isPositive, i, summedVoltage);
        }
        if(summedVoltage > 0.0){
            i = 0;
            isPositive = true;
            isNegative = false;
            while(positive[i] < summedVoltage){
                i++;
            }
            temperature = i;

            temperature = interpolate(isNegative, isPositive, i, summedVoltage);
        }
        putsUart0("i = ");
        sprintf(str, "%d", i);
        putsUart0(str);
        putsUart0("\n");

        putsUart0("Actual Temperature (C): ");
        sprintf(str, "%f", temperature);
        putsUart0(str);
        putsUart0("\n\n");


        waitMicrosecond(500000);
    }
	while(1);
}
