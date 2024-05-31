#include <Wire.h>
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
bool isFirstCalibration = true;
float rollInitial, pitchInitial;
#include <PulsePosition.h>
PulsePositionInput ReceiverInput(RISING);
int ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
float roll, pitch, yaw;
int ChannelNumber = 0;
float Voltage, Current, BatteryRemaining, BatteryAtStart;
float CurrentConsumed = 0;
float BatteryDefault = 1500;

uint32_t LoopTimer;
bool init = true;
/*
flight mode = 0: manual mode
flight mode = 1: hold altitude use pressure sensor
flight mode = 2: hold altitude and position use pressure and GPS sensor
*/
int flightMode = 0;

float AccXYZ[3][1] = {{0}, {0}, {0}};
float matrixA[3][3] = {{0.996629, -0.004342, -0.001918},
                       {-0.004342, 0.993976, -0.002411},
                       {-0.001918, -0.002411, 0.982793}};
float vectorB[3][1] = {{0.060620}, {-0.007024}, {-0.088088}};
void calibAcc()
{
    float temp[3][3] = {{}, {}, {}};
    for (int i = 0; i < 3; i++)
    {
        temp[i][0] = AccXYZ[i][0] - vectorB[i][0];
    }

    for (int i = 0; i < 3; i++)
    {
        AccXYZ[i][0] = 0;
        for (int j = 0; j < 3; j++)
        {
            AccXYZ[i][0] += matrixA[i][j] * temp[j][0];
        }
    }
}

// PID control for rate flightMode
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float TempInputThrottle;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};

float PRateRoll = 1.5; // 0.6, 2.25
float PRatePitch = PRateRoll;
float PRateYaw = 2;
float IRateRoll = 10.5; // 3.5, 7.5, 10.5
float IRatePitch = IRateRoll;
float IRateYaw = 12;
float DRateRoll = 0.06; // 0.03, 0.05
float DRatePitch = DRateRoll;
float DRateYaw = 0;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// kalman filter and PID for stable mode
float AngleRoll, AnglePitch, AngleYaw;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll = 2.0; // 2.0, 5.0
float PAnglePitch = PAngleRoll;
float IAngleRoll = 0; // 3.5
float IAnglePitch = IAngleRoll;
float DAngleRoll = 0; // 0.0
float DAnglePitch = DAngleRoll;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
    KalmanState = KalmanState + 0.004 * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}
void battery_voltage(void)
{
    Voltage = (float)analogRead(15) / 62;
    Current = (float)analogRead(21) * 0.089;
}
void read_receiver(void)
{
    ChannelNumber = ReceiverInput.available();
    if (ChannelNumber > 0)
    {
        for (int i = 1; i <= ChannelNumber; i++)
        {
            ReceiverValue[i - 1] = ReceiverInput.read(i);
        }
    }
}
void gyro_signals(void)
{
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    // Wire.write(0x05);
    Wire.write(0x06);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x8);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();
    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;
    AccXYZ[0][0] = (float)AccXLSB / 4096;
    AccXYZ[1][0] = (float)AccYLSB / 4096;
    AccXYZ[2][0] = (float)AccZLSB / 4096;
}
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
    if (Iterm > 400)
        Iterm = 400;
    else if (Iterm < -400)
        Iterm = -400;
    float Dterm = D * (Error - PrevError) / 0.004;
    float PIDOutput = Pterm + Iterm + Dterm;
    if (PIDOutput > 400)
        PIDOutput = 400;
    else if (PIDOutput < -400)
        PIDOutput = -400;
    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}
void reset_pid(void)
{
    PrevErrorRateRoll = 0;
    PrevErrorRatePitch = 0;
    PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0;
    PrevItermRatePitch = 0;
    PrevItermRateYaw = 0;
    PrevErrorAngleRoll = 0;
    PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = 0;
    PrevItermAnglePitch = 0;
}

//-----------------begin hold altitude use ultrasonic sensor mode-----------------------------
#define GY_US42V2_ADDRESS 0x70
unsigned int altitude = 0;
bool isGPS_signal = false;

IntervalTimer myTimer;
volatile unsigned long holdingInterval = 0;
float prevInputThrottle = 0.0;
bool holdThrottleFlag = false;
bool holdAltitudeFlag_sonar = true;
bool holdAltitudeFlag_pressure = true;
volatile unsigned long measureDistanceInterval = 0;
bool measureDistanceFlag = false;
int count_toggle_ledGPS;

// PID control for hold altitude mode
float DesiredAltitude;
float ErrorAltitude;
float PrevErrorAltitude;
float PrevItermAltitude;

float P_Altitude = 1.0;   // 2.0
float I_Altitude = 0.005; // 0.005
float D_Altitude = 1.5;

void pid_equation_altitude(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
    if (Iterm > 400)
        Iterm = 400;
    else if (Iterm < -400)
        Iterm = -400;
    float Dterm = D * (Error - PrevError) / 0.004;
    float PIDOutput = Pterm + Iterm + Dterm;
    if (PIDOutput > 400)
        PIDOutput = 400;
    else if (PIDOutput < -400)
        PIDOutput = -400;
    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}
void timerRun()
{
    // holdingInterval++;
    // if (holdingInterval >= 500)
    // {
    //     holdingInterval = 0;
    //     holdThrottleFlag = true;
    // }
    if (!isGPS_signal)
    {
        digitalWrite(6, HIGH);
    }
    else
    {
        count_toggle_ledGPS++;
        if (count_toggle_ledGPS == 1000)
        {
            digitalWrite(6, HIGH);
        }
        if (count_toggle_ledGPS == 2000)
        {
            count_toggle_ledGPS = 0;
            digitalWrite(6, LOW);
        }
    }
    measureDistanceInterval++;
    if (measureDistanceInterval >= 65)
    {
        measureDistanceInterval = 0;
        measureDistanceFlag = true;
    }
}
bool isHoldThrottle()
{
    if (abs(prevInputThrottle - InputThrottle) < 20)
    {
        return true;
    }
    else
    {
        return false;
    }
}

unsigned int getAltitude()
{
    unsigned int distance = 0;
    Wire.requestFrom(GY_US42V2_ADDRESS, 2); // Request 2 bytes from sensor
    if (Wire.available() >= 2)
    {
        byte highByte = Wire.read();
        byte lowByte = Wire.read();
        distance = (highByte << 8) | lowByte; // Combine high and low bytes
    }

    // Start measurement
    Wire.beginTransmission(GY_US42V2_ADDRESS);
    Wire.write(0x51); // Command to start measurement
    Wire.endTransmission();

    return distance;
}
//-----------------end hold altitude use ultrasonic sensor mode-----------------------------

//----------------BEGIN ALTITUDE HOLD USE PRESSURE SENSOR MODE-----------------------------
#define MS5611_address 0x77 // The I2C address of the MS5611 barometer is 0x77 in
uint8_t start;
uint32_t stable_time = 0;
// Pressure variables.
float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[20], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT;

int32_t pressure_rotating_mem[45], pressure_total_avarage;
uint8_t pressure_rotating_mem_location = 0;
float pressure_rotating_mem_actual;

// PID control for hold altitude mode
float DesiredAltitude_pressure;
float ErrorAltitude_pressure;
float PrevErrorAltitude_pressure;
float PrevItermAltitude_pressure;

float P_Altitude_pressure = 3.0;  // 1.4 3.4
float I_Altitude_pressure = 0.5;  // 0.2, 0.5
float D_Altitude_pressure = 6.75; // 1.75 3.75
float PID_error_gain_altitude;
void pid_equation_pressure(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
    PID_error_gain_altitude = 0; // Set the pid_error_gain_altitude to 0.
    if (Error > 10 || Error < -10)
    {                                                       // If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        PID_error_gain_altitude = (abs(Error) - 10) / 20.0; // The positive pid_error_gain_altitude variable is calculated based based on the error.
        if (PID_error_gain_altitude > 3)
            PID_error_gain_altitude = 3; // To prevent extreme P-gains it must be limited to 3.
    }
    float Pterm = (P + PID_error_gain_altitude) * Error;
    // float Pterm = (P)*Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
    if (Iterm > 400)
        Iterm = 400;
    else if (Iterm < -400)
        Iterm = -400;
    float Dterm = D * (Error - PrevError) / 0.004;
    float PIDOutput = Pterm + Iterm + Dterm;
    if (PIDOutput > 400)
        PIDOutput = 400;
    else if (PIDOutput < -400)
        PIDOutput = -400;
    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}

void read_barometer(void)
{
    barometer_counter++;
    stable_time++;
    if (barometer_counter == 1)
    {
        if (temperature_counter == 0)
        {
            Wire.beginTransmission(MS5611_address); // Open a connection with the MS5611
            Wire.write(0x00);                       // Send a 0 to indicate that we want to poll the requested data.
            Wire.endTransmission();                 // End the transmission with the MS5611.
            Wire.requestFrom(MS5611_address, 3);    // Poll 3 data bytes from the MS5611.
            // Store the temperature in a 5 location rotating memory to prevent temperature spikes.
            raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
            raw_temperature_rotating_memory[average_temperature_mem_location] = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
            raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
            average_temperature_mem_location++;
            if (average_temperature_mem_location == 5)
                average_temperature_mem_location = 0;
            raw_temperature = raw_average_temperature_total / 5; // Calculate the avarage temperature of the last 5 measurements.
        }
        else
        {
            // Get pressure data from MS-5611
            Wire.beginTransmission(MS5611_address);                            // Open a connection with the MS5611.
            Wire.write(0x00);                                                  // Send a 0 to indicate that we want to poll the requested data.
            Wire.endTransmission();                                            // End the transmission with the MS5611.
            Wire.requestFrom(MS5611_address, 3);                               // Poll 3 data bytes from the MS5611.
            raw_pressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read(); // Shift the individual bytes in the correct position and add them to the raw_pressure variable.
        }

        temperature_counter++; // Increase the temperature_counter variable.
        if (temperature_counter == 20)
        {                            // When the temperature counter equals 20.
            temperature_counter = 0; // Reset the temperature_counter variable.
            // Request temperature data
            Wire.beginTransmission(MS5611_address); // Open a connection with the MS5611.
            Wire.write(0x58);                       // Send a 0x58 to indicate that we want to request the temperature data.
            Wire.endTransmission();                 // End the transmission with the MS5611.
        }
        else
        { // If the temperature_counter variable does not equal 20.
            // Request pressure data
            Wire.beginTransmission(MS5611_address); // Open a connection with the MS5611
            Wire.write(0x48);                       // Send a 0x48 to indicate that we want to request the pressure data.
            Wire.endTransmission();                 // End the transmission with the MS5611.
        }
    }
    if (barometer_counter == 2)
    {
        // Calculate pressure as explained in the datasheet of the MS-5611.
        dT = C[5];
        dT <<= 8;
        dT *= -1;
        dT += raw_temperature;
        OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
        SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
        P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
        // To get a smoother pressure value we will use a 20 location rotating memory.
        pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location]; // Subtract the current memory position to make room for the new value.
        pressure_rotating_mem[pressure_rotating_mem_location] = P;                       // Calculate the new change between the actual pressure and the previous measurement.
        pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location]; // Add the new value to the long term avarage value.
        pressure_rotating_mem_location++;                                                // Increase the rotating memory location.
        if (pressure_rotating_mem_location == 20)
            pressure_rotating_mem_location = 0;                    // Start at 0 when the memory location 20 is reached.
        actual_pressure_fast = (float)pressure_total_avarage / 20; // Calculate the average pressure of the last 20 pressure readings.

        actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
        actual_pressure_diff = actual_pressure_slow - actual_pressure_fast; // Calculate the difference between the fast and the slow avarage value.
        if (actual_pressure_diff > 8)
            actual_pressure_diff = 8; // If the difference is larger then 8 limit the difference to 8.
        if (actual_pressure_diff < -8)
            actual_pressure_diff = -8; // If the difference is smaller then -8 limit the difference to -8.
        // If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
        if (actual_pressure_diff > 1 || actual_pressure_diff < -1.0)
            actual_pressure_slow -= actual_pressure_diff / 6.0;
        actual_pressure = actual_pressure_slow; // The actual_pressure is used in the program for altitude calculations.
    }
    if (barometer_counter == 3)
    {
        barometer_counter = 0;
    }
}

//----------------END ALTITUDE HOLD USE PRESSURE SENSOR MODE-----------------------------

//----------------  BEGIN COMPASS----------------------------------------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float MagXYZ[3][1] = {{0}, {0}, {0}};

float MagMatrixA[3][3] = {{0.890893, 0.003222, 0.009447},
                          {0.003222, 0.871716, -0.004255},
                          {0.009447, -0.004255, 0.837771}};
float MagVectorB[3][1] = {{1.973203}, {-16.262507}, {1.325709}};
void calibMag()
{
    float temp[3][3] = {{}, {}, {}};
    for (int i = 0; i < 3; i++)
    {
        temp[i][0] = MagXYZ[i][0] - MagVectorB[i][0];
    }

    for (int i = 0; i < 3; i++)
    {
        MagXYZ[i][0] = 0;
        for (int j = 0; j < 3; j++)
        {
            MagXYZ[i][0] += MagMatrixA[i][j] * temp[j][0];
        }
    }
}

float MagMatrixA_lv2[3][3] = {{0.938648, 0.090491, -0.042117},
                              {0.090491, 0.981488, 0.052350},
                              {-0.042117, 0.052350, 1.074485}};
float MagVectorBlv2[3][1] = {{-3.992910}, {2.361108}, {-1.076436}};
void calibMagLv2()
{
    float temp[3][3] = {{}, {}, {}};
    for (int i = 0; i < 3; i++)
    {
        temp[i][0] = MagXYZ[i][0] - MagVectorBlv2[i][0];
    }

    for (int i = 0; i < 3; i++)
    {
        MagXYZ[i][0] = 0;
        for (int j = 0; j < 3; j++)
        {
            MagXYZ[i][0] += MagMatrixA_lv2[i][j] * temp[j][0];
        }
    }
}

float MagMatrixA_lv3[3][3] = {{0.960092, -0.077023, 0.039598},
                              {-0.077023, 1.025752, -0.057869},
                              {0.039598, -0.057869, 0.879902}};
float MagVectorBlv3[3][1] = {{1.613222}, {-2.328276}, {1.855162}};
void calibMagLv3()
{
    float temp[3][3] = {{}, {}, {}};
    for (int i = 0; i < 3; i++)
    {
        temp[i][0] = MagXYZ[i][0] - MagVectorBlv3[i][0];
    }

    for (int i = 0; i < 3; i++)
    {
        MagXYZ[i][0] = 0;
        for (int j = 0; j < 3; j++)
        {
            MagXYZ[i][0] += MagMatrixA_lv3[i][j] * temp[j][0];
        }
    }
}

float MagMatrixA_lv4[3][3] = {{0.999171, -0.023199, 0.011314},
                              {-0.023199, 0.934559, 0.012608},
                              {0.011314, 0.012608, 1.004113}};
float MagVectorBlv4[3][1] = {{1.485420}, {1.127117}, {-0.301822}};
void calibMagLv4()
{
    float temp[3][3] = {{}, {}, {}};
    for (int i = 0; i < 3; i++)
    {
        temp[i][0] = MagXYZ[i][0] - MagVectorBlv4[i][0];
    }

    for (int i = 0; i < 3; i++)
    {
        MagXYZ[i][0] = 0;
        for (int j = 0; j < 3; j++)
        {
            MagXYZ[i][0] += MagMatrixA_lv4[i][j] * temp[j][0];
        }
    }
}

uint8_t compass_address = 0x1E;
float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
float compass_scale_y, compass_scale_z;
int16_t compass_offset_x, compass_offset_y, compass_offset_z;
int16_t compass_cal_values[6] = {-345, 741, -567, 568, -507, 1046};
float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
int16_t compass_x, compass_y, compass_z;
float declination = 0.8;
void read_compass()
{
    Wire.beginTransmission(compass_address); // Start communication with the compass.
    Wire.write(0x03);                        // We want to start reading at the hexadecimal location 0x03.
    Wire.endTransmission();                  // End the transmission with the gyro.

    Wire.requestFrom(compass_address, 6);       // Request 6 bytes from the compass.
    compass_y = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the compass_y variable.
    compass_y *= +1;                            // Invert the direction of the axis.
    compass_z = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the compass_z variable.;
    // compass_z *= +1;                            // Invert the direction of the axis.
    compass_x = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the compass_x variable.;
    compass_x *= +1;                            // Invert the direction of the axis.

    // Before the compass can give accurate measurements it needs to be calibrated. At startup the compass_offset and compass_scale
    // variables are calculated. The following part will adjust the raw compas values so they can be used for the calculation of the heading.
    compass_y += compass_offset_y; // Add the y-offset to the raw value.
    compass_y *= compass_scale_y;  // Scale the y-value so it matches the other axis.
    compass_z += compass_offset_z; // Add the z-offset to the raw value.
    compass_z *= compass_scale_z;  // Scale the z-value so it matches the other axis.
    compass_x += compass_offset_x; // Add the x-offset to the raw value.

    // The compass values change when the roll and pitch angle of the quadcopter changes. That's the reason that the x and y values need to calculated for a virtual horizontal position.
    // The 0.0174533 value is phi/180 as the functions are in radians in stead of degrees.
    compass_x_horizontal = (float)compass_x * cos(AnglePitch * +0.0174533) + (float)compass_y * sin(AngleRoll * 0.0174533) * sin(AnglePitch * +0.0174533) - (float)compass_z * cos(AngleRoll * 0.0174533) * sin(AnglePitch * +0.0174533);
    compass_y_horizontal = (float)compass_y * cos(AngleRoll * 0.0174533) + (float)compass_z * sin(AngleRoll * 0.0174533);

    // Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
    // Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
    if (compass_y_horizontal < 0)
        actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
    else
        actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);

    actual_compass_heading += declination; // Add the declination to the magnetic compass heading to get the geographic north.
    if (actual_compass_heading < 0)
        actual_compass_heading += 360; // If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
    else if (actual_compass_heading >= 360)
        actual_compass_heading -= 360; // If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
}

// At startup the registers of the compass need to be set. After that the calibration offset and scale values are calculated.
void setup_compass()
{
    Wire.beginTransmission(compass_address); // Start communication with the compass.
    Wire.write(0x00);                        // We want to write to the Configuration Register A (00 hex).
    Wire.write(0x78);                        // Set the Configuration Regiser A bits as 01111000 to set sample rate (average of 8 at 75Hz).
    Wire.write(0x20);                        // Set the Configuration Regiser B bits as 00100000 to set the gain at +/-1.3Ga.
    Wire.write(0x00);                        // Set the Mode Regiser bits as 00000000 to set Continues-Measurement Mode.
    Wire.endTransmission();                  // End the transmission with the compass.

    // Calculate the alibration offset and scale values
    compass_scale_y = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[3] - compass_cal_values[2]);
    compass_scale_z = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);

    compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
    compass_offset_y = (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]) * compass_scale_y;
    compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;
}

// The following subrouting calculates the smallest difference between two heading values.
float course_deviation(float course_b, float course_c)
{
    course_a = course_b - course_c;
    if (course_a < -180 || course_a > 180)
    {
        if (course_c > 180)
            base_course_mirrored = course_c - 180;
        else
            base_course_mirrored = course_c + 180;
        if (course_b > 180)
            actual_course_mirrored = course_b - 180;
        else
            actual_course_mirrored = course_b + 180;
        course_a = actual_course_mirrored - base_course_mirrored;
    }
    return course_a;
}
//----------------  END COMPASS------------------------------------------------------

//----------------BEGIN GPS--------------------------------------------
float gps_p_gain = 0.1;  // Gain setting for the GPS P-controller (default = 2.7).5.7
float gps_d_gain = 10.5; // Gain setting for the GPS D-controller (default = 6.5)30.5
float gps_i_gain = 0.1;  // Adjust this value as needed for your application
// Initialization of integral error terms (if not already initialized)
float gps_lat_error_integral = 0;
float gps_lon_error_integral = 0;
bool isHoldPosition_GPS = false;

uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
uint8_t waypoint_set = 0, latitude_north, longiude_east;
uint16_t message_counter;
uint8_t gps_add_counter;
int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
uint8_t new_line_found, new_gps_data_available, new_gps_data_counter;
uint8_t gps_rotating_mem_location, return_to_home_step;
int32_t gps_lat_total_avarage, gps_lon_total_avarage;
int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
int32_t gps_lat_error, gps_lon_error;
int32_t gps_lat_error_previous, gps_lon_error_previous;
uint32_t gps_watchdog_timer;

float l_lon_gps_float_adjust, l_lat_gps_float_adjust, gps_man_adjust_heading;
float return_to_home_lat_factor, return_to_home_lon_factor, return_to_home_move_factor;
uint8_t home_point_recorded;
int32_t lat_gps_home, lon_gps_home;
void gps_setup(void)
{
    Serial1.begin(38400);
    uint8_t disableGSA[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
    Serial1.write(disableGSA, 16);
    delay(250);

    uint8_t disableGSV[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
    Serial1.write(disableGSV, 16);
    delay(250);

    uint8_t disableRMC[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
    Serial1.write(disableRMC, 16);
    delay(250);

    uint8_t disableVTG[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
    Serial1.write(disableVTG, 16);
    delay(250);

    uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
    uint8_t Set_to_10Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
    Serial1.write(Set_to_10Hz, 14);
    delay(200);
}

void read_gps(void)
{
    while (Serial1.available() && new_line_found == 0)
    {                                           // Stay in this loop as long as there is serial information from the GPS available.
        char read_serial_byte = Serial1.read(); // Load a new serial byte in the read_serial_byte variable.
        if (read_serial_byte == '$')
        { // If the new byte equals a $ character.
            for (message_counter = 0; message_counter <= 99; message_counter++)
            {                                             // Clear the old data from the incomming buffer array.
                incomming_message[message_counter] = '-'; // Write a - at every position.
            }
            message_counter = 0; // Reset the message_counter variable because we want to start writing at the begin of the array.
        }
        else if (message_counter <= 99)
            message_counter++;                                 // If the received byte does not equal a $ character, increase the message_counter variable.
        incomming_message[message_counter] = read_serial_byte; // Write the new received byte to the new position in the incomming_message array.
        if (read_serial_byte == '*')
            new_line_found = 1; // Every NMEA line end with a *. If this character is detected the new_line_found variable is set to 1.
    }

    // If the software has detected a new NMEA line it will check if it's a valid line that can be used.
    if (new_line_found == 1)
    {                       // If a new NMEA line is found.
        new_line_found = 0; // Reset the new_line_found variable for the next line.
        if (incomming_message[4] == 'L' && incomming_message[5] == 'L' && incomming_message[7] == ',')
        { // When there is no GPS fix or latitude/longitude information available.
            // Set some variables to 0 if no valid information is found by the GPS module. This is needed for GPS lost when flying.
            isGPS_signal = false;
            l_lat_gps = 0;
            l_lon_gps = 0;
            lat_gps_previous = 0;
            lon_gps_previous = 0;
            number_used_sats = 0;
        }
        // If the line starts with GA and if there is a GPS fix we can scan the line for the latitude, longitude and number of satellites.
        if (incomming_message[4] == 'G' && incomming_message[5] == 'A' && (incomming_message[44] == '1' || incomming_message[44] == '2'))
        {
            isGPS_signal = true;
            lat_gps_actual = ((int)incomming_message[19] - 48) * (long)10000000;   // Filter the minutes for the GGA line multiplied by 10.
            lat_gps_actual += ((int)incomming_message[20] - 48) * (long)1000000;   // Filter the minutes for the GGA line multiplied by 10.
            lat_gps_actual += ((int)incomming_message[22] - 48) * (long)100000;    // Filter the minutes for the GGA line multiplied by 10.
            lat_gps_actual += ((int)incomming_message[23] - 48) * (long)10000;     // Filter the minutes for the GGA line multiplied by 10.
            lat_gps_actual += ((int)incomming_message[24] - 48) * (long)1000;      // Filter the minutes for the GGA line multiplied by 10.
            lat_gps_actual += ((int)incomming_message[25] - 48) * (long)100;       // Filter the minutes for the GGA line multiplied by 10.
            lat_gps_actual += ((int)incomming_message[26] - 48) * (long)10;        // Filter the minutes for the GGA line multiplied by 10.
            lat_gps_actual /= (long)6;                                             // To convert the minutes to degrees we need to divide the minutes by 6.
            lat_gps_actual += ((int)incomming_message[17] - 48) * (long)100000000; // Add the degrees multiplied by 10.
            lat_gps_actual += ((int)incomming_message[18] - 48) * (long)10000000;  // Add the degrees multiplied by 10.
            lat_gps_actual /= 10;                                                  // Divide everything by 10.

            lon_gps_actual = ((int)incomming_message[33] - 48) * (long)10000000;    // Filter the minutes for the GGA line multiplied by 10.
            lon_gps_actual += ((int)incomming_message[34] - 48) * (long)1000000;    // Filter the minutes for the GGA line multiplied by 10.
            lon_gps_actual += ((int)incomming_message[36] - 48) * (long)100000;     // Filter the minutes for the GGA line multiplied by 10.
            lon_gps_actual += ((int)incomming_message[37] - 48) * (long)10000;      // Filter the minutes for the GGA line multiplied by 10.
            lon_gps_actual += ((int)incomming_message[38] - 48) * (long)1000;       // Filter the minutes for the GGA line multiplied by 10.
            lon_gps_actual += ((int)incomming_message[39] - 48) * (long)100;        // Filter the minutes for the GGA line multiplied by 10.
            lon_gps_actual += ((int)incomming_message[40] - 48) * (long)10;         // Filter the minutes for the GGA line multiplied by 10.
            lon_gps_actual /= (long)6;                                              // To convert the minutes to degrees we need to divide the minutes by 6.
            lon_gps_actual += ((int)incomming_message[30] - 48) * (long)1000000000; // Add the degrees multiplied by 10.
            lon_gps_actual += ((int)incomming_message[31] - 48) * (long)100000000;  // Add the degrees multiplied by 10.
            lon_gps_actual += ((int)incomming_message[32] - 48) * (long)10000000;   // Add the degrees multiplied by 10.
            lon_gps_actual /= 10;                                                   // Divide everything by 10.

            if (incomming_message[28] == 'N')
            {
                latitude_north = 1; // When flying north of the equator the latitude_north variable will be set to 1.
            }
            else
                latitude_north = 0; // When flying south of the equator the latitude_north variable will be set to 0.

            if (incomming_message[42] == 'E')
            {
                longiude_east = 1; // When flying east of the prime meridian the longiude_east variable will be set to 1.
            }
            else
                longiude_east = 0; // When flying west of the prime meridian the longiude_east variable will be set to 0.

            number_used_sats = ((int)incomming_message[46] - 48) * (long)10; // Filter the number of satillites from the GGA line.
            number_used_sats += (int)incomming_message[47] - 48;             // Filter the number of satillites from the GGA line.

            if (lat_gps_previous == 0 && lon_gps_previous == 0)
            {                                      // If this is the first time the GPS code is used.
                lat_gps_previous = lat_gps_actual; // Set the lat_gps_previous variable to the lat_gps_actual variable.
                lon_gps_previous = lon_gps_actual; // Set the lon_gps_previous variable to the lon_gps_actual variable.
            }

            lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0; // Divide the difference between the new and previous latitude by ten.
            lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0; // Divide the difference between the new and previous longitude by ten.

            l_lat_gps = lat_gps_previous; // Set the l_lat_gps variable to the previous latitude value.
            l_lon_gps = lon_gps_previous; // Set the l_lon_gps variable to the previous longitude value.

            lat_gps_previous = lat_gps_actual; // Remember the new latitude value in the lat_gps_previous variable for the next loop.
            lon_gps_previous = lon_gps_actual; // Remember the new longitude value in the lat_gps_previous variable for the next loop.

            // The GPS is set to a 5Hz refresh rate. Between every 2 GPS measurments, 9 GPS values are simulated.
            gps_add_counter = 2;        // Set the gps_add_counter variable to 5 as a count down loop timer
            new_gps_data_counter = 9;   // Set the new_gps_data_counter to 9. This is the number of simulated values between 2 GPS measurements.
            lat_gps_add = 0;            // Reset the lat_gps_add variable.
            lon_gps_add = 0;            // Reset the lon_gps_add variable.
            new_gps_data_available = 1; // Set the new_gps_data_available to indicate that there is new data available.
        }

        // If the line starts with SA and if there is a GPS fix we can scan the line for the fix type (none, 2D or 3D).
        if (incomming_message[4] == 'S' && incomming_message[5] == 'A')
            fix_type = (int)incomming_message[9] - 48;
    }

    // After 5 program loops 5 x 4ms = 20ms the gps_add_counter is 0.
    if (gps_add_counter == 0 && new_gps_data_counter > 0)
    {                               // If gps_add_counter is 0 and there are new GPS simulations needed.
        new_gps_data_available = 1; // Set the new_gps_data_available to indicate that there is new data available.
        new_gps_data_counter--;     // Decrement the new_gps_data_counter so there will only be 9 simulations
        gps_add_counter = 2;        // Set the gps_add_counter variable to 5 as a count down loop timer

        lat_gps_add += lat_gps_loop_add; // Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
        if (abs(lat_gps_add) >= 1)
        {                                    // If the absolute value of lat_gps_add is larger then 1.
            l_lat_gps += (int)lat_gps_add;   // Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
            lat_gps_add -= (int)lat_gps_add; // Subtract the lat_gps_add value as an integer so the decimal value remains.
        }

        lon_gps_add += lon_gps_loop_add; // Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
        if (abs(lon_gps_add) >= 1)
        {                                    // If the absolute value of lat_gps_add is larger then 1.
            l_lon_gps += (int)lon_gps_add;   // Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
            lon_gps_add -= (int)lon_gps_add; // Subtract the lat_gps_add value as an integer so the decimal value remains.
        }
    }

    if (new_gps_data_available)
    {
        new_gps_data_available = 0; // Reset the new_gps_data_available variable.

        if (flightMode >= 2 && waypoint_set == 0)
        {                               // If the flight mode is 3 (GPS hold) and no waypoints are set.
            waypoint_set = 1;           // Indicate that the waypoints are set.
            l_lat_waypoint = l_lat_gps; // Remember the current latitude as GPS hold waypoint.
            l_lon_waypoint = l_lon_gps; // Remember the current longitude as GPS hold waypoint.
        }

        if (flightMode >= 2 && waypoint_set == 1)
        { // If the GPS hold mode and the waypoints are stored.
            // GPS stick move adjustments
            // isHoldPosition_GPS = 1;
            if (flightMode == 2)
            {
                if (!latitude_north)
                {
                    l_lat_gps_float_adjust += 0.0015 * (((ReceiverValue[1] - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((ReceiverValue[0] - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); // South correction
                }
                else
                {
                    l_lat_gps_float_adjust -= 0.0015 * (((ReceiverValue[1] - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((ReceiverValue[0] - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); // North correction
                }

                if (!longiude_east)
                {
                    l_lon_gps_float_adjust -= (0.0015 * (((ReceiverValue[0] - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((ReceiverValue[1] - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); // West correction
                }

                else
                {
                    l_lon_gps_float_adjust += (0.0015 * (((ReceiverValue[0] - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((ReceiverValue[1] - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); // East correction
                }
            }

            if (l_lat_gps_float_adjust > 1)
            {
                l_lat_waypoint++;
                l_lat_gps_float_adjust--;
            }
            if (l_lat_gps_float_adjust < -1)
            {
                l_lat_waypoint--;
                l_lat_gps_float_adjust++;
            }

            if (l_lon_gps_float_adjust > 1)
            {
                l_lon_waypoint++;
                l_lon_gps_float_adjust--;
            }
            if (l_lon_gps_float_adjust < -1)
            {
                l_lon_waypoint--;
                l_lon_gps_float_adjust++;
            }
            // MODIFIED
            gps_lon_error = l_lon_waypoint - l_lon_gps;  // Calculate the latitude error between waypoint and actual position.
            gps_lat_error = -l_lat_gps + l_lat_waypoint; // Calculate the longitude error between waypoint and actual position.

            gps_lat_total_avarage -= gps_lat_rotating_mem[gps_rotating_mem_location];                 // Subtract the current memory position to make room for the new value.
            gps_lat_rotating_mem[gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous; // Calculate the new change between the actual pressure and the previous measurement.
            gps_lat_total_avarage += gps_lat_rotating_mem[gps_rotating_mem_location];                 // Add the new value to the long term avarage value.

            gps_lon_total_avarage -= gps_lon_rotating_mem[gps_rotating_mem_location];                 // Subtract the current memory position to make room for the new value.
            gps_lon_rotating_mem[gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous; // Calculate the new change between the actual pressure and the previous measurement.
            gps_lon_total_avarage += gps_lon_rotating_mem[gps_rotating_mem_location];                 // Add the new value to the long term avarage value.
            gps_rotating_mem_location++;                                                              // Increase the rotating memory location.
            if (gps_rotating_mem_location == 35)
                gps_rotating_mem_location = 0; // Start at 0 when the memory location 35 is reached.

            gps_lat_error_previous = gps_lat_error; // Remember the error for the next loop.
            gps_lon_error_previous = gps_lon_error; // Remember the error for the next loop.
            gps_lat_error_integral += gps_lat_error;
            gps_lon_error_integral += gps_lon_error;
            // Calculate the GPS pitch and roll correction as if the nose of the multicopter is facing north.
            // The Proportional part = (float)gps_lat_error * gps_p_gain.
            // The Derivative part = (float)gps_lat_total_avarage * gps_d_gain.
            gps_pitch_adjust_north = (float)gps_lat_error * gps_p_gain + (float)gps_lat_total_avarage * gps_d_gain;
            gps_roll_adjust_north = (float)gps_lon_error * gps_p_gain + (float)gps_lon_total_avarage * gps_d_gain;

            if (!latitude_north)
                gps_pitch_adjust_north *= -1; // Invert the pitch adjustmet because the quadcopter is flying south of the equator.
            if (!longiude_east)
                gps_roll_adjust_north *= -1; // Invert the roll adjustmet because the quadcopter is flying west of the prime meridian.

            // Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
            gps_roll_adjust = ((float)gps_roll_adjust_north * cos(AngleYaw * 0.017453)) - ((float)gps_pitch_adjust_north * cos((AngleYaw - 90) * 0.017453));
            gps_pitch_adjust = ((float)gps_pitch_adjust_north * cos(AngleYaw * 0.017453)) + ((float)gps_roll_adjust_north * cos((AngleYaw - 90) * 0.017453));

            // Limit the maximum correction to 300. This way we still have full controll with the pitch and roll stick on the transmitter.
            if (gps_roll_adjust > 300)
                gps_roll_adjust = 300;
            if (gps_roll_adjust < -300)
                gps_roll_adjust = -300;
            if (gps_pitch_adjust > 300)
                gps_pitch_adjust = 300;
            if (gps_pitch_adjust < -300)
                gps_pitch_adjust = -300;
        }
    }

    if (flightMode < 2 && waypoint_set > 0)
    {
        // isHoldPosition_GPS = 0;
        // If the GPS hold mode is disabled and the waypoints are set.
        gps_roll_adjust = 0;  // Reset the gps_roll_adjust variable to disable the correction.
        gps_pitch_adjust = 0; // Reset the gps_pitch_adjust variable to disable the correction.
        if (waypoint_set == 1)
        {                                  // If the waypoints are stored
            gps_rotating_mem_location = 0; // Set the gps_rotating_mem_location to zero so we can empty the
            waypoint_set = 2;              // Set the waypoint_set variable to 2 as an indication that the buffer is not cleared.
        }
        gps_lon_rotating_mem[gps_rotating_mem_location] = 0; // Reset the current gps_lon_rotating_mem location.
        gps_lat_rotating_mem[gps_rotating_mem_location] = 0; // Reset the current gps_lon_rotating_mem location.
        gps_rotating_mem_location++;                         // Increment the gps_rotating_mem_location variable for the next loop.
        if (gps_rotating_mem_location == 36)
        {                     // If the gps_rotating_mem_location equals 36, all the buffer locations are cleared.
            waypoint_set = 0; // Reset the waypoint_set variable to 0.
            // Reset the variables that are used for the D-controller.
            gps_lat_error_previous = 0;
            gps_lon_error_previous = 0;
            gps_lat_total_avarage = 0;
            gps_lon_total_avarage = 0;
            gps_rotating_mem_location = 0;
            // Reset the waypoints.
            l_lat_waypoint = 0;
            l_lon_waypoint = 0;
        }
    }
}

//----------------END GPS--------------------------------------------
void setup()
{
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Wire.setClock(400000);
    Wire.begin();
    delay(250);

    gps_setup();

    // IMU sensors
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    // pressure sensor
    //  Check if the MS5611 barometer is responding.
    Wire.beginTransmission(MS5611_address); // Start communication with the MS5611.
    Wire.endTransmission();                 // End the transmission and register the exit status.
    // For calculating the pressure the 6 calibration values need to be polled from the MS5611.
    // These 2 byte values are stored in the memory location 0xA2 and up.
    for (start = 1; start <= 6; start++)
    {
        Wire.beginTransmission(MS5611_address); // Start communication with the MPU-6050.
        Wire.write(0xA0 + start * 2);           // Send the address that we want to read.
        Wire.endTransmission();                 // End the transmission.

        Wire.requestFrom(MS5611_address, 2);       // Request 2 bytes from the MS5611.
        C[start] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the C[x] calibration variable.
    }
    OFF_C2 = C[2] * pow(2, 16);
    SENS_C1 = C[1] * pow(2, 15);

    for (start = 0; start < 100; start++)
    {                     // This loop runs 100 times.
        read_barometer(); // Read and calculate the barometer data.
        delay(4);         // The main program loop also runs 250Hz (4ms per loop).
    }
    actual_pressure = 0; // Reset the pressure calculations.

    // Start measurement altitude
    Wire.beginTransmission(GY_US42V2_ADDRESS);
    Wire.write(0x51); // Command to start measurement
    Wire.endTransmission();

    // compass sensor
    // setup_compass(); // Initiallize the compass and set the correct registers.
    // read_compass();  // Read and calculate the compass data.
    // AngleYaw = actual_compass_heading;
    if (!mag.begin())
    {
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
        while (1)
            ;
    }

    for (RateCalibrationNumber = 0;
         RateCalibrationNumber < 2000;
         RateCalibrationNumber++)
    {
        gyro_signals();
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
    analogWriteFrequency(9, 250);
    analogWriteFrequency(2, 250);
    analogWriteFrequency(3, 250);
    analogWriteFrequency(4, 250);
    analogWriteResolution(12);
    pinMode(6, OUTPUT);
    // digitalWrite(6, HIGH);
    battery_voltage();
    if (Voltage > 8.3)
    {
        digitalWrite(5, LOW);
        BatteryAtStart = BatteryDefault;
    }
    else if (Voltage < 7.5)
    {
        BatteryAtStart = 30 / 100 * BatteryDefault;
    }
    else
    {
        digitalWrite(5, LOW);
        BatteryAtStart = (82 * Voltage - 580) / 100 * BatteryDefault;
    }
    ReceiverInput.begin(14);

    // while (ReceiverValue[2] < 1020 ||
    //        ReceiverValue[2] > 1050)
    // {
    //     read_receiver();
    //     delay(4);
    // }
    LoopTimer = micros();
    myTimer.begin(timerRun, 1000);
}
void loop()
{
    gyro_signals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    calibAcc();
    AngleRoll = atan(AccXYZ[1][0] / sqrt(AccXYZ[0][0] * AccXYZ[0][0] + AccXYZ[2][0] * AccXYZ[2][0])) * 1 / (PI / 180);
    AnglePitch = -atan(AccXYZ[0][0] / sqrt(AccXYZ[1][0] * AccXYZ[1][0] + AccXYZ[2][0] * AccXYZ[2][0])) * 1 / (PI / 180);
    // if (isFirstCalibration)
    // {
    //     isFirstCalibration = false;
    //     rollInitial = AngleRoll;
    //     pitchInitial = AnglePitch;
    // }
    AngleRoll -= 0.65;
    AnglePitch -= 0.25;
    AnglePitch -= AngleRoll * sin((float)RateYaw * 0.000001066); // If the IMU has yawed transfer the roll angle to the pitch angel.
    AngleRoll += AnglePitch * sin((float)RateYaw * 0.000001066);
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    read_receiver();
    read_barometer();

    if (gps_add_counter >= 0)
        gps_add_counter--;
    read_gps();

    // read_compass();
    // AngleYaw += (float)RateYaw * 0.0000611; // Calculate the traveled yaw angle and add this to the angle_yaw variable.
    // if (AngleYaw < 0)
    //     AngleYaw += 360; // If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
    // else if (AngleYaw >= 360)
    //     AngleYaw -= 360;
    sensors_event_t event;
    mag.getEvent(&event);
    MagXYZ[0][0] = event.magnetic.x;
    MagXYZ[1][0] = event.magnetic.y;
    MagXYZ[2][0] = event.magnetic.z;
    calibMag();
    calibMagLv2();
    calibMagLv3();
    calibMagLv4();
    MagXYZ[0][0] *= -1;
    MagXYZ[1][0] *= -1;
    // MagXYZ[2][0] *= -1;

    compass_x_horizontal = (float)MagXYZ[1][0] * cos(KalmanAnglePitch * (PI / 180)) + (float)MagXYZ[0][0] * sin(KalmanAngleRoll * PI / 180) * sin(KalmanAnglePitch * (PI / 180)) - (float)MagXYZ[2][0] * cos(KalmanAngleRoll * PI / 180) * sin(KalmanAnglePitch * (PI / 180));
    compass_y_horizontal = (float)MagXYZ[0][0] * cos(KalmanAngleRoll * PI / 180) + (float)MagXYZ[2][0] * sin(KalmanAngleRoll * PI / 180);
    if (compass_y_horizontal < 0)
        actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / PI)));
    else
        actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / PI);
    // if (compass_y_horizontal < 0)
    //     actual_compass_heading = 180 + (180 + ((atan2(compass_x_horizontal, compass_y_horizontal)) * (180 / PI)));
    // else
    //     actual_compass_heading = (atan2(compass_x_horizontal, compass_y_horizontal)) * (180 / PI);
    // actual_compass_heading = 360 - actual_compass_heading;

    actual_compass_heading += declination; // Add the declination to the magnetic compass heading to get the geographic north.
    if (actual_compass_heading < 0)
        actual_compass_heading += 360; // If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
    else if (actual_compass_heading >= 360)
        actual_compass_heading -= 360; // If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
    AngleYaw += RateYaw * 0.0000611;
    if (AngleYaw < 0)
        AngleYaw += 360;
    else if (AngleYaw >= 360)
        AngleYaw -= 360;
    AngleYaw -= course_deviation(AngleYaw, actual_compass_heading) / 100.0; // Calculate the difference between the gyro and compass heading and make a small correction.
    if (AngleYaw < 0)
        AngleYaw += 360; // If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
    else if (AngleYaw >= 360)
        AngleYaw -= 360;
    gps_man_adjust_heading = AngleYaw;

    if (1)
    {
        // print into serial for testing
        // Serial.print("Altitude: ");
        // Serial.println(altitude);
        Serial.print("roll: ");
        Serial.print(KalmanAngleRoll);
        Serial.print("  pitch: ");
        Serial.print(KalmanAnglePitch);
        Serial.print("  yaw: ");
        Serial.print(AngleYaw);
        Serial.print("  Pres: ");
        Serial.print(actual_pressure);
        // Serial.print("  Pres point: ");
        // Serial.print(DesiredAltitude_pressure);
        Serial.print("  Lat: ");
        Serial.print(lat_gps_actual);
        Serial.print("  Lon: ");
        Serial.print(lon_gps_actual);
        // Serial.print("  fix type: ");
        // Serial.print(fix_type);
        Serial.print("  sats: ");
        Serial.print(number_used_sats);
        Serial.print("  mode: ");
        Serial.print(flightMode);
        Serial.print("  Hold Pos: ");
        Serial.print(isHoldPosition_GPS);
        Serial.print("  lat waypoint: ");
        Serial.print(l_lat_waypoint);
        Serial.print("  lon waypoint: ");
        Serial.println(l_lon_waypoint);
        // Serial.print(" Roll [µs]: ");
        // Serial.print(ReceiverValue[0]);
        // Serial.print(" Pitch [µs]: ");
        // Serial.print(ReceiverValue[1]);
        // Serial.print(" Throttle [µs]: ");
        // Serial.print(ReceiverValue[2]);
        // Serial.print(" Yaw [µs]: ");
        // Serial.println(ReceiverValue[3]);
    }

    InputThrottle = ReceiverValue[2];
    DesiredAngleRoll = 0.15 * (ReceiverValue[0] - 1500);  // 1495
    DesiredAnglePitch = 0.15 * (ReceiverValue[1] - 1500); // 1505
    DesiredRateYaw = 0.0 * (ReceiverValue[3] - 1500);
    // Hold altitude use ultra-sonic sensor
    // if (measureDistanceFlag)
    // {
    //     measureDistanceFlag = false;
    //     altitude = getAltitude();
    // }
    if (ReceiverValue[4] < 1600 && ReceiverValue[5] < 1600)
    {
        flightMode = 0; // mannual flight mode
    }
    if (ReceiverValue[4] > 1600 && ReceiverValue[5] < 1600)
    {
        flightMode = 1; // hold altitude use pressure sensor
    }
    if (ReceiverValue[4] < 1600 && ReceiverValue[5] > 1600)
    {
        flightMode = 2; // hold altitude and position use pressure and gps sensor
    }
    if (ReceiverValue[4] > 1600 && ReceiverValue[5] > 1600)
    {
        flightMode = 2;
    }
    if (flightMode == 0)
    {
        holdAltitudeFlag_sonar = true;
        holdAltitudeFlag_pressure = true;
        TempInputThrottle = InputThrottle;
    }
    // if (flightMode == 1)
    // {
    //     if (holdAltitudeFlag_sonar)
    //     {
    //         DesiredAltitude = getAltitude();
    //         holdAltitudeFlag_sonar = false;
    //     }
    //     ErrorAltitude = DesiredAltitude - altitude;
    //     pid_equation_altitude(ErrorAltitude, P_Altitude, I_Altitude, D_Altitude, PrevErrorAltitude, PrevItermAltitude);
    //     InputThrottle = TempInputThrottle + PIDReturn[0];
    //     PrevErrorAltitude = PIDReturn[1];
    //     PrevItermAltitude = PIDReturn[2];
    // }
    if (flightMode == 1)
    {
        if (holdAltitudeFlag_pressure)
        {
            DesiredAltitude_pressure = actual_pressure;
            holdAltitudeFlag_pressure = false;
        }
        ErrorAltitude_pressure = actual_pressure - DesiredAltitude_pressure;
        pid_equation_pressure(ErrorAltitude_pressure, P_Altitude_pressure, I_Altitude_pressure, D_Altitude_pressure, PrevErrorAltitude_pressure, PrevItermAltitude_pressure);
        InputThrottle = TempInputThrottle + PIDReturn[0];
        PrevErrorAltitude_pressure = PIDReturn[1];
        PrevItermAltitude_pressure = PIDReturn[2];
    }
    if (flightMode == 2)
    {
        // hold altitude
        if (holdAltitudeFlag_pressure)
        {
            DesiredAltitude_pressure = actual_pressure;
            holdAltitudeFlag_pressure = false;
        }
        ErrorAltitude_pressure = actual_pressure - DesiredAltitude_pressure;
        pid_equation_pressure(ErrorAltitude_pressure, P_Altitude_pressure, I_Altitude_pressure, D_Altitude_pressure, PrevErrorAltitude_pressure, PrevItermAltitude_pressure);
        InputThrottle = TempInputThrottle + PIDReturn[0];
        PrevErrorAltitude_pressure = PIDReturn[1];
        PrevItermAltitude_pressure = PIDReturn[2];

        // hold position
        DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500 + gps_roll_adjust);
        DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500 + gps_pitch_adjust);
        if (DesiredAngleRoll > 50)
            DesiredAngleRoll = 50;
        if (DesiredAngleRoll < -50)
            DesiredAngleRoll = -50;
        if (DesiredAnglePitch > 50)
            DesiredAnglePitch = 50;
        if (DesiredAnglePitch < -50)
            DesiredAnglePitch = -50;
    }
    ErrorAngleRoll = (DesiredAngleRoll - KalmanAngleRoll);
    ErrorAnglePitch = (DesiredAnglePitch - KalmanAnglePitch);

    // angle control
    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
    DesiredRateRoll = PIDReturn[0];
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];

    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
    DesiredRatePitch = PIDReturn[0];
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];

    // rate control

    ErrorRateRoll = (DesiredRateRoll - RateRoll);
    ErrorRatePitch = (DesiredRatePitch - RatePitch);
    ErrorRateYaw = (DesiredRateYaw - RateYaw);
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];
    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch = PIDReturn[0];
    PrevErrorRatePitch = PIDReturn[1];
    PrevItermRatePitch = PIDReturn[2];
    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
    InputYaw = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[1];
    PrevItermRateYaw = PIDReturn[2];

    if (InputThrottle > 1800)
        InputThrottle = 1800;
    MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
    MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
    MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
    MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);
    // MotorInput1 = 1.024 * (InputThrottle + InputRoll - InputPitch - InputYaw);
    // MotorInput2 = 1.024 * (InputThrottle + InputRoll + InputPitch + InputYaw);
    // MotorInput3 = 1.024 * (InputThrottle - InputRoll + InputPitch - InputYaw);
    // MotorInput4 = 1.024 * (InputThrottle - InputRoll - InputPitch + InputYaw);
    if (MotorInput1 > 2000)
        MotorInput1 = 1999;
    if (MotorInput2 > 2000)
        MotorInput2 = 1999;
    if (MotorInput3 > 2000)
        MotorInput3 = 1999;
    if (MotorInput4 > 2000)
        MotorInput4 = 1999;
    int ThrottleIdle = 1180;
    if (MotorInput1 < ThrottleIdle)
        MotorInput1 = ThrottleIdle;
    if (MotorInput2 < ThrottleIdle)
        MotorInput2 = ThrottleIdle;
    if (MotorInput3 < ThrottleIdle)
        MotorInput3 = ThrottleIdle;
    if (MotorInput4 < ThrottleIdle)
        MotorInput4 = ThrottleIdle;
    int ThrottleCutOff = 1000;
    if (ReceiverValue[2] < 1050)
    {
        MotorInput1 = ThrottleCutOff;
        MotorInput2 = ThrottleCutOff;
        MotorInput3 = ThrottleCutOff;
        MotorInput4 = ThrottleCutOff;
        reset_pid();
    }
    analogWrite(9, MotorInput1);
    analogWrite(2, MotorInput2);
    analogWrite(3, MotorInput3);
    analogWrite(4, MotorInput4);
    battery_voltage();
    CurrentConsumed = Current * 1000 * 0.004 / 3600 + CurrentConsumed;
    BatteryRemaining = (BatteryAtStart - CurrentConsumed) / BatteryDefault * 100;
    if (BatteryRemaining <= 30)
        digitalWrite(5, HIGH);
    else
        digitalWrite(5, LOW);
    while (micros() - LoopTimer < 4000)
        ;
    LoopTimer = micros();
}