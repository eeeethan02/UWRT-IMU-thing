/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include "Kalman.h"
#include <cmath>
#include <map>
#include <string>
#include <chrono>
#include <thread>
#include <fstream>
#include <istream>
#include <iostream>

using namespace std;

// constexpr float M_PI = 3.1415925F;

Kalman::Kalman()
{
    /* We will set the variables like so, these can also be tuned by the user */
    Q_angle = 0.001f; // f means it's a float and not a double to save memory
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f; // Reset the angle
    bias = 0.0f;  // Reset bias
    rate = 0.0f;

    P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

Kalman::Kalman(float Qangle, float Qbias, float Rmeasure, float startAngle)
{
    Q_angle = Qangle;
    Q_bias = Qbias;
    R_measure = Rmeasure;

    angle = startAngle;
    P[0][0] = 0.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman::filterSync(float newAngle, float newRate, float dt)
{
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    // Using 2d array to represent 2x2 matrix P (P[row][column])
    /* Step 2 */
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};

void Kalman::filterAsync(std::ifstream &gyroCSV, std::ifstream &accelCSV, std::ofstream &filteredCSV)
{
    if (!gyroCSV)
    {
        cout << "Gyro file not open" << endl;
        return;
    };
    if (!accelCSV)
    {
        cout << "Accelerometer file not open" << endl;
        return;
    };
    if (!filteredCSV)
    {
        cout << "Filtered file not open" << endl;
        return;
    };
    float time = 0.0, temp = 0.0, noisyData = 0.0, cleanData = 0.0, filteredData = 0.0;
    string line;
    getline(gyroCSV, line);
    getline(accelCSV, line);
    cout << line << endl;
    filteredCSV << "time, filtered sine wave" << endl;
    while (gyroCSV >> time && accelCSV >> temp)
    {
        int comma = ',';
        gyroCSV.ignore(1, comma);
        accelCSV.ignore(1, comma);
        gyroCSV >> noisyData;
        accelCSV >> cleanData;
        filteredData = this->filterSync(noisyData, cleanData, this->timeStep);
        filteredCSV << time << ", " << filteredData << endl;
    }
    cout << "done";
    return;
}

void Kalman::setAngle(float angle)
{
    this->angle = angle;
}; // Used to set angle, this should be set as the starting angle

float Kalman::getRate() { return this->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void Kalman::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void Kalman::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

void Kalman::settimeStep(float time_step) { this->timeStep = time_step; };

float Kalman::getQangle() { return this->Q_angle; };
float Kalman::getQbias() { return this->Q_bias; };
float Kalman::getRmeasure() { return this->R_measure; };

float Kalman::gettimeStep() { return this->timeStep; };

void csvTest()
{
    Kalman filterX(0.001, 0.003, 0.03, 0.0);
    ifstream noiseReader("C:\\Users\\eeeet\\OneDrive\\Desktop\\UWRobotics\\Kalman Filter\\noisy_sine_wave.csv");
    ifstream cleanReader("C:\\Users\\eeeet\\OneDrive\\Desktop\\UWRobotics\\Kalman Filter\\clean_sine_wave.csv");
    ofstream writer("C:\\Users\\eeeet\\OneDrive\\Desktop\\UWRobotics\\Kalman Filter\\filtered_sine_wave.csv");
    filterX.filterAsync(noiseReader, cleanReader, writer);
    writer.close();
    return;
}

int main()
{
    csvTest();
    return 1;
}
