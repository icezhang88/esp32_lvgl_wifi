//
// Created by icezhang on 2023-01-04.
//

#ifndef GATT_SERVER_DEMOS_KALM_FILTER_H
#define GATT_SERVER_DEMOS_KALM_FILTER_H


#include <stdio.h>
#include "mpu6050_driver.h"
#include "freertos/FreeRTOS.h"
#include "math.h"
#include "time.h"
#include "esp_log.h"
#define RAD_TO_DEG 57.295779513082320876798154814105
const double Accel_Z_corrector = 14418.0;
uint32_t timer;

typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;
Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};


double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

#endif //GATT_SERVER_DEMOS_KALM_FILTER_H
