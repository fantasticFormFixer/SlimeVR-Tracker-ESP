/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain, S.J. Remington & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "mpu9250sensor.h"
#include "network/network.h"
#include "globals.h"
#include "helper_3dmath.h"
#include <i2cscan.h>
#include "calibration.h"
#include "magneto1.4.h"
#include "GlobalVars.h"

#if !(defined(MPU9250_USE_MAHONY) && MPU9250_USE_MAHONY)
    #include "dmpmag.h"
#else
constexpr float gscale = (250. / 32768.0) * (PI / 180.0); //gyro default 250 LSB per d/s -> rad/s
#endif

#define MAG_CORR_RATIO 0.02

void MPU9250Sensor::motionSetup() {
    // initialize device
    imu.initialize(addr);
    if(!imu.testConnection()) {
        m_Logger.fatal("Can't connect to MPU9250 (0x%02x) at address 0x%02x", imu.getDeviceID(), addr);
        return;
    }

    m_Logger.info("Connected to MPU9250 (0x%02x) at address 0x%02x", imu.getDeviceID(), addr);

    int16_t ax,ay,az;

    // turn on while flip back to calibrate. then, flip again after 5 seconds.
    // TODO: Move calibration invoke after calibrate button on slimeVR server available
    imu.getAcceleration(&ax, &ay, &az);
    float g_az = (float)az / 16384; // For 2G sensitivity
    if(g_az < -0.75f) {
        ledManager.on();
        m_Logger.info("Flip front to confirm start calibration");
        delay(5000);
        ledManager.off();

        imu.getAcceleration(&ax, &ay, &az);
        g_az = (float)az / 16384;
        if(g_az > 0.75f) {
            m_Logger.debug("Starting calibration...");
            startCalibration(0);
        }
    }

    // Initialize the configuration
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type) {
        case SlimeVR::Configuration::CalibrationConfigType::MPU9250:
            m_Calibration = sensorCalibration.data.mpu9250;
            break;

        case SlimeVR::Configuration::CalibrationConfigType::NONE:
            m_Logger.warn("No calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
            break;

        default:
            m_Logger.warn("Incompatible calibration data found for sensor %d, ignoring...", sensorId);
            m_Logger.info("Calibration is advised");
        }
    }

#if defined(MPU9250_USE_MAHONY) && MPU9250_USE_MAHONY
    this->working = true;
    this->configured = true;
#else
    devStatus = imu.dmpInitialize();
    if(devStatus == 0){
        ledManager.pattern(50, 50, 5);

        // turn on the DMP, now that it's ready
        m_Logger.debug("Enabling DMP...");
        imu.setDMPEnabled(true);

        // TODO: Add interrupt support
        // mpuIntStatus = imu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        m_Logger.debug("DMP ready! Waiting for first interrupt...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();
        working = true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        m_Logger.error("DMP Initialization failed (code %d)", devStatus);
    }
#endif
}

void MPU9250Sensor::motionLoop() {
#if ENABLE_INSPECTION
    {
        int16_t rX, rY, rZ, aX, aY, aZ, mX, mY, mZ;
        imu.getRotation(&rX, &rY, &rZ);
        imu.getAcceleration(&aX, &aY, &aZ);
        imu.getMagnetometer(&mX, &mY, &mZ);

        Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, mX, mY, mZ, 255);
    }
#endif

#if defined(MPU9250_USE_MAHONY) && MPU9250_USE_MAHONY
    unsigned long now = micros();
    unsigned long deltat = now - last; //seconds since last update

    this->last = now;

    getMPUScaled();
    // this->m_Logger.debug("A: %f\t%f\t%f\tG: %f\t%f\t%f\tM: %f\t%f\t%f", UNPACK_VECTOR_ARRAY(this->A.array), UNPACK_VECTOR_ARRAY(this->G.array), UNPACK_VECTOR_ARRAY(this->M.array));

    FusionAhrsUpdate(&this->ahrs, this->G, this->A, this->M, deltat);

    FusionQuaternion q = FusionAhrsGetQuaternion(&this->ahrs);
    FusionEuler e = FusionQuaternionToEuler(q);
    this->m_Logger.debug("%f\t%f\t%f\t%f\t->\t%f\t%f\t%f", UNPACK_QUATERNION_ARRAY(q.array), UNPACK_VECTOR_ARRAY(e.array));

    this->dmpQuat = FusionQuaternionMultiply(q, this->dmpSensorOffset);
    this->quaternion.set(this->dmpQuat.element.x, this->dmpQuat.element.y, this->dmpQuat.element.z, this->dmpQuat.element.w);
#else
    // Update quaternion
    if(!dmpReady)
        return;
    Quaternion rawQuat{};
    if(!imu.GetCurrentFIFOPacket(fifoBuffer,imu.dmpGetFIFOPacketSize())) return;
    if(imu.dmpGetQuaternion(&rawQuat, fifoBuffer)) return; // FIFO CORRUPTED
    Quat quat(-rawQuat.y,rawQuat.x,rawQuat.z,rawQuat.w);

    getMPUScaled();

    if (Mxyz[0] == 0.0f && Mxyz[1] == 0.0f && Mxyz[2] == 0.0f) {
        return;
    }

    VectorFloat grav;
    imu.dmpGetGravity(&grav, &rawQuat);

    float Grav[] = {grav.x, grav.y, grav.z};

    if (correction.length_squared() == 0.0f) {
        correction = getCorrection(Grav, Mxyz, quat);
    } else {
        Quat newCorr = getCorrection(Grav, Mxyz, quat);

        if(!__isnanf(newCorr.w)) {
            correction = correction.slerp(newCorr, MAG_CORR_RATIO);
        }
    }

    quaternion = correction * quat;
    quaternion *= sensorOffset;
#endif

#if ENABLE_INSPECTION
    {
        Network::sendInspectionFusedIMUData(sensorId, quaternion);
    }
#endif

    if(!lastQuatSent.equalsWithEpsilon(quaternion)) {
        newData = true;
        lastQuatSent = quaternion;
    }
}

void MPU9250Sensor::getMPUScaled()
{
    // float temp[3];
    // int i;

#if defined(MPU9250_USE_MAHONY) && MPU9250_USE_MAHONY
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

    // imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    imu.getRotation(&gx, &gy, &gz);
    imu.getAcceleration(&ax, &ay, &az);
    imu.getMagnetometer(&mx, &my, &mz);

    this->G.array[0] = (float)gx;
    this->G.array[1] = (float)gy;
    this->G.array[2] = (float)gz;

    // this->G.array[0] = ((float)gx - this->m_Calibration.G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    // this->G.array[1] = ((float)gy - this->m_Calibration.G_off[1]) * gscale;
    // this->G.array[2] = ((float)gz - this->m_Calibration.G_off[2]) * gscale;

    this->A.array[0] = (float)ax;
    this->A.array[1] = (float)ay;
    this->A.array[2] = (float)az;

    //apply offsets (bias) and scale factors from Magneto
    // #if useFullCalibrationMatrix == true
    //     for (i = 0; i < 3; i++)
    //         temp[i] = (this->A.array[i] - this->m_Calibration.A_B[i]);
    //     this->A.array[0] = this->m_Calibration.A_Ainv[0][0] * temp[0] + this->m_Calibration.A_Ainv[0][1] * temp[1] + this->m_Calibration.A_Ainv[0][2] * temp[2];
    //     this->A.array[1] = this->m_Calibration.A_Ainv[1][0] * temp[0] + this->m_Calibration.A_Ainv[1][1] * temp[1] + this->m_Calibration.A_Ainv[1][2] * temp[2];
    //     this->A.array[2] = this->m_Calibration.A_Ainv[2][0] * temp[0] + this->m_Calibration.A_Ainv[2][1] * temp[1] + this->m_Calibration.A_Ainv[2][2] * temp[2];
    // #else
    //     for (i = 0; i < 3; i++)
    //         this->A.array[i] = (this->A.array[i] - this->m-Calibration.A_B[i]);
    // #endif

    // Orientations of axes are set in accordance with the datasheet
    // See Section 9.1 Orientation of Axes
    // https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
    this->M.array[0] = (float)my;
    this->M.array[1] = (float)mx;
    this->M.array[2] = -(float)mz;
    //apply offsets and scale factors from Magneto
    // #if useFullCalibrationMatrix == true
    //     for (i = 0; i < 3; i++)
    //         temp[i] = (this->M.array[i] - this->m_Calibration.M_B[i]);
    //     this->M.array[0] = this->m_Calibration.M_Ainv[0][0] * temp[0] + this->m_Calibration.M_Ainv[0][1] * temp[1] + this->m_Calibration.M_Ainv[0][2] * temp[2];
    //     this->M.array[1] = this->m_Calibration.M_Ainv[1][0] * temp[0] + this->m_Calibration.M_Ainv[1][1] * temp[1] + this->m_Calibration.M_Ainv[1][2] * temp[2];
    //     this->M.array[2] = this->m_Calibration.M_Ainv[2][0] * temp[0] + this->m_Calibration.M_Ainv[2][1] * temp[1] + this->m_Calibration.M_Ainv[2][2] * temp[2];
    // #else
    //     for (i = 0; i < 3; i++)
    //         this->M.array[i] = (this->M.array[i] - this->m_Calibration.M_B[i]);
    // #endif
#else
    int16_t mx, my, mz;
    // with DMP, we just need mag data
    imu.getMagnetometer(&mx, &my, &mz);

    // Orientations of axes are set in accordance with the datasheet
    // See Section 9.1 Orientation of Axes
    // https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
    Mxyz[0] = (float)my;
    Mxyz[1] = (float)mx;
    Mxyz[2] = -(float)mz;
    //apply offsets and scale factors from Magneto
    #if useFullCalibrationMatrix == true
        for (i = 0; i < 3; i++)
            temp[i] = (Mxyz[i] - m_Calibration.M_B[i]);
        Mxyz[0] = m_Calibration.M_Ainv[0][0] * temp[0] + m_Calibration.M_Ainv[0][1] * temp[1] + m_Calibration.M_Ainv[0][2] * temp[2];
        Mxyz[1] = m_Calibration.M_Ainv[1][0] * temp[0] + m_Calibration.M_Ainv[1][1] * temp[1] + m_Calibration.M_Ainv[1][2] * temp[2];
        Mxyz[2] = m_Calibration.M_Ainv[2][0] * temp[0] + m_Calibration.M_Ainv[2][1] * temp[1] + m_Calibration.M_Ainv[2][2] * temp[2];
    #else
        for (i = 0; i < 3; i++)
            Mxyz[i] = (Mxyz[i] - m_Calibration.M_B[i]);
    #endif
#endif
}

void MPU9250Sensor::startCalibration(int calibrationType) {
    ledManager.on();

#if defined(MPU9250_USE_MAHONY) && MPU9250_USE_MAHONY
    m_Logger.debug("Gathering raw data for device calibration...");
    constexpr int calibrationSamples = 300;
    // Reset values
    this->G = {};

    // Wait for sensor to calm down before calibration
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    for (int i = 0; i < calibrationSamples; i++)
    {
        int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        this->G.array[0] += float(gx);
        this->G.array[1] += float(gy);
        this->G.array[2] += float(gz);
    }
    this->G.array[0] /= calibrationSamples;
    this->G.array[1] /= calibrationSamples;
    this->G.array[2] /= calibrationSamples;

#ifdef DEBUG_SENSOR
    m_Logger.trace("Gyro calibration results: %f %f %f", Gxyz[0], Gxyz[1], Gxyz[2]);
#endif

    Network::sendRawCalibrationData(this->G.array, CALIBRATION_TYPE_EXTERNAL_GYRO, 0);
    m_Calibration.G_off[0] = this->G.array[0];
    m_Calibration.G_off[1] = this->G.array[1];
    m_Calibration.G_off[2] = this->G.array[2];

    // Blink calibrating led before user should rotate the sensor
    m_Logger.info("Gently rotate the device while it's gathering accelerometer and magnetometer data");
    ledManager.pattern(15, 300, 3000/310);
    float *calibrationDataAcc = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    float *calibrationDataMag = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < calibrationSamples; i++) {
        ledManager.on();
        int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        calibrationDataAcc[i * 3 + 0] = ax;
        calibrationDataAcc[i * 3 + 1] = ay;
        calibrationDataAcc[i * 3 + 2] = az;
        calibrationDataMag[i * 3 + 0] = my;
        calibrationDataMag[i * 3 + 1] = mx;
        calibrationDataMag[i * 3 + 2] = -mz;
        Network::sendRawCalibrationData(calibrationDataAcc, CALIBRATION_TYPE_EXTERNAL_ACCEL, 0);
        Network::sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
        ledManager.off();
        delay(250);
    }
    m_Logger.debug("Calculating calibration data...");

    float A_BAinv[4][3];
    float M_BAinv[4][3];
    CalculateCalibration(calibrationDataAcc, calibrationSamples, A_BAinv);
    free(calibrationDataAcc);
    CalculateCalibration(calibrationDataMag, calibrationSamples, M_BAinv);
    free(calibrationDataMag);
    m_Logger.debug("Finished Calculate Calibration data");
    m_Logger.debug("Accelerometer calibration matrix:");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++)
    {
        m_Calibration.A_B[i] = A_BAinv[0][i];
        m_Calibration.A_Ainv[0][i] = A_BAinv[1][i];
        m_Calibration.A_Ainv[1][i] = A_BAinv[2][i];
        m_Calibration.A_Ainv[2][i] = A_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", A_BAinv[0][i], A_BAinv[1][i], A_BAinv[2][i], A_BAinv[3][i]);
    }
    m_Logger.debug("}");
    m_Logger.debug("[INFO] Magnetometer calibration matrix:");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.M_B[i] = M_BAinv[0][i];
        m_Calibration.M_Ainv[0][i] = M_BAinv[1][i];
        m_Calibration.M_Ainv[1][i] = M_BAinv[2][i];
        m_Calibration.M_Ainv[2][i] = M_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    }
    m_Logger.debug("}");
#else
    // with DMP, we just need mag data
    constexpr int calibrationSamples = 300;

    // Blink calibrating led before user should rotate the sensor
    m_Logger.info("Gently rotate the device while it's gathering magnetometer data");
    ledManager.pattern(15, 300, 3000/310);
    float *calibrationDataMag = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < calibrationSamples; i++) {
        ledManager.on();
        int16_t mx,my,mz;
        imu.getMagnetometer(&mx, &my, &mz);
        calibrationDataMag[i * 3 + 0] = my;
        calibrationDataMag[i * 3 + 1] = mx;
        calibrationDataMag[i * 3 + 2] = -mz;
        Network::sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
        ledManager.off();
        delay(250);
    }
    m_Logger.debug("Calculating calibration data...");

    float M_BAinv[4][3];
    CalculateCalibration(calibrationDataMag, calibrationSamples, M_BAinv);
    free(calibrationDataMag);

    m_Logger.debug("[INFO] Magnetometer calibration matrix:");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.M_B[i] = M_BAinv[0][i];
        m_Calibration.M_Ainv[0][i] = M_BAinv[1][i];
        m_Calibration.M_Ainv[1][i] = M_BAinv[2][i];
        m_Calibration.M_Ainv[2][i] = M_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    }
    m_Logger.debug("}");
#endif

    m_Logger.debug("Saving the calibration data");

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::MPU9250;
    calibration.data.mpu9250 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    ledManager.off();
    Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
    m_Logger.debug("Saved the calibration data");

    m_Logger.info("Calibration data gathered");
}
