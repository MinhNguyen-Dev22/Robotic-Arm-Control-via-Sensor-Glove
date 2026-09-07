/**
 * DỰ ÁN: GĂNG TAY ĐIỀU KHIỂN ROBOT - PIPELINE 10.4 (ADAPTIVE ORTHOGONAL ZUPT EDITION)
 * Cải tiến:
 * 1. Trực chuẩn hóa Vai (S1, S2) bằng Gram-Schmidt.
 * 2. Bộ lọc trạng thái EMA ZUPT chống rung/nhiễu vi cấp.
 * 3. Tăng tốc độ bù Drift Heading khi ở trạng thái tĩnh.
 * 4. Phân ly động học Cổ tay hoàn toàn khỏi Cẳng tay thông qua ZUPT Gating.
 */

#include <Wire.h>
#ifndef ICM_20948_USE_DMP
#define ICM_20948_USE_DMP
#endif
#include "ICM_20948.h"
#include <esp_now.h>
#include <WiFi.h>

// ==============================================================================
// ĐỊNH NGHĨA CHÂN GIAO TIẾP
// ==============================================================================
#define I2C_BUS0_SDA    21
#define I2C_BUS0_SCL    22
#define I2C_BUS1_SDA    19
#define I2C_BUS1_SCL    18
#define LED_PIN         32
#define FLEX_PIN        34
#define BOOT_BUTTON_PIN 0

#define I2CBus0 Wire
#define I2CBus1 Wire1
#define AD0_VCC 1
#define AD0_GND 0

// ==============================================================================
// CẤU TRÚC DỮ LIỆU
// ==============================================================================
struct Quat { float w, x, y, z; };
struct Vec3 { float x, y, z; }; 
struct Mat3x3 { float m[3][3]; };
struct JointMap { float rawMin, rawMax, servoMin, servoMax; };

JointMap mapS1 = { -90.0f,  90.0f,  10.0f, 150.0f }; 
JointMap mapS2 = { -90.0f,  90.0f,  65.0f, 135.0f }; 
JointMap mapS3 = {   0.0f, 140.0f,   0.0f,  90.0f }; 
JointMap mapS4 = {-150.0f,  85.0f,  60.0f, 140.0f }; 
JointMap mapS5 = { -95.0f,  35.0f,  40.0f, 170.0f }; 

const Vec3 ANAT_AXIS_X = {1.0f, 0.0f, 0.0f}; 
const Vec3 ANAT_AXIS_Y = {0.0f, 1.0f, 0.0f}; 
const Vec3 ANAT_AXIS_Z = {0.0f, 0.0f, 1.0f}; 

// ==============================================================================
// TOÁN HỌC KHÔNG GIAN 3D
// ==============================================================================
namespace Math3D {
    Quat quatInverse(Quat q) { return {q.w, -q.x, -q.y, -q.z}; }
    float quatDot(Quat q1, Quat q2) { return q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z; }
    float vecDot(Vec3 v1, Vec3 v2) { return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z; }
    Vec3 vecCross(Vec3 a, Vec3 b) { return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x }; }
    Vec3 vecScale(Vec3 v, float s) { return {v.x * s, v.y * s, v.z * s}; }
    float vecMagnitude(Vec3 v) { return sqrt(v.x*v.x + v.y*v.y + v.z*v.z); }
    
    void vecNormalize(Vec3 &v) {
        float n = vecMagnitude(v);
        if (n > 1e-6f) { v.x /= n; v.y /= n; v.z /= n; } else { v = {0,0,0}; }
    }
    
    void normalize(Quat &q) {
        float n = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
        if (n > 1e-6f) { q.w /= n; q.x /= n; q.y /= n; q.z /= n; } else { q = {1,0,0,0}; }
    }

    Quat quatMultiply(Quat q1, Quat q2) {
        Quat res;
        res.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
        res.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
        res.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
        res.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
        normalize(res); return res;
    }

    Vec3 rotateVector(Vec3 v, Quat q) {
        Quat vQ = {0, v.x, v.y, v.z};
        Quat res = quatMultiply(quatMultiply(q, vQ), quatInverse(q));
        return {res.x, res.y, res.z};
    }

    Quat nlerp(Quat qPrev, Quat qNew, float t) {
        if (quatDot(qPrev, qNew) < 0.0f) qNew = {-qNew.w, -qNew.x, -qNew.y, -qNew.z}; 
        Quat res = { qPrev.w + t*(qNew.w - qPrev.w), qPrev.x + t*(qNew.x - qPrev.x),
                     qPrev.y + t*(qNew.y - qPrev.y), qPrev.z + t*(qNew.z - qPrev.z) };
        normalize(res); return res;
    }

    Mat3x3 quatToMatrix(Quat q) {
        Mat3x3 m;
        float xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
        float xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
        float wx = q.w*q.x, wy = q.w*q.y, wz = q.w*q.z;
        m.m[0][0] = 1.0f - 2.0f*(yy + zz); m.m[0][1] = 2.0f*(xy - wz);        m.m[0][2] = 2.0f*(xz + wy);
        m.m[1][0] = 2.0f*(xy + wz);        m.m[1][1] = 1.0f - 2.0f*(xx + zz); m.m[1][2] = 2.0f*(yz - wx);
        m.m[2][0] = 2.0f*(xz - wy);        m.m[2][1] = 2.0f*(yz + wx);        m.m[2][2] = 1.0f - 2.0f*(xx + yy);
        return m;
    }

    Quat matrixToQuat(Mat3x3 m) {
        Quat q;
        float tr = m.m[0][0] + m.m[1][1] + m.m[2][2];
        if (tr > 0.0f) {
            float S = sqrt(tr + 1.0f) * 2.0f;
            q.w = 0.25f * S; q.x = (m.m[2][1] - m.m[1][2]) / S;
            q.y = (m.m[0][2] - m.m[2][0]) / S; q.z = (m.m[1][0] - m.m[0][1]) / S;
        } else if ((m.m[0][0] > m.m[1][1]) && (m.m[0][0] > m.m[2][2])) {
            float S = sqrt(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.0f;
            q.w = (m.m[2][1] - m.m[1][2]) / S; q.x = 0.25f * S;
            q.y = (m.m[0][1] + m.m[1][0]) / S; q.z = (m.m[0][2] + m.m[2][0]) / S;
        } else if (m.m[1][1] > m.m[2][2]) {
            float S = sqrt(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.0f;
            q.w = (m.m[0][2] - m.m[2][0]) / S; q.x = (m.m[0][1] + m.m[1][0]) / S;
            q.y = 0.25f * S; q.z = (m.m[1][2] + m.m[2][1]) / S;
        } else {
            float S = sqrt(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.0f;
            q.w = (m.m[1][0] - m.m[0][1]) / S; q.x = (m.m[0][2] + m.m[2][0]) / S;
            q.y = (m.m[1][2] + m.m[2][1]) / S; q.z = 0.25f * S;
        }
        normalize(q); return q;
    }

    Quat build3DOFMountQuat_GramSchmidt(Vec3 vHingePCA, Vec3 vGravityNPose, bool isHingeZ, const char* jointName = "") {
        vecNormalize(vHingePCA); vecNormalize(vGravityNPose);
        Mat3x3 R; Vec3 eX, eY, eZ; 
        if (isHingeZ) {
            eZ = vHingePCA;
            eX = vecCross(vecScale(vGravityNPose, -1.0f), eZ);
            if (vecMagnitude(eX) < 0.05f) eX = vecCross(ANAT_AXIS_X, eZ);
            vecNormalize(eX); eY = vecCross(eZ, eX); vecNormalize(eY);
        } else {
            eY = vHingePCA;
            eX = vecCross(eY, vGravityNPose);
            if (vecMagnitude(eX) < 0.05f) eX = vecCross(eY, ANAT_AXIS_Z);
            vecNormalize(eX); eZ = vecCross(eX, eY); vecNormalize(eZ);
        }
        R.m[0][0]=eX.x; R.m[0][1]=eY.x; R.m[0][2]=eZ.x;
        R.m[1][0]=eX.y; R.m[1][1]=eY.y; R.m[1][2]=eZ.y;
        R.m[2][0]=eX.z; R.m[2][1]=eY.z; R.m[2][2]=eZ.z;
        return matrixToQuat(R);
    }

    Quat buildHandMount_GramSchmidt(Vec3 vTwist, Vec3 vSwing, const char* jointName = "") {
        vecNormalize(vTwist);
        float proj = vecDot(vSwing, vTwist);
        Vec3 eX = { vSwing.x - proj*vTwist.x, vSwing.y - proj*vTwist.y, vSwing.z - proj*vTwist.z };
        vecNormalize(eX);
        Vec3 eY = vTwist; 
        Vec3 eZ = vecCross(eX, eY); vecNormalize(eZ);
        Mat3x3 R;
        R.m[0][0]=eX.x; R.m[0][1]=eY.x; R.m[0][2]=eZ.x;
        R.m[1][0]=eX.y; R.m[1][1]=eY.y; R.m[1][2]=eZ.y;
        R.m[2][0]=eX.z; R.m[2][1]=eY.z; R.m[2][2]=eZ.z;
        return matrixToQuat(R);
    }

    void decomposeSwingTwist(Quat q, Vec3 axis, Quat &swing, Quat &twist) {
        Vec3 v = {q.x, q.y, q.z};
        float proj = vecDot(v, axis);
        Vec3 vTwist = vecScale(axis, proj);
        twist = {q.w, vTwist.x, vTwist.y, vTwist.z};
        normalize(twist);
        swing = quatMultiply(q, quatInverse(twist));
    }
    
    float getTwistAngle(Quat qTwist, Vec3 axis) {
        float angle = 2.0f * atan2(sqrt(qTwist.x*qTwist.x + qTwist.y*qTwist.y + qTwist.z*qTwist.z), qTwist.w) * 180.0f / PI;
        if (vecDot({qTwist.x, qTwist.y, qTwist.z}, axis) < 0.0f) angle = -angle;
        return angle;
    }
}

// ==============================================================================
// GIAO TIẾP DMP & HELPER
// ==============================================================================
void printQuat(const char* label, Quat q) { Serial.printf("%-20s : [W: %6.3f | X: %6.3f | Y: %6.3f | Z: %6.3f]\n", label, q.w, q.x, q.y, q.z); }
void printVec(const char* label, Vec3 v) { Serial.printf("%-20s : [X: %6.3f | Y: %6.3f | Z: %6.3f]\n", label, v.x, v.y, v.z); }

Quat dmpQuat9ToBodyQuat(icm_20948_DMP_data_t &data) {
    Quat q; const float Q30_SCALE = 1.0f / 1073741824.0f; 
    q.x = ((int32_t)data.Quat9.Data.Q1) * Q30_SCALE;
    q.y = ((int32_t)data.Quat9.Data.Q2) * Q30_SCALE;
    q.z = ((int32_t)data.Quat9.Data.Q3) * Q30_SCALE;
    q.w = ((int32_t)data.Quat9.Data.Q1 + 0) ? sqrtf(constrain(1.0f - (q.x*q.x + q.y*q.y + q.z*q.z), 0.0f, 1.0f)) : 1.0f;
    if (data.header & DMP_header_bitmap_Quat9) q.w = ((int32_t)data.Quat9.Data.Q1 == 0 && q.x == 0 && q.y == 0 && q.z == 0) ? 1.0f : sqrtf(constrain(1.0f - (q.x*q.x + q.y*q.y + q.z*q.z), 0.0f, 1.0f));
    Math3D::normalize(q); return q;
}

void initDMPDevice(ICM_20948_I2C &imu, TwoWire &wirePort, uint8_t ad0_val) {
    imu.begin(wirePort, ad0_val);
    if (imu.status != ICM_20948_Stat_Ok) return;
    imu.initializeDMP(); imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION);
    imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 1); imu.enableFIFO();
    imu.enableDMP(); imu.resetDMP(); imu.resetFIFO();
}

// ==============================================================================
// BIẾN TOÀN CỤC
// ==============================================================================
ICM_20948_I2C imuHand, imuForearm, imuUpperArm;
Quat rawUpper, rawForearm, rawHand, filteredUpper, filteredForearm, filteredHand;
Quat Q_Mount_Upper, Q_Mount_Forearm, Q_Mount_Hand;
Quat Home_Anat_Upper, Home_Anat_Forearm, Home_Anat_Hand;
Quat Home_Rel_Elbow, Home_Rel_Wrist;
Quat Q_DriftRel_Shoulder, Q_DriftRel_Elbow, Q_DriftRel_Wrist;

Vec3 dynAxis_S1_Yaw = ANAT_AXIS_Z;    
Vec3 dynAxis_S2_Pitch = ANAT_AXIS_Y;  
Vec3 dynAxis_S3_Elbow = ANAT_AXIS_Z;  
Vec3 dynAxis_S4_Twist = ANAT_AXIS_Y;  
Vec3 dynAxis_S5_Swing = ANAT_AXIS_X;  

// EMA ZUPT States
float emaGyrMagU = 0.0f, emaGyrMagF = 0.0f, emaGyrMagH = 0.0f;
float emaAccMagU = 1000.0f, emaAccMagF = 1000.0f;

float flexGripThreshold = 2700.0f; 
float flexReleaseThreshold = 2650.0f;
bool firstFrameAfterCalib = true; bool systemCalibrated = false; 

unsigned long lastSendTime = 0, lastDebugTime = 0, staticStartTime = 0;
float currentS1=0, currentS2=0, currentS3=0, currentS4=0, currentS5=0, currentS6=0;
const float DEADBAND_DEG = 1.2f, Q_FILTER_ALPHA = 0.4f;

typedef struct { float S1, S2, S3, S4, S5, S6; int flex; } RobotCommand;
RobotCommand txData;
uint8_t rxMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

unsigned long tHand = 0, tForearm = 0, tUpper = 0;
Quat rawHandBuf, rawForearmBuf, rawUpperBuf;

// ==============================================================================
// ĐỌC DỮ LIỆU ĐỒNG BỘ & CẬP NHẬT ZUPT STATES
// ==============================================================================
bool fetchSynchronizedIMUData() {
    unsigned long now = millis();
    if (imuHand.dataReady()) {
        icm_20948_DMP_data_t dmp; imuHand.readDMPdataFromFIFO(&dmp);
        if (imuHand.status == ICM_20948_Stat_Ok && (dmp.header & DMP_header_bitmap_Quat9)) { rawHandBuf = dmpQuat9ToBodyQuat(dmp); tHand = now; }
    }
    if (imuForearm.dataReady()) {
        icm_20948_DMP_data_t dmp; imuForearm.readDMPdataFromFIFO(&dmp);
        if (imuForearm.status == ICM_20948_Stat_Ok && (dmp.header & DMP_header_bitmap_Quat9)) { rawForearmBuf = dmpQuat9ToBodyQuat(dmp); tForearm = now; }
    }
    if (imuUpperArm.dataReady()) {
        icm_20948_DMP_data_t dmp; imuUpperArm.readDMPdataFromFIFO(&dmp);
        if (imuUpperArm.status == ICM_20948_Stat_Ok && (dmp.header & DMP_header_bitmap_Quat9)) { rawUpperBuf = dmpQuat9ToBodyQuat(dmp); tUpper = now; }
    }

    if (tHand > 0 && tForearm > 0 && tUpper > 0) {
        unsigned long maxT = max(tHand, max(tForearm, tUpper));
        unsigned long minT = min(tHand, min(tForearm, tUpper));
        if ((maxT - minT) <= 25) {
            static unsigned long lastValidSync = 0;
            if (maxT != lastValidSync) {
                lastValidSync = maxT;
                rawHand = rawHandBuf; rawForearm = rawForearmBuf; rawUpper = rawUpperBuf;
                if (firstFrameAfterCalib) {
                    filteredHand = rawHand; filteredForearm = rawForearm; filteredUpper = rawUpper;
                } else {
                    filteredHand    = Math3D::nlerp(filteredHand,    rawHand,    Q_FILTER_ALPHA);
                    filteredForearm = Math3D::nlerp(filteredForearm, rawForearm, Q_FILTER_ALPHA);
                    filteredUpper   = Math3D::nlerp(filteredUpper,   rawUpper,   Q_FILTER_ALPHA);
                }

                // Cập nhật EMA ZUPT Filter
                imuUpperArm.getAGMT(); imuForearm.getAGMT(); imuHand.getAGMT();
                float gyrU = sqrtf(pow(imuUpperArm.gyrX(),2) + pow(imuUpperArm.gyrY(),2) + pow(imuUpperArm.gyrZ(),2));
                float gyrF = sqrtf(pow(imuForearm.gyrX(),2)  + pow(imuForearm.gyrY(),2)  + pow(imuForearm.gyrZ(),2));
                float gyrH = sqrtf(pow(imuHand.gyrX(),2)     + pow(imuHand.gyrY(),2)     + pow(imuHand.gyrZ(),2));
                emaGyrMagU = 0.2f * gyrU + 0.8f * emaGyrMagU;
                emaGyrMagF = 0.2f * gyrF + 0.8f * emaGyrMagF;
                emaGyrMagH = 0.2f * gyrH + 0.8f * emaGyrMagH;

                float accU = sqrtf(pow(imuUpperArm.accX(),2) + pow(imuUpperArm.accY(),2) + pow(imuUpperArm.accZ(),2));
                float accF = sqrtf(pow(imuForearm.accX(),2)  + pow(imuForearm.accY(),2)  + pow(imuForearm.accZ(),2));
                emaAccMagU = 0.2f * accU + 0.8f * emaAccMagU;
                emaAccMagF = 0.2f * accF + 0.8f * emaAccMagF;

                return true; 
            }
        }
    }
    return false;
}

Vec3 getSensorGravityVector(ICM_20948_I2C &imu) {
    ICM_20948_AGMT_t agmt = imu.getAGMT();
    Vec3 g = { (float)agmt.acc.axes.x, (float)agmt.acc.axes.y, (float)agmt.acc.axes.z };
    Math3D::vecNormalize(g); return g;
}

bool checkZUPT_ZeroVelocityGate() {
    // Ngưỡng ZUPT tích hợp bộ lọc EMA chống gai nhiễu (Adaptive approach)
    if (emaGyrMagU > 12.0f || emaGyrMagF > 12.0f || emaGyrMagH > 15.0f) return false;
    if (abs(emaAccMagU - 1000.0f) > 180.0f || abs(emaAccMagF - 1000.0f) > 180.0f) return false;
    return true;
}

// ==============================================================================
// HIỆU CHUẨN ĐỘNG HỌC & PCA
// ==============================================================================
Vec3 executeRelativePCA_SensorFrame(int jointMode, const char* jointName) {
    Vec3 axis = {0.0f, 0.0f, 1.0f}; int samples = 0; bool isDataValid = false;
    do {
        Serial.printf("\n[PCA Debug] Đang học trục khớp [%s]...\n", jointName);
        digitalWrite(LED_PIN, HIGH);
        
        float xx=0, xy=0, xz=0, yy=0, yz=0, zz=0; 
        samples = 0; 
        unsigned long start = millis();

        while (millis() - start < 3000) {
            if (fetchSynchronizedIMUData()) {
                imuUpperArm.getAGMT(); imuForearm.getAGMT(); imuHand.getAGMT();
                Vec3 w;
                if (jointMode == 2) w = {imuUpperArm.gyrX(), imuUpperArm.gyrY(), imuUpperArm.gyrZ()};
                else if (jointMode == 1) w = {imuForearm.gyrX() - imuUpperArm.gyrX(), imuForearm.gyrY() - imuUpperArm.gyrY(), imuForearm.gyrZ() - imuUpperArm.gyrZ()};
                else w = {imuHand.gyrX() - imuForearm.gyrX(), imuHand.gyrY() - imuForearm.gyrY(), imuHand.gyrZ() - imuForearm.gyrZ()};

                if (Math3D::vecMagnitude(w) > 0.5f) {
                    Math3D::vecNormalize(w);
                    xx += w.x * w.x; xy += w.x * w.y; xz += w.x * w.z;
                    yy += w.y * w.y; yz += w.y * w.z; zz += w.z * w.z;
                    samples++;
                }
            } delay(1);
        }
        digitalWrite(LED_PIN, LOW);

        if (samples < 20) {
            Serial.printf("[PCA Debug] THẤT BẠI: Chỉ thu được %d mẫu (Cần >= 20).\n", samples);
            delay(1000); 
        } else {
            isDataValid = true; 
            Serial.printf("[PCA Debug] THÀNH CÔNG: %d mẫu. Ma trận hiệp phương sai:\n", samples);
            Serial.printf("  xx: %.4f | xy: %.4f | xz: %.4f\n", xx, xy, xz);
            Serial.printf("  yy: %.4f | yz: %.4f | zz: %.4f\n", yy, yz, zz);

            axis = {1.0f, 1.0f, 1.0f};
            for (int i = 0; i < 8; i++) {
                Vec3 nextA = { xx*axis.x + xy*axis.y + xz*axis.z, 
                               xy*axis.x + yy*axis.y + yz*axis.z, 
                               xz*axis.x + yz*axis.y + zz*axis.z };
                axis = nextA; Math3D::vecNormalize(axis);
            }
            if (axis.z < 0) axis = Math3D::vecScale(axis, -1.0f);
            
            Serial.printf("[PCA Debug] Trục chính thu được (Eigenvector): [%.4f, %.4f, %.4f]\n", axis.x, axis.y, axis.z);
        }
    } while (!isDataValid);
    return axis;
}

void waitUserTrigger() {
    while (digitalRead(BOOT_BUTTON_PIN) == LOW) { fetchSynchronizedIMUData(); delay(10); } delay(50); 
    unsigned long blinkTimer = millis(); bool ledState = false;
    while (digitalRead(BOOT_BUTTON_PIN) == HIGH) {
        fetchSynchronizedIMUData(); 
        if (millis() - blinkTimer > 500) { blinkTimer = millis(); ledState = !ledState; digitalWrite(LED_PIN, ledState); }
        if (Serial.available()) { char c = Serial.read(); if (c == '\n' || c == '\r' || c == ' ') return; } delay(5);
    }
    digitalWrite(LED_PIN, LOW); 
    while (digitalRead(BOOT_BUTTON_PIN) == LOW) { fetchSynchronizedIMUData(); delay(10); } delay(50); 
}

void calibrateFlexSensor() {
    Serial.println("\n--- [FLEX CALIB] ĐO NGƯỠNG KẸP TAY ---");
    Serial.println("-> HƯỚNG DẪN: Giữ ngón tay DUỖI THẲNG TỰ NHIÊN.");
    Serial.println("-> [NHẤN NÚT BOOT ĐỂ CHỐT MỐC DUỖI]..."); waitUserTrigger();
    float valRelaxed = (float)analogRead(FLEX_PIN);

    Serial.println("-> HƯỚNG DẪN: GẬP NGÓN TAY CHẶT HẾT CỠ.");
    Serial.println("-> [NHẤN NÚT BOOT ĐỂ CHỐT MỐC GẬP]..."); waitUserTrigger();
    float valGripped = (float)analogRead(FLEX_PIN);

    float range = abs(valGripped - valRelaxed);
    if (valGripped > valRelaxed) {
        flexGripThreshold = valRelaxed + (range * 0.7f); flexReleaseThreshold = valRelaxed + (range * 0.4f);
    } else {
        flexGripThreshold = valRelaxed - (range * 0.7f); flexReleaseThreshold = valRelaxed - (range * 0.4f);
    }
    Serial.printf("[OK] ThreshGrip: %.0f | ThreshRelease: %.0f\n", flexGripThreshold, flexReleaseThreshold);
}

void runFullAnatomical3DOFMountCalibration() {
    Serial.println("\n=======================================================");
    Serial.println("[FULL CALIB] HIỆU CHUẨN ĐỘNG HỌC 5 TRỤC TRỰC GIAO");
    Serial.println("=======================================================");

    calibrateFlexSensor();

    Serial.println("\n[BƯỚC 0] TƯ THẾ N-POSE (Lấy trọng lực)");
    Serial.println("-> [NHẤN NÚT BOOT]..."); waitUserTrigger(); fetchSynchronizedIMUData(); 
    Vec3 gUpper = getSensorGravityVector(imuUpperArm); Vec3 gForearm = getSensorGravityVector(imuForearm);

    Serial.println("\n[BƯỚC 1] VAI S2 (Nâng lên / Hạ xuống)");
    Serial.println("-> [NHẤN NÚT BOOT] RỒI VẬN ĐỘNG..."); waitUserTrigger();
    Vec3 pcaShoulderPitch = executeRelativePCA_SensorFrame(2, "VAI (Lên/Xuống)"); 

    Serial.println("\n[BƯỚC 2] VAI S1 (Đưa tay sang ngang / Khép lại)");
    Serial.println("-> [NHẤN NÚT BOOT] RỒI VẬN ĐỘNG..."); waitUserTrigger();
    Vec3 pcaShoulderYaw = executeRelativePCA_SensorFrame(2, "VAI (Sang ngang)"); 

    Serial.println("\n[BƯỚC 3] KHUỶU S3 (Gập / Duỗi cẳng tay)");
    Serial.println("-> [NHẤN NÚT BOOT] RỒI VẬN ĐỘNG..."); waitUserTrigger();
    Vec3 pcaElbow = executeRelativePCA_SensorFrame(1, "KHUỶU TAY"); 

    Serial.println("\n[BƯỚC 4] CỔ TAY S4 (Lật sấp / Lật ngửa)");
    Serial.println("-> [NHẤN NÚT BOOT] RỒI VẬN ĐỘNG..."); waitUserTrigger();
    Vec3 pcaWristTwist = executeRelativePCA_SensorFrame(0, "CỔ TAY (Sấp/Ngửa)");

    Serial.println("\n[BƯỚC 5] CỔ TAY S5 (Gập cổ tay / Ngóc cổ tay)");
    Serial.println("-> [NHẤN NÚT BOOT] RỒI VẬN ĐỘNG..."); waitUserTrigger();
    Vec3 pcaWristSwing = executeRelativePCA_SensorFrame(0, "CỔ TAY (Gập/Ngửa)");

    Serial.println("\n[ĐANG TÍNH TOÁN MA TRẬN TRỰC GIAO]...");
    
    Q_Mount_Upper   = Math3D::build3DOFMountQuat_GramSchmidt(pcaShoulderPitch, gUpper, true);
    Q_Mount_Forearm = Math3D::build3DOFMountQuat_GramSchmidt(pcaElbow, gForearm, true);
    Q_Mount_Hand    = Math3D::buildHandMount_GramSchmidt(pcaWristTwist, pcaWristSwing, "BÀN TAY");

    // Xoay trục PCA vào hệ quy chiếu động học
    dynAxis_S1_Yaw   = Math3D::rotateVector(pcaShoulderYaw,   Q_Mount_Upper);
    Vec3 rawPitch    = Math3D::rotateVector(pcaShoulderPitch, Q_Mount_Upper);
    
    // [CẬP NHẬT 10.4] Trực chuẩn hóa Vai (Gram-Schmidt) - S1 vuông góc hoàn toàn S2
    float projShoulder = Math3D::vecDot(rawPitch, dynAxis_S1_Yaw);
    dynAxis_S2_Pitch = { rawPitch.x - projShoulder*dynAxis_S1_Yaw.x, rawPitch.y - projShoulder*dynAxis_S1_Yaw.y, rawPitch.z - projShoulder*dynAxis_S1_Yaw.z };
    Math3D::vecNormalize(dynAxis_S2_Pitch);

    dynAxis_S3_Elbow = Math3D::rotateVector(pcaElbow,         Q_Mount_Forearm);
    dynAxis_S4_Twist = Math3D::rotateVector(pcaWristTwist,    Q_Mount_Hand);
    dynAxis_S5_Swing = Math3D::rotateVector(pcaWristSwing,    Q_Mount_Hand);

    Serial.println("\n=======================================================");
    Serial.println("[SUCCESS] HIỆU CHUẨN HOÀN TẤT!");
    Serial.println("=======================================================");
}

void runZeroCalibrationRoutine() {
    Serial.println("\n[ZERO CALIB] GIỮ YÊN TAY Ở TƯ THẾ CHUẨN N-POSE..."); digitalWrite(LED_PIN, HIGH);
    Quat sumU = {0,0,0,0}, sumF = {0,0,0,0}, sumH = {0,0,0,0}; int count = 0;
    while (count < 50) {
        if (fetchSynchronizedIMUData()) {
            Quat u = Math3D::quatMultiply(filteredUpper, Q_Mount_Upper); Quat f = Math3D::quatMultiply(filteredForearm, Q_Mount_Forearm); Quat h = Math3D::quatMultiply(filteredHand, Q_Mount_Hand);
            if (Math3D::quatDot(sumU, u) < 0) u = {-u.w,-u.x,-u.y,-u.z}; sumU.w+=u.w; sumU.x+=u.x; sumU.y+=u.y; sumU.z+=u.z;
            if (Math3D::quatDot(sumF, f) < 0) f = {-f.w,-f.x,-f.y,-f.z}; sumF.w+=f.w; sumF.x+=f.x; sumF.y+=f.y; sumF.z+=f.z;
            if (Math3D::quatDot(sumH, h) < 0) h = {-h.w,-h.x,-h.y,-h.z}; sumH.w+=h.w; sumH.x+=h.x; sumH.y+=h.y; sumH.z+=h.z;
            count++;
        } delay(1); 
    } digitalWrite(LED_PIN, LOW);
    Math3D::normalize(sumU); Math3D::normalize(sumF); Math3D::normalize(sumH);
    
    Home_Anat_Upper = sumU; Home_Anat_Forearm = sumF; Home_Anat_Hand = sumH;
    Home_Rel_Elbow  = Math3D::quatMultiply(Math3D::quatInverse(Home_Anat_Upper), Home_Anat_Forearm);
    Home_Rel_Wrist  = Math3D::quatMultiply(Math3D::quatInverse(Home_Anat_Forearm), Home_Anat_Hand);

    Q_DriftRel_Shoulder = {1,0,0,0}; Q_DriftRel_Elbow = {1,0,0,0}; Q_DriftRel_Wrist = {1,0,0,0}; 
    firstFrameAfterCalib = true; systemCalibrated = true; 
    Serial.println("[SUCCESS] Zero Calibration thành công!");
}

void handleRelativeHeadingDriftCompensation(Quat curU, Quat curF, Quat curH) {
    if (checkZUPT_ZeroVelocityGate()) {
        if (staticStartTime == 0) staticStartTime = millis();
        else if (millis() - staticStartTime > 500) { // Giảm thời gian chờ xuống 500ms
            // [CẬP NHẬT 10.4] Tăng hệ số bù trôi (0.005f) giúp hội tụ nhanh hơn gấp 8 lần khi đứng im
            Quat errU = Math3D::quatMultiply(curU, Math3D::quatInverse(Home_Anat_Upper));
            Quat swU, twU_Yaw; Math3D::decomposeSwingTwist(errU, dynAxis_S1_Yaw, swU, twU_Yaw);
            Q_DriftRel_Shoulder = Math3D::nlerp(Q_DriftRel_Shoulder, twU_Yaw, 0.005f);

            Quat rawRelElbow = Math3D::quatMultiply(Math3D::quatInverse(curU), curF);
            Quat errElbow = Math3D::quatMultiply(rawRelElbow, Math3D::quatInverse(Home_Rel_Elbow));
            Quat swE, twE_Yaw; Math3D::decomposeSwingTwist(errElbow, dynAxis_S3_Elbow, swE, twE_Yaw);
            Q_DriftRel_Elbow = Math3D::nlerp(Q_DriftRel_Elbow, twE_Yaw, 0.005f);

            Quat rawRelWrist = Math3D::quatMultiply(Math3D::quatInverse(curF), curH);
            Quat errWrist = Math3D::quatMultiply(rawRelWrist, Math3D::quatInverse(Home_Rel_Wrist));
            Quat swW, twW_Yaw; Math3D::decomposeSwingTwist(errWrist, dynAxis_S4_Twist, swW, twW_Yaw);
            Q_DriftRel_Wrist = Math3D::nlerp(Q_DriftRel_Wrist, twW_Yaw, 0.005f);
        }
    } else { staticStartTime = 0; }
}

void clampBiomechanicalLimits(float &s1, float &s2, float &s3, float &s4, float &s5) {
    s1 = constrain(s1, -90.0f,  90.0f); s2 = constrain(s2, -90.0f,  90.0f);
    s3 = constrain(s3,   0.0f, 140.0f); s4 = constrain(s4, -150.0f, 85.0f); s5 = constrain(s5, -95.0f, 35.0f);
}

float applyDeadband(float tgt, float cur, float thres) { return (abs(tgt - cur) > thres) ? tgt : cur; }
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
    Serial.begin(115200); delay(1000);
    pinMode(LED_PIN, OUTPUT); pinMode(FLEX_PIN, INPUT); pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP); 
    analogReadResolution(12); analogSetAttenuation(ADC_11db);

    I2CBus0.begin(I2C_BUS0_SDA, I2C_BUS0_SCL, 400000); I2CBus1.begin(I2C_BUS1_SDA, I2C_BUS1_SCL, 400000);
    initDMPDevice(imuHand, I2CBus1, AD0_VCC); initDMPDevice(imuForearm, I2CBus0, AD0_GND); initDMPDevice(imuUpperArm, I2CBus0, AD0_VCC);
    imuHand.resetDMP(); imuForearm.resetDMP(); imuUpperArm.resetDMP(); delay(300);

    Serial.println("Chờ bộ lọc DMP hội tụ (60s)...");
    unsigned long start = millis();
    while(millis() - start < 60000) { fetchSynchronizedIMUData(); delay(5); }
    
    Serial.println("Hệ thống Pipeline 10.4 Sẵn Sàng!");
    WiFi.mode(WIFI_STA); WiFi.channel(1);
    if (esp_now_init() == ESP_OK) {
        esp_now_peer_info_t peerInfo = {}; memcpy(peerInfo.peer_addr, rxMacAddress, 6);
        peerInfo.channel = 1; peerInfo.encrypt = false; esp_now_add_peer(&peerInfo);
    }
}

// ==============================================================================
// VÒNG LẶP CHÍNH
// ==============================================================================
void loop() {
    static unsigned long buttonPressStart = 0; static bool isButtonPressed = false;
    bool currentButtonState = (digitalRead(BOOT_BUTTON_PIN) == LOW);
    if (currentButtonState && !isButtonPressed) { isButtonPressed = true; buttonPressStart = millis(); } 
    else if (!currentButtonState && isButtonPressed) {
        isButtonPressed = false;
        if (millis() - buttonPressStart >= 2000) runFullAnatomical3DOFMountCalibration();
        else if (millis() - buttonPressStart >= 50) runZeroCalibrationRoutine();
    }

    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'f' || c == 'F') { runFullAnatomical3DOFMountCalibration(); while(Serial.available()) Serial.read(); }
        else if (c == 'c' || c == 'C') { runZeroCalibrationRoutine(); while(Serial.available()) Serial.read(); }
    }

    if (fetchSynchronizedIMUData() && systemCalibrated) {
        
        Quat Anat_U = Math3D::quatMultiply(filteredUpper,   Q_Mount_Upper);
        Quat Anat_F = Math3D::quatMultiply(filteredForearm, Q_Mount_Forearm);
        Quat Anat_H = Math3D::quatMultiply(filteredHand,    Q_Mount_Hand);

        handleRelativeHeadingDriftCompensation(Anat_U, Anat_F, Anat_H);

        Quat Corrected_U = Math3D::quatMultiply(Anat_U, Math3D::quatInverse(Q_DriftRel_Shoulder));
        Quat Q_Shoulder  = Math3D::quatMultiply(Math3D::quatInverse(Home_Anat_Upper), Corrected_U);

        Quat rawRelElbow = Math3D::quatMultiply(Math3D::quatInverse(Anat_U), Anat_F);
        Quat Q_Elbow     = Math3D::quatMultiply(Math3D::quatInverse(Q_DriftRel_Elbow), Math3D::quatMultiply(Math3D::quatInverse(Home_Rel_Elbow), rawRelElbow));

        Quat rawRelWrist = Math3D::quatMultiply(Math3D::quatInverse(Anat_F), Anat_H);
        Quat Q_Wrist     = Math3D::quatMultiply(Math3D::quatInverse(Q_DriftRel_Wrist), Math3D::quatMultiply(Math3D::quatInverse(Home_Rel_Wrist), rawRelWrist));

        // --- CASCADED KINEMATICS TRỰC GIAO ---
        Quat residual, twist;

        // Vai
        Math3D::decomposeSwingTwist(Q_Shoulder, dynAxis_S1_Yaw, residual, twist);
        float rawS1 = Math3D::getTwistAngle(twist, dynAxis_S1_Yaw);
        Math3D::decomposeSwingTwist(residual, dynAxis_S2_Pitch, residual, twist);
        float rawS2 = Math3D::getTwistAngle(twist, dynAxis_S2_Pitch);

        // Khuỷu
        Math3D::decomposeSwingTwist(Q_Elbow, dynAxis_S3_Elbow, residual, twist);
        float rawS3 = Math3D::getTwistAngle(twist, dynAxis_S3_Elbow);

                // Cổ tay
        Math3D::decomposeSwingTwist(Q_Wrist, dynAxis_S4_Twist, residual, twist);
        float tempS4 = Math3D::getTwistAngle(twist, dynAxis_S4_Twist);
        Math3D::decomposeSwingTwist(residual, dynAxis_S5_Swing, residual, twist);
        float tempS5 = Math3D::getTwistAngle(twist, dynAxis_S5_Swing);

                // --- ZUPT MOTION GATING & FLEX LOGIC ---
        bool isArmMoving = (emaGyrMagU > 15.0f || emaGyrMagF > 15.0f);
        static float frozen_S4 = 0.0f;
        static float frozen_S5 = 0.0f;
        static bool isGripping = false;

        int flexRaw = analogRead(FLEX_PIN);
        static float smoothedFlex = flexRaw;
        smoothedFlex = 0.15f * flexRaw + 0.85f * smoothedFlex;

        if (smoothedFlex > flexGripThreshold) isGripping = true;
        else if (smoothedFlex < flexReleaseThreshold) isGripping = false;

        float rawS4 = frozen_S4;
        float rawS5 = frozen_S5;

        if (!isArmMoving) {
            // Chỉ decomposition khi tay dừng
            Math3D::decomposeSwingTwist(Q_Wrist, dynAxis_S4_Twist, residual, twist);
            float tempS4 = Math3D::getTwistAngle(twist, dynAxis_S4_Twist);
            Math3D::decomposeSwingTwist(residual, dynAxis_S5_Swing, residual, twist);
            float tempS5 = Math3D::getTwistAngle(twist, dynAxis_S5_Swing);

            if (isGripping) {
                rawS4 = frozen_S4;
                rawS5 = frozen_S5;
            } else {
                float delta4 = fabs(tempS4 - frozen_S4);
                float delta5 = fabs(tempS5 - frozen_S5);
                if (delta4 > delta5) {
                    rawS4 = tempS4;
                    rawS5 = frozen_S5;
                } else {
                    rawS4 = frozen_S4;
                    rawS5 = tempS5;
                }
            }
            frozen_S4 = rawS4;
            frozen_S5 = rawS5;
        }

        float rawS6 = isGripping ? 10.0f : 50.0f;

        clampBiomechanicalLimits(rawS1, rawS2, rawS3, rawS4, rawS5);

        // --- GỬI ĐIỀU KHIỂN ---
        if (millis() - lastSendTime > 20) {
            lastSendTime = millis();

            float t1 = constrain(mapFloat(rawS1, mapS1.rawMin, mapS1.rawMax, mapS1.servoMin, mapS1.servoMax), mapS1.servoMin, mapS1.servoMax);
            float t2 = constrain(mapFloat(rawS2, mapS2.rawMin, mapS2.rawMax, mapS2.servoMin, mapS2.servoMax), mapS2.servoMin, mapS2.servoMax);
            float t3 = constrain(mapFloat(rawS3, mapS3.rawMin, mapS3.rawMax, mapS3.servoMin, mapS3.servoMax), mapS3.servoMin, mapS3.servoMax);
            float t4 = constrain(mapFloat(rawS4, mapS4.rawMin, mapS4.rawMax, mapS4.servoMin, mapS4.servoMax), mapS4.servoMin, mapS4.servoMax);
            float t5 = constrain(mapFloat(rawS5, mapS5.rawMin, mapS5.rawMax, mapS5.servoMin, mapS5.servoMax), mapS5.servoMin, mapS5.servoMax);

            if (firstFrameAfterCalib) {
                currentS1 = t1; currentS2 = t2; currentS3 = t3; currentS4 = t4; currentS5 = t5; currentS6 = rawS6;
                firstFrameAfterCalib = false;
            } else {
                currentS1 = applyDeadband(t1, currentS1, DEADBAND_DEG); currentS2 = applyDeadband(t2, currentS2, DEADBAND_DEG);
                currentS3 = applyDeadband(t3, currentS3, DEADBAND_DEG); currentS4 = applyDeadband(t4, currentS4, DEADBAND_DEG);
                currentS5 = applyDeadband(t5, currentS5, DEADBAND_DEG); currentS6 = applyDeadband(rawS6, currentS6, DEADBAND_DEG);
            }

            txData.S1 = currentS2; txData.S2 = currentS1; txData.S3 = currentS3;
            txData.S4 = currentS4; txData.S5 = currentS5; txData.S6 = currentS6;
            esp_now_send(rxMacAddress, (uint8_t*)&txData, sizeof(txData));

            if (millis() - lastDebugTime > 200) {
                lastDebugTime = millis();
                Serial.printf("[PIPELINE 10.4] S1:%.1f|S2:%.1f|S3:%.1f|S4:%.1f|S5:%.1f|KẸP:%s|ARM:%s\n", 
                               rawS2, rawS1, rawS3, rawS4, rawS5, isGripping?"ON":"OFF", isArmMoving?"MOVE":"STOP");
            }
        }
    }
}