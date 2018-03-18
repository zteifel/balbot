#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

SoftwareSerial BT(3,2);
MPU6050 mpu;
#define OUTPUT_READABLE_EULER

int MOTOR_A1=5;
int MOTOR_A2=6;
int MOTOR_B1=9;
int MOTOR_B2=10;

float PID, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

double kp=5;//3.55
double ki=0.1;//0.003
double kd=1.5;//2.05

float elapsedTime, time, timePrev;
boolean reverse;
float angle; float desired_angle = 4;

uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    BT.begin(9600);
    Serial.begin(115200);

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-928);
    mpu.setYAccelOffset(-466);
    mpu.setZAccelOffset(974);

    mpu.setXGyroOffset(57);
    mpu.setYGyroOffset(-45);
    mpu.setZGyroOffset(49);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    // init motors
    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);

    time = millis();
}

void reset() {
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, LOW);
    pid_i = 0;
}

void setParams() {
    for(int i=1; i<=3; i++) {
        String pos = BT.readStringUntil('&');
        if ( i == 1) {
            kp = pos.toFloat();
        } else if ( i == 2 ) {
            kd = pos.toFloat();
        } else {
            ki = pos.toFloat();
        }
    }
}

void loop() {

    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000;

    if (BT.available()) {
        reset();
        setParams();
        BT.println("New parameters:");
        BT.print("kp="); BT.println(kp);
        BT.print("kd="); BT.println(kd);
        BT.print("ki="); BT.println(ki);
    }

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    angle = abs(euler[2] * 180/M_PI)-90 ;
    error = angle - desired_angle;
    delay(1000);
    BT.println(error);
    // BT.println();
    // BT.flush();
    if ( abs(error) > 75 or error == 0) {
        digitalWrite(MOTOR_A1, LOW);
        digitalWrite(MOTOR_A2, LOW);
        digitalWrite(MOTOR_B1, LOW);
        digitalWrite(MOTOR_B2, LOW);
    } else {
        pid_p = kp*error;

        if ( -5 < error < 5 ) {
            pid_i = pid_i+ error*ki;
        } else {
            pid_i = 0;
        }

        pid_d = kd*((error - previous_error)/elapsedTime);
        previous_error = error;

        PID = pid_p + pid_i + pid_d;

        reverse = false;
        if ( PID < 0) reverse = true;
        PID = abs(PID);
        if (PID > 200) PID = 200+55;

        // Actuators
        if (reverse ) {
            digitalWrite(MOTOR_A1,LOW);
            digitalWrite(MOTOR_B1,LOW);

            analogWrite(MOTOR_A2,PID);
            analogWrite(MOTOR_B2,PID);
        } else {
            digitalWrite(MOTOR_A2,LOW);
            digitalWrite(MOTOR_B2,LOW);

            analogWrite(MOTOR_A1,PID);
            analogWrite(MOTOR_B1,PID);
        }
    }
}
