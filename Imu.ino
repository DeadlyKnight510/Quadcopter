// =============================Setup MPU6050=============================
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

  MPU6050 mpu;
  #define OUTPUT_READABLE_YAWPITCHROLL

  //MPU control/status vars
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  //Orientation/Motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
  void dmpDataReady() {
      mpuInterrupt = true;
  }

// =============================Setup MPU6050 Done=============================

void imu_initialize()
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    mpu.setXGyroOffset(57);
    mpu.setYGyroOffset(37);
    mpu.setZGyroOffset(48);
    mpu.setZAccelOffset(1532); // 1688 factory default for my test chip

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);   
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
    }

}

void imu_update() 
{

  if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {

    }
      
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
    }
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angleX = ypr[2] * 180/M_PI; //roll
    angleY = ypr[1] * 180/M_PI; //pitch
    angleZ = ypr[0] * 180/M_PI; //yaw
}

