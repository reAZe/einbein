#include <iostream>
#include <unistd.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>

#include <einbein/Regelung/IMU/IMU.hpp>




extern "C"{
//   #include "../mpu9250/mpu9250/mpu9250.h"
  #include "../src/mpu9250/mpu9250/MahonyAHRS.h"
  #include "../src/mpu9250/glue/linux_glue.h"
  
}


//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;



//Konstruktor
IMU::IMU(double ts){
  ddxIMU_1 = Matrix<3,1,double>({0,0,0});
  dxIMU_1 = Matrix<3,1,double>({0,0,0});
  xIMU_1 = Matrix<3,1,double>({0,0,0});
  angleIMU_1 = Matrix<3,1,double>({0,0,0});
  int i2c_bus = DEFAULT_I2C_BUS;
  int sample_rate = DEFAULT_SAMPLE_RATE_HZ;
  Ts = 0.01;//= ts; 
  
  int first = 1;
  
  if(init(i2c_bus, sample_rate)) 
    exit(1);
  
  out_angleIMU.getSignal().setValue(angleIMU);
  out_ddxIMU.getSignal().setValue(ddxIMU);
};

//Destruktor
IMU::~IMU(){};



void IMU::run(){
  //----------------------------- set Input -----------------------------
  
  
  //----------------------------- run -----------------------------
  getData(accel, gyro);
  
  
  
  MahonyAHRSupdate(gyro(VEC3_X),gyro(VEC3_Y),gyro(VEC3_Z),accel(VEC3_X),accel(VEC3_Y),accel(VEC3_Z),0,0,0);
  
  
  quaternion(QUAT_W) = q0;
  quaternion(QUAT_X) = q1;
  quaternion(QUAT_Y) = q2;
  quaternion(QUAT_Z) = q3;
  
  quaternionToEuler(quaternion,angleIMU);
  
  quaternionToRotMatr(quaternion,rotMatr);
  
  accelCompensation(rotMatr,accel,ddxIMU);
  
  positionVelocity(ddxIMU,dxIMU,xIMU);
  
  angularvelocity(angleIMU,dangleIMU);

  
  //Offset
  ddxIMU(0) = ddxIMU(0) + 0.008524181122449;
  ddxIMU(1) = ddxIMU(1) + 0.023237142857143; 
  ddxIMU(2) = ddxIMU(2) + 0.146394056122449  + 0.001027492857143; 
  
  
//       printf("%f;%f;%f; %f;%f;%f\n", 
//        ddxIMU(0),  ddxIMU(1),  ddxIMU(2), angleIMU(0), angleIMU(1), angleIMU(2));
     
  
  
  //----------------------------- set Output -----------------------------
  out_angleIMU.getSignal().setValue(angleIMU);
  out_dangleIMU.getSignal().setValue(dangleIMU);
  out_ddxIMU.getSignal().setValue(ddxIMU);
  out_dxIMU.getSignal().setValue(dxIMU);
  out_xIMU.getSignal().setValue(xIMU);
  
  //set Name
  out_angleIMU.getSignal().setName("alpha: beta: gamma [rad]");
  out_dangleIMU.getSignal().setName("alpha: beta: gamma [rad/s]");
  out_ddxIMU.getSignal().setName("ddx: ddy: ddz [m/s²]");
  out_dxIMU.getSignal().setName("dx: dy: dz [m/s]");
  out_xIMU.getSignal().setName("x: y: z [m]");

}// end run


int IMU::init(int i2c_bus, int sample_rate){
//   if (i2c_bus < MIN_I2C_BUS || i2c_bus > MAX_I2C_BUS) {
//     printf("Invalid I2C bus %d\n", i2c_bus);
//     return -1;
//   }
  
//   if (sample_rate < MIN_SAMPLE_RATE || sample_rate > MAX_SAMPLE_RATE) {
//     printf("Invalid sample rate %d\n", sample_rate);
//     return -1;
//   }

  linux_set_i2c_bus(i2c_bus);
  
  if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
    printf("\nmpu_set_sensors() failed\n");
    return -1;
  }
  
  if (mpu_init(NULL)) {
    printf("\nmpu_init() failed\n");
    return -1;
  }
  
  if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
    printf("\nmpu_set_sensors() failed\n");
    return -1;
  }
  
  if (mpu_set_sample_rate(sample_rate)) {
    printf("\nmpu_set_sample_rate() failed\n");
    return -1;
  }
  
  return 0;
}// end init


void IMU::getData(Matrix<3,1,double> &accel, Matrix<3,1,double> &gyro){
  unsigned char data_read[6];
  //read accel
  i2c_read(IMUADDR, RAWACCEL, 6, data_read); 
  rawAccel(VEC3_X) = ((short)data_read[0] << 8) | data_read[1];
  rawAccel(VEC3_Y) = ((short)data_read[2] << 8) | data_read[3];
  rawAccel(VEC3_Z) = ((short)data_read[4] << 8) | data_read[5];
  
  //read gyro
  i2c_read(IMUADDR, RAWGYRO, 6, data_read);
  rawGyro(VEC3_X) = ((short)data_read[0] << 8) | data_read[1];
  rawGyro(VEC3_Y) = ((short)data_read[2] << 8) | data_read[3];
  rawGyro(VEC3_Z) = ((short)data_read[4] << 8) | data_read[5];
  
  int i;
  for(i=VEC3_X;i<(VEC3_Z+1);i++){
    accel(i) = (float)rawAccel(i) / (ACCSENS); //OFFSET einfügen
    gyro(i) = (float)rawGyro(i) * DEGREE_TO_RAD / (GYROSENS);  //OFFSET einfügen
  } 
}// end getData


void IMU::quaternionToEuler(Matrix<4,1,double> quaternion, Matrix<3,1,double> &angleIMU){
  double q0 = quaternion[QUAT_W];
  double q1 = quaternion[QUAT_X];
  double q2 = quaternion[QUAT_Y];
  double q3 = quaternion[QUAT_Z];
  
  angleIMU(VEC3_X) = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
  angleIMU(VEC3_Y) = asin(2*(q0*q2-q3*q1));
  angleIMU(VEC3_Z) = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
}// end quaternionToEuler


void IMU::quaternionToRotMatr(Matrix<4,1,double> quaternion, Matrix<3,3,double> &rotMatr){
  /*
   * Rotation Matrix from the inertial frame to the body frame
   */
  double q0 = quaternion[QUAT_W];
  double q1 = quaternion[QUAT_X];
  double q2 = quaternion[QUAT_Y];
  double q3 = quaternion[QUAT_Z];
  
  rotMatr = Matrix<3,3,double>({q0*q0+q1*q1-q2*q2-q3*q3, 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3),
			       2*(q1*q2+q0*q3), q0*q0-q1*q1+q2*q2-q3*q3, 2*(q2*q3-q0*q1),
			       2*(q1*q3-q0*q2), 2*(q0*q1+q2*q3), q0*q0-q1*q1-q2*q2+q3*q3});//.transpose();  
}// end quaternionToRotMatr


void IMU::accelCompensation(Matrix<3,3,double> rotMatr, Matrix<3,1,double> accel, Matrix<3,1,double> &ddxIMU){
  ddxIMU = (accel + rotMatr * Matrix<3,1,double>({0.0, 0.0, -1.0}))*9.81;
}// end accelCompensation


void IMU::positionVelocity(Matrix<3,1,double> ddxIMU, Matrix<3,1,double> &dxIMU, Matrix<3,1,double> &xIMU){
  dxIMU = dxIMU_1 + (ddxIMU+ddxIMU_1)*Ts/2; 
  xIMU = xIMU_1 + (dxIMU+dxIMU_1)*Ts/2; 
  
  dxIMU_1 = dxIMU;
  xIMU_1 = xIMU;
}// end positionVelocity


void IMU::angularvelocity(Matrix<3,1,double> angleIMU, Matrix<3,1,double> &dangleIMU){
  dangleIMU = (angleIMU - angleIMU_1)/Ts;
  
  angleIMU_1 = angleIMU;
}// end angularvelocity