#ifndef EINBEIN_IMU_HPP
#define EINBEIN_IMU_HPP


#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/math/Matrix.hpp>


#define DEFAULT_I2C_BUS 	2
#define DEFAULT_SAMPLE_RATE_HZ	100
#define	DEGREE_TO_RAD		((float)M_PI / 180.0f)
#define	RAD_TO_DEGREE		(180.0f / (float)M_PI)
#define VEC3_X			0
#define VEC3_Y			1
#define VEC3_Z			2
#define QUAT_W			0
#define QUAT_X			1
#define QUAT_Y			2
#define QUAT_Z			3
#define RAWACCEL		0x3B
#define RAWGYRO			0x43
#define IMUADDR			0x69
#define ACCSENS  		(32768/16)
#define GYROSENS 		(32768/2000)
#define OFFSETGYRO_X		(37)
#define OFFSETGYRO_Y		(-13)
#define OFFSETGYRO_Z		(19)
#define OFFSETACCEL_X		(0.0011)
#define OFFSETACCEL_Y		(0.0032)
#define OFFSETACCEL_Z		(0.0036)


using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::logger;
using namespace eeros::math;

namespace einbein{
  class IMU : public eeros::control::Block{
  
  public:
    IMU(double ts);
    virtual ~IMU();
    
    //define outputs
    virtual eeros::control::Output<Matrix<3,1,double>>& getOut_angleIMUr(){return out_angleIMU;}
    virtual eeros::control::Output<Matrix<3,1,double>>& getOut_dangleIMUr(){return out_dangleIMU;}
    virtual eeros::control::Output<Matrix<3,1,double>>& getOut_ddxIMU(){return out_ddxIMU;}
    virtual eeros::control::Output<Matrix<3,1,double>>& getOut_dxIMU(){return out_dxIMU;}
    virtual eeros::control::Output<Matrix<3,1,double>>& getOut_xIMU(){return out_xIMU;}
    
    
  protected:
    //define outputs
    eeros::control::Output<Matrix<3,1,double>> out_angleIMU;
    eeros::control::Output<Matrix<3,1,double>> out_dangleIMU;
    eeros::control::Output<Matrix<3,1,double>> out_ddxIMU;
    eeros::control::Output<Matrix<3,1,double>> out_dxIMU;
    eeros::control::Output<Matrix<3,1,double>> out_xIMU;
    
    
  private:
    virtual void run();
    virtual int init(int i2c_bus, int sample_rate);
    virtual void getData(Matrix<3,1,double> &accel, Matrix<3,1,double> &gyro);
    virtual void quaternionToEuler(Matrix<4,1,double> quaternion, Matrix<3,1,double> &angle);
    virtual void quaternionToRotMatr(Matrix<4,1,double> quaternion, Matrix<3,3,double> &rotMatr);
    virtual void accelCompensation(Matrix<3,3,double> rotMatr, Matrix<3,1,double> accel, Matrix<3,1,double> &ddxIMU);
    virtual void positionVelocity(Matrix<3,1,double> ddxIMU, Matrix<3,1,double> &dxIMU, Matrix<3,1,double> &xIMU);
    virtual void angularvelocity(Matrix<3,1,double> angleIMU, Matrix<3,1,double> &dangleIMU);
    
    int first;
    Matrix<4,1,double> quaternion;
    Matrix<3,3,double> rotMatr;
    Matrix<3,1,short> rawGyro;
    Matrix<3,1,short> rawAccel;
    Matrix<3,1,double> accel;
    Matrix<3,1,double> gyro;
    Matrix<3,1,double> ddxIMU;
    Matrix<3,1,double> dxIMU;
    Matrix<3,1,double> xIMU;
    Matrix<3,1,double> ddxIMU_1;
    Matrix<3,1,double> dxIMU_1;
    Matrix<3,1,double> xIMU_1;
    Matrix<3,1,double> angleIMU;
    Matrix<3,1,double> angleIMU_1;
    Matrix<3,1,double> dangleIMU;
    double Ts; 
    
  };//end class IMU
}//end namspace einbein

#endif // EINBEIN_IMU_HPP