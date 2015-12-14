#include <einbein/Regelung/ZustBest/CSZustBest.hpp>

using namespace einbein;
using namespace eeros::math;
using namespace eeros::control;

CSZustBest::CSZustBest(double ts) : 
    //init Block
    zustBest(ts),
    deMux_IMU_dd(), deMux_IMU_d(), deMux_IMU_Winkel(),
    imu(ts),
    encoderIMU(ts),
    
    timedomain("Zustandsbestimmung time domain",ts,true) {
    deMux_IMU_Winkel.getIn().connect(imu.getOut_angleIMUr());
    deMux_IMU_dd.getIn().connect(imu.getOut_ddxIMU());
    deMux_IMU_d.getIn().connect(imu.getOut_dxIMU());
    zustBest.getIn_ddzIMU().connect(deMux_IMU_dd.getOut(2));  
    zustBest.getIn_dzIMU().connect(deMux_IMU_d.getOut(2));      
    zustBest.getIn_enc().connect(encoderIMU.getOut_d_enc());
    
    
    //Add Block to run-Methode
    timedomain.addBlock(&imu);
    timedomain.addBlock(&deMux_IMU_Winkel);
    timedomain.addBlock(&deMux_IMU_dd);
    timedomain.addBlock(&deMux_IMU_d);
    timedomain.addBlock(&encoderIMU);
    timedomain.addBlock(&zustBest);
    
  }

  void CSZustBest::start(){
    timedomain.start();
  }
  
  void CSZustBest::stop(){
    timedomain.stop();
  }
  
  void CSZustBest::join(){
    timedomain.join();
  }