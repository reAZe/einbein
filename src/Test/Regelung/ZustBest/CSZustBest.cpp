#include <einbein/Regelung/ZustBest/CSZustBest.hpp>

using namespace einbein;
using namespace eeros::math;
using namespace eeros::control;

CSZustBest::CSZustBest(double ts) : 
    //init Block
    zustBest(ts),
    deMux_IMU_Winkel(), 
    deMux_Encoder(),
    imu(ts),
    encoderIMU(ts),
    
    timedomain("Zustandsbestimmung time domain",ts,true) {
    deMux_IMU_Winkel.getIn().connect(imu.getOut_angleIMUr());
    deMux_Encoder.getIn().connect(encoderIMU.getOut_enc());
    zustBest.getIn_enc().connect(encoderIMU.getOut_d_enc());
    
    
    //Add Block to run-Methode
    timedomain.addBlock(&imu);
    timedomain.addBlock(&deMux_IMU_Winkel);
    timedomain.addBlock(&deMux_Encoder);
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