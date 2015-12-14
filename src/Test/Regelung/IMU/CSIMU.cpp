#include "einbein/Regelung/IMU/CSIMU.hpp"

using namespace einbein;


CSIMU::CSIMU(double ts) :
  //init Block
  imu(ts),
  timedomain("IMU time domain",ts,true){
    
    //Add Block to run-Methode
    timedomain.addBlock(&imu);
  }
  
  
void CSIMU::start(){
  timedomain.start();
}  

void CSIMU::stop(){
  timedomain.stop();
}  

void CSIMU::join(){
  timedomain.join();
}  