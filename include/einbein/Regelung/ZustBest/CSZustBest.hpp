#ifndef EINBEIN_CSZUSTBEST_HPP
#define EINBEIN_CSZUSTBEST_HPP

#include <einbein/Regelung/ZustBest/ZustBest.hpp>
#include <einbein/Regelung/Encoder/Encoder.hpp>
#include <einbein/Regelung/IMU/IMU.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/DeMux.hpp>

namespace einbein {

  class CSZustBest{
  public:
    ZustBest zustBest;
    IMU imu;
    Encoder encoderIMU;
    CSZustBest(double ts);
    
    
    eeros::control::DeMux<3, double> deMux_IMU_dd;
    eeros::control::DeMux<3, double> deMux_IMU_d;  
    eeros::control::DeMux<3, double> deMux_IMU_Winkel;  
    eeros::control::DeMux<3, double> deMux_Encoder;
    
    
    void start();
    void stop();
    void join();
    
  private:
    eeros::control::TimeDomain timedomain;
  };

}

#endif // EINBEIN_CSZUSTBEST_HPP