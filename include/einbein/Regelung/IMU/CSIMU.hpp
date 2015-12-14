#ifndef EINBEIN_CSIMU_HPP
#define EINBEIN_CSIMU_HPP

#include <einbein/Regelung/IMU/IMU.hpp>
#include <eeros/control/TimeDomain.hpp>


namespace einbein{
  class CSIMU{
  public:
    IMU imu;
    CSIMU(double ts);
    void start();
    void stop();
    void join();
    
  private:
    eeros::control::TimeDomain timedomain;
  };
}

#endif // EINBEIN_CSIMU_HPP