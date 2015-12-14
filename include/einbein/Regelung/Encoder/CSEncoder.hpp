#ifndef EINBEIN_CSENCODER_HPP
#define EINBEIN_CSENCODER_HPP


#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>
#include <einbein/Regelung/Encoder/Encoder.hpp>

namespace einbein {

class CSEncoder{
  public:
  Encoder encoder;
  CSEncoder(double ts);
  void start();
  void stop();
  void join();
  
  

    
  private:
  eeros::control::TimeDomain timedomain;
};

}

#endif // EINBEIN_CSENCODER_HPP