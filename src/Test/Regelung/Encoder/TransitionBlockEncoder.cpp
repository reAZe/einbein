#include <einbein/Regelung/Encoder/TransitionBlockEncoder.hpp>

using namespace einbein; 
using namespace eeros;


TransitionBlockEncoder::TransitionBlockEncoder(){}


TransitionBlockEncoder::~TransitionBlockEncoder(){}



void TransitionBlockEncoder::runA(){//fast
  
  std::lock_guard<std::mutex> lock(mutex);
  enc_transition = in_fEnc.getSignal().getValue();

  //printf("runA --> fast\n");
}


void TransitionBlockEncoder::runB(){//slow
  std::lock_guard<std::mutex> lock(mutex);
  
  out_sEnc.getSignal().setValue(enc_transition);

  //printf("runB --> slow\n");
  
}
