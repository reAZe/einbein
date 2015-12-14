
#include "einbein/Regelung/Encoder/CSEncoder.hpp"



using namespace einbein;
using namespace eeros::math;
using namespace eeros::control;


CSEncoder::CSEncoder(double ts) :
    //init Block
    encoder(ts),
    timedomain("Main time domain", ts, true) {

    
    //Add Block to run-Methode
    timedomain.addBlock(&encoder);


    printf("\nAdd Block to run-Methode\n");
   }
  
void CSEncoder::start(){
    timedomain.start();
    printf("start CSEncoder\n");
}  


void CSEncoder::stop(){
    timedomain.stop();
}
 

void CSEncoder::join(){
    timedomain.join();
} 
 