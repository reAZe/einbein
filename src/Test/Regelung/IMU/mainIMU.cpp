#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include "einbein/Regelung/IMU//CSIMU.hpp"
#include <eeros/logger/SysLogWriter.hpp>


using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace einbein;

// enable to stop the program with ctrl c
volatile bool running = true;

// handler on press on ctrl c that will set running to false
void signalHandler(int signum) {
	running = false;
}



int main() {
  //set the interrupt handler
  signal(SIGINT, signalHandler);
  
  
  //start the loggers
  StreamLogWriter w(std::cout);
  SysLogWriter s("delta");
  w.show(0);
  s.show();
  
  Logger<LogWriter>::setDefaultWriter(&w);	
  Logger<LogWriter> log('M');
  HAL& hal = HAL::instance();
  
  
  log.trace() << "Application sismplesystem started...";
  
  // create control system
  CSIMU csIMU(1);
  
  // initialize hardware
  csIMU.start();
  
  log.trace() << "Init done!";
  
  
  while (running) {
    std::cout << "\n------------------------------------------------------------------------------ "  << std::endl; 
    
    std::cout << "ang		: " << csIMU.imu.getOut_angleIMUr().getSignal().getValue() << "	[rad]" << std::endl;
    std::cout << "angVel		: " << csIMU.imu.getOut_angleIMUr().getSignal().getValue() << "	[rad]" << std::endl;
    std::cout << "acc		: " << csIMU.imu.getOut_ddxIMU().getSignal().getValue() << "	[m/s²]" << std::endl;
    std::cout << "vel		: " << csIMU.imu.getOut_dxIMU().getSignal().getValue() << "	[m/s]" << std::endl;
    std::cout << "pos		: " << csIMU.imu.getOut_xIMU().getSignal().getValue() << "	[m]" << std::endl;
    
    sleep(1);
  }
  
  csIMU.stop();
  csIMU.join();

  return 0;
}