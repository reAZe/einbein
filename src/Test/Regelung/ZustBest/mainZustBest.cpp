#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <einbein/Regelung/ZustBest/CSZustBest.hpp>
#include <eeros/logger/SysLogWriter.hpp>


#include <eeros/hal/FlinkDevice.hpp>
#include <eeros/hal/FlinkDigIn.hpp>
#include <eeros/hal/FlinkDigOut.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/hal/FlinkAnalogOut.hpp>
#include <eeros/hal/FlinkPwm.hpp>
#include "../include/einbein/constants.hpp"


#define FPGA_DEVICE "/dev/flink0"

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
  
  FlinkDevice onBoard (FPGA_DEVICE);
  //Encoder
	FlinkDigOut Mot1EncSupret("Mot1EncSupret", &onBoard, GPIO_MOT_ENC, 1);
	FlinkDigOut Mot2EncSupret("Mot2EncSupret", &onBoard, GPIO_MOT_ENC, 3);
	FlinkDigOut Mot3EncSupret("Mot3EncSupret", &onBoard, GPIO_MOT_ENC, 5);
	
	FlinkFqd mot1Enc("mot1Enc", &onBoard, FQD_ID, 0, fqd0scale, fqd0offset, fqd0delta);	//Motor 1 = Channel 0
	FlinkFqd mot2Enc("mot2Enc", &onBoard, FQD_ID, 1, fqd1scale, fqd1offset, fqd1delta);	//Motor 2 = Channel 1
	FlinkFqd mot3Enc("mot3Enc", &onBoard, FQD_ID, 2, fqd2scale, fqd2offset, fqd2delta); 	//Motor 3 = Channel 2	
  
	
	//Taster
	FlinkDigIn TasterRes1("TasterRes1", &onBoard, GPIO_SWITCH_LED, 11);
	FlinkDigIn TasterRes2("TasterRes2", &onBoard, GPIO_SWITCH_LED, 12);
 	FlinkDigIn TasterRes3("TasterRes3", &onBoard, GPIO_SWITCH_LED, 13);
	
  	
	hal.addPeripheralInput(&mot1Enc);
	hal.addPeripheralOutput(&Mot1EncSupret);
	hal.addPeripheralInput(&mot2Enc);
	hal.addPeripheralOutput(&Mot2EncSupret);	
	hal.addPeripheralInput(&mot3Enc);
	hal.addPeripheralOutput(&Mot3EncSupret);
	hal.addPeripheralInput(&TasterRes3);
	hal.addPeripheralInput(&TasterRes2);
	hal.addPeripheralInput(&TasterRes1);
  
  
  // create control system
  CSZustBest csZustBest(0.001);
  
  // initialize hardware
  csZustBest.start();
  
  log.trace() << "Init done!";
  
  
  unsigned int zust;
  unsigned int zust_1 = 0;
  
  while(running){
   // std::cout << "\n------------------------------------------------------------------------------ "  << std::endl; 
    
    		
    if (!TasterRes2.get()){
      std::cout << "\nReset Encoder \n" << std::endl;

      mot1Enc.reset();
      mot2Enc.reset();
      mot3Enc.reset();
       
    }//end if Taster 2
    std::cout << "Encoder: " << csZustBest.encoderIMU.getOut_enc().getSignal().getValue() << std::endl;
    std::cout << "Beschleunigung: " << csZustBest.deMux_IMU_dd.getOut(2).getSignal().getValue() << std::endl;
    std::cout << "Geschwindigkeit: " << csZustBest.deMux_IMU_d.getOut(2).getSignal().getValue() << std::endl;
//     printf("%f;%f;%f; %f;%f;%f\n", 
//       csZustBest.deMux_IMU_dd.getOut(0).getSignal().getValue(), csZustBest.deMux_IMU_dd.getOut(1).getSignal().getValue(), csZustBest.deMux_IMU_dd.getOut(2).getSignal().getValue(),
//       csZustBest.deMux_IMU_Winkel.getOut(0).getSignal().getValue(), csZustBest.deMux_IMU_Winkel.getOut(1).getSignal().getValue(), csZustBest.deMux_IMU_Winkel.getOut(2).getSignal().getValue()
//     );
      

    
    
//     zust = csZustBest.zustBest.getOut_Zustand().getSignal().getValue();  
//     if(zust != zust_1){
//       std::cout << "Zustand: " << csZustBest.zustBest.getOut_Zustand().getSignal().getValue() << std::endl;
//       zust_1 = zust;
//     }
    sleep(1);
    //usleep(100000);
  }
  
  csZustBest.stop();
  csZustBest.join();
  
  return 0;
}