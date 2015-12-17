#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include "einbein/Regelung/CSController.hpp"
#include <eeros/logger/SysLogWriter.hpp>


#include <eeros/hal/FlinkDevice.hpp>
#include <eeros/hal/FlinkDigIn.hpp>
#include <eeros/hal/FlinkDigOut.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/hal/FlinkAnalogOut.hpp>
#include <eeros/hal/FlinkPwm.hpp>
#include "../include/einbein/constants.hpp"


#define FPGA_DEVICE "/dev/flink0"

// unique-id of flink subdevices



using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace einbein;
using namespace std;

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
	
		
	log.trace() << "Application simplesystem started...";
	
	//flink Initialisieren
	FlinkDevice onBoard (FPGA_DEVICE);
	
	//Encoder
	FlinkDigOut Mot1EncSupret("Mot1EncSupret", &onBoard, GPIO_MOT_ENC, 1);
	FlinkDigOut Mot2EncSupret("Mot2EncSupret", &onBoard, GPIO_MOT_ENC, 3);
	FlinkDigOut Mot3EncSupret("Mot3EncSupret", &onBoard, GPIO_MOT_ENC, 5);
	
	FlinkFqd mot1Enc("mot1Enc", &onBoard, FQD_ID, 0, fqd0scale, fqd0offset, fqd0delta);	//Motor 1 = Channel 0
	FlinkFqd mot2Enc("mot2Enc", &onBoard, FQD_ID, 1, fqd1scale, fqd1offset, fqd1delta);	//Motor 2 = Channel 1
	FlinkFqd mot3Enc("mot3Enc", &onBoard, FQD_ID, 2, fqd2scale, fqd2offset, fqd2delta); 	//Motor 3 = Channel 2	
	
	
	//Motor 
	//Inhibit
	FlinkDigOut Mot1IO1("Mot1Inhibit", &onBoard, GPIO_MOT, GPIO_M1_INHIB);
	FlinkDigOut Mot2IO1("Mot2Inhibit", &onBoard, GPIO_MOT, GPIO_M2_INHIB);
	FlinkDigOut Mot3IO1("Mot3Inhibit", &onBoard, GPIO_MOT, GPIO_M3_INHIB);
	//Inret
	FlinkDigOut Mot1Inret("Mot1Inret", &onBoard, GPIO_MOT, GPIO_M1_INRET);
	FlinkDigOut Mot2Inret("Mot2Inret", &onBoard, GPIO_MOT, GPIO_M2_INRET);
	FlinkDigOut Mot3Inret("Mot3Inret", &onBoard, GPIO_MOT, GPIO_M3_INRET);
	//DAC
 	FlinkAnalogOut DACMot1("DACMot1", &onBoard, DAC_MOT, DAC_MOT_1, dac1Scale, dac1Offset);
 	FlinkAnalogOut DACMot2("DACMot2", &onBoard, DAC_MOT, DAC_MOT_2, dac2Scale, dac2Offset);
 	FlinkAnalogOut DACMot3("DACMot3", &onBoard, DAC_MOT, DAC_MOT_3, dac3Scale, dac3Offset);

	
	
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
	
	hal.addPeripheralOutput(&Mot1IO1);
	hal.addPeripheralOutput(&Mot2IO1);
	hal.addPeripheralOutput(&Mot3IO1);
	hal.addPeripheralOutput(&Mot1Inret);
	hal.addPeripheralOutput(&Mot2Inret);
	hal.addPeripheralOutput(&Mot3Inret);
	hal.addPeripheralOutput(&DACMot1);
	hal.addPeripheralOutput(&DACMot2);
	hal.addPeripheralOutput(&DACMot3);
	
	
	
	Mot1EncSupret.set(false);	//set output to GND
	Mot2EncSupret.set(false);	//set output to GND
	Mot3EncSupret.set(false);	//set output to GND
	
	Mot1Inret.set(false);		//set output to GND
	Mot2Inret.set(false);		//set output to GND
	Mot3Inret.set(false);		//set output to GND
	
	Mot1IO1.set(false);		//set output to GND
	Mot2IO1.set(false);		//set output to GND
	Mot3IO1.set(false);		//set output to GND
	

	
	
	// create control system
	CSControll cSControll(0.001); //schneller als Nachstellzeit PD-Regler
	
	// initialize hardware
	cSControll.start();
	
	printf("Init done\n");

	bool toggle = false;
	double x_Richtung, y_Richtung, z_Richtung;

	
	while (running) {

// 		std::cout << "\n------------------------------------------------------------------------------ "  << std::endl; 
// 		
// 		
//  		//enc
//  		std::cout << "enc		: " << cSControll.encoder.getOut_enc().getSignal().getValue() << "  [m]" << std::endl;
//  		//d_enc
//   		std::cout << "d_enc		: " << cSControll.encoder.getOut_d_enc().getSignal().getValue() << "  [m/s]" << std::endl;
// // 		//Pf_IMU
// // 		std::cout << "Pf_IMU		: " << cSControll.vorKinController.getOut_Pf_IMU().getSignal().getValue() << "  [m]" << std::endl;
// 		//Pf_0
// 		std::cout << "Pf_0		: " << cSControll.vorKinController.getOut_Pf_0().getSignal().getValue() << "  [m]" << std::endl;
// 		//FMot
// 		std::cout << "F_Mot 		: " << cSControll.vorKinController.getOut_FMsoll().getSignal().getValue() <<" [N]"  << std::endl;   
// 		
// 		
// 		
// 		
// 		// 		//IM
// 		//std::cout << "Saturation I	: " << cSControll.saturation.getOut().getSignal().getValue() << "  [m]" << std::endl;
// 		//Reset Encoder
// 		
//  		std::cout << "FSollMot_p_Out	: " << cSControll.pMotor.getOut_FSollMot().getSignal().getValue() << "  [V]" << std::endl;
//  		std::cout << "Saturation In	: " << cSControll.saturation.getIn().getSignal().getValue() << "  [V]" << std::endl;
//  		std::cout << "Saturation Out	: " << cSControll.saturation.getOut().getSignal().getValue() << "  [V]" << std::endl; 		
//		std::cout << "dac		: " << cSControll.i2DAC.getIn_Voltage().getSignal().getValue() << "  [V]" << std::endl;
		
		//Ausgabe Trajektorie [xf_Trajekt, yf_Trajekt, zf_Trajekt, xf_ist, yf_ist, zf_ist, , xf_soll, yf_soll, zf_soll]
		/*printf("%f;  %f; %f;  %f;  %f;  %f;  %f;  %f;  %f\n", 	
			cSControll.trajektorie_xf.getOut_x_d().getSignal().getValue(), cSControll.trajektorie_yf.getOut_x_d().getSignal().getValue(), cSControll.trajektorie_zf.getOut_x_d().getSignal().getValue(),
			cSControll.deMux_Pf0.getOut(0).getSignal().getValue(), cSControll.deMux_Pf0.getOut(1).getSignal().getValue(), cSControll.deMux_Pf0.getOut(2).getSignal().getValue(),
			cSControll.deMux_xSoll.getOut(0).getSignal().getValue(), cSControll.deMux_xSoll.getOut(1).getSignal().getValue(), cSControll.deMux_xSoll.getOut(2).getSignal().getValue() 
		);*/
		
		
		//Ausgabe Regelgrösse [xf_ist, yf_ist, zf_ist, , xf_soll, yf_soll, zf_soll, FxSoll, FySoll, FzSoll,
		//		      M1_Saturation, M2_Saturation, M3_Saturation, FM1, FM2, FM3, d_istX, d_distY, d_distZ
		//			xSollTraje, ySollTraje, zSollTraje]
// 		printf("%f;  %f;  %f;  %f;  %f;  %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f\n", 
// 			cSControll.deMux_Pf0.getOut(0).getSignal().getValue(), cSControll.deMux_Pf0.getOut(1).getSignal().getValue(), cSControll.deMux_Pf0.getOut(2).getSignal().getValue(),
// 			cSControll.deMux_xSoll.getOut(0).getSignal().getValue(), cSControll.deMux_xSoll.getOut(1).getSignal().getValue(), cSControll.deMux_xSoll.getOut(2).getSignal().getValue(), 
// 			cSControll.pDV_xf.getOut_F_0().getSignal().getValue(), cSControll.pDV_yf.getOut_F_0().getSignal().getValue(), cSControll.pDV_zf.getOut_F_0().getSignal().getValue(),
// 			cSControll.deMux_Saturation.getOut(0).getSignal().getValue(), cSControll.deMux_Saturation.getOut(1).getSignal().getValue(), cSControll.deMux_Saturation.getOut(2).getSignal().getValue(),
// 			cSControll.deMux_vorKin.getOut(0).getSignal().getValue(), cSControll.deMux_vorKin.getOut(1).getSignal().getValue(), cSControll.deMux_vorKin.getOut(2).getSignal().getValue(),
// 			cSControll.trajektorie_xf.getOut_x_d().getSignal().getValue(), cSControll.trajektorie_yf.getOut_x_d().getSignal().getValue(), cSControll.trajektorie_zf.getOut_x_d().getSignal().getValue()
// 		  	);

	    
// 		//Ausgabe Fussposition [xf_ist, yf_ist, zf_ist]
// 		printf("%f; %f; %f; \n", 
// 		  cSControll.deMux_Pf0.getOut(0).getSignal().getValue(), cSControll.deMux_Pf0.getOut(1).getSignal().getValue(), cSControll.deMux_Pf0.getOut(2).getSignal().getValue()
// 		);
// 		

	      //Ausgabe Encoderwerte
	      printf("%f; %f; \n",
		cSControll.deMuxEncoder.getOut(0).getSignal().getValue(), cSControll.deMuxdEncoder.getOut(0).getSignal().getValue()
	      );


		
	      if (!TasterRes1.get()){
		
		  //Vorwärtkinematik mit Kraft 
		  /*std::cout << "\nKraft in x_Richtung eingeben \n" << std::endl;
		  cin >> x_Richtung;
		  std::cout << "\nKraft in y_Richtung eingeben \n" << std::endl;
		  cin >> y_Richtung;
		  std::cout << "\nKraft in z_Richtung eingeben \n" << std::endl;
		  cin >> z_Richtung;
		  
		  cSControll.F_Soll.setValue({x_Richtung, y_Richtung, z_Richtung});*/
		  
		  std::cout << "\nPosition in x_Richtung eingeben \n" << std::endl;
		  cin >> x_Richtung;
		  std::cout << "\nPosition in y_Richtung eingeben \n" << std::endl;
		  cin >> y_Richtung;
		  std::cout << "\nPosition in z_Richtung eingeben \n" << std::endl;
		  cin >> z_Richtung;
		  
		  cSControll.x_Soll.setValue({x_Richtung, y_Richtung, z_Richtung});
		  
		  
		  
		}//end if Taster 1
		
		
		
		
		
		
		if (!TasterRes2.get()){
		  std::cout << "\nReset Encoder \n" << std::endl;

		  mot1Enc.reset();
		  mot2Enc.reset();
		  mot3Enc.reset();
		  
		  
		}//end if Taster 2
		
		
		
		if(!TasterRes3.get()){

		  toggle = !toggle;
 
		  
		  std::cout << "\nToggle" << toggle <<  " "<< std::endl;
		  
		  Mot1IO1.set(toggle);		//set output to 3.3 V
		  Mot2IO1.set(toggle);		//set output to 3.3 V
		  Mot3IO1.set(toggle);		//set output to 3.3 V
  
		}//end if Taster 3

		
		sleep(1);
		//usleep(1000);
	}
	
	

	cSControll.stop();
	cSControll.join();
	
	
	
	return 0;	
}
