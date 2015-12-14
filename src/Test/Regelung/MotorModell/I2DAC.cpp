#include "einbein/Regelung/MotorModell/I2DAC.hpp"


using namespace einbein;
using namespace eeros::math;
using namespace eeros::hal;



I2DAC::I2DAC() :
outDAC{
	HAL::instance().getRealPeripheralOutput("DACMot1"),
	HAL::instance().getRealPeripheralOutput("DACMot2"),
	HAL::instance().getRealPeripheralOutput("DACMot3")}{}


I2DAC::~I2DAC() { }


void I2DAC::run()
{
	voltage = in_Voltage.getSignal().getValue(); 

		
	outDAC[2]-> set(voltage(0));
	outDAC[0]-> set(voltage(1));
	outDAC[1]-> set(voltage(2));

}//end run
