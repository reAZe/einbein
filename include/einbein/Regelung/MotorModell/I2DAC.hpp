#ifndef EINBEIN_I2DAC_HPP
#define EINBEIN_I2DAC_HPP


#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include <eeros/hal/PeripheralOutput.hpp>
#include <eeros/hal/HAL.hpp>
#include <cmath>

using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::math;


namespace einbein
{

	class I2DAC : public eeros::control::Block
	{
	public:
		I2DAC();
		virtual ~I2DAC();
		
		virtual eeros::control::Input<Vector3>& getIn_Voltage() {return in_Voltage;}
		
		virtual double getOut_DAC1() {return outDAC[0]->get();}
		virtual double getOut_DAC2() {return outDAC[1]->get();}
		virtual double getOut_DAC3() {return outDAC[2]->get();}
	
		
	protected:
		eeros::control::Input<Vector3> in_Voltage;
		eeros::hal::PeripheralOutput<double>* outDAC[3];

		
	private:
		virtual void run();
		Vector3 voltage;
		
	};
	
}


#endif //EINBEIN_I2DAC_hpp