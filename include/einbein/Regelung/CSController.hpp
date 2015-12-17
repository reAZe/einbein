#ifndef EINBEIN_CSCONTROLL_HPP
#define EINBEIN_CSCONTROLL_HPP


#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/Mux.hpp>
#include <einbein/Regelung/Encoder/Encoder.hpp>
#include "einbein/Regelung/Base2Tool/VorKin.hpp"
#include "einbein/Regelung/Encoder/TransitionBlockEncoder.hpp"
#include "einbein/Regelung/MotorModell/MotorModell.hpp"
#include "einbein/Regelung/MotorModell/I2DAC.hpp"
#include "einbein/Regelung/PMotor/PMotor.hpp"
#include <eeros/control/Saturation.hpp>
#include "einbein/Regelung/Base2Tool/VorKin.hpp"
#include "einbein/Regelung/Trajektorie/Trajektorie.hpp"
#include "einbein/Regelung/PDV/PDV.hpp"
#include "einbein/Regelung/Trajektorie/constInput.hpp"




namespace einbein{

class CSControll{
  public:
  CSControll(double ts);
  MotorModell motorModell;
  eeros::control::Saturation<Vector3>  saturation;
  Encoder encoder;
  VorKin vorKin;
  I2DAC i2DAC;
  PMotor pMotor; 
  VorKin vorKinController;
  Trajektorie trajektorie_xf;
  Trajektorie trajektorie_yf;
  Trajektorie trajektorie_zf;
  PDV pDV_xf;
  PDV pDV_yf;
  PDV pDV_zf;
  constInput constInput_f;
  
   
  eeros::control::Constant<Vector3> F_Soll;
  eeros::control::Constant<Vector3> x_Soll;
  eeros::control::Constant<double> alpha1;
  eeros::control::Constant<double> beta1;
  eeros::control::Constant<double> gamma1;
  eeros::control::Constant<double> T_sprung;
  eeros::control::DeMux<3, double> deMux_xSoll;
  eeros::control::DeMux<3, double> deMux_Pf0;
  eeros::control::DeMux<3, double> deMux_vorKin;
  eeros::control::DeMux<3, double> deMux_Saturation;
  eeros::control::DeMux<3, double> deMuxConstInput;
  eeros::control::DeMux<3, double> deMuxdEncoder;
  eeros::control::DeMux<3, double> deMuxEncoder;
  eeros::control::Mux<3, double> mux_FRegler;
 
  
  void start();
  void stop();
  void join();
  
   
  private:
  eeros::control::TimeDomain timedomain;

};

}

#endif // EINBEIN_CSCONTROLL_HPP