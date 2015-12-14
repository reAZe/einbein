#include <cmath>
#include <einbein/Regelung/Trajektorie/constInput.hpp>
#include <einbein/templates_function.hpp>


#include <iostream>
#include <ostream>


//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;



//Konstruktor
constInput::constInput(){
  xStart 	= Vector3{0,0,0};
  xSoll_1  	= 0;
  out_xStartConst.getSignal().setValue(xStart);
  
  xStartConst  = in_xStart.getSignal().getValue();
  
};

//Destruktor
constInput::~constInput(){};



void constInput::run(){
//-----------------------------  set Input-------------------------------------------------
  xSoll	= in_xSoll.getSignal().getValue(); 
  xStart= in_xStart.getSignal().getValue();
  
   if (xSoll != xSoll_1){
     xStartConst = xStart;
     xSoll_1 = xSoll;
  }
  
  
//-----------------------------  set Output ----------------------------------------------- 

  out_xStartConst.getSignal().setValue(xStartConst);
   
  //set Timestamp
  out_xStartConst.getSignal().setTimestamp(in_xSoll.getSignal().getTimestamp());
  

  
  
}//end run