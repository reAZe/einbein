
#include "einbein/Regelung/CSController.hpp"
#include <../../eeduro-project/linaro-tc/arm-linux-gnueabihf/libc/usr/include/time.h>


#include <iostream>
#include <ostream>



using namespace einbein;
using namespace eeros::math;
using namespace eeros::control;


CSControll::CSControll(double ts) :
    encoder(ts),
    motorModell(),
    saturation(-9.8, 9.8),	//Vorsicht, Wahl der Saturation. Nicht maximal mögliche Spannung wählen.
    i2DAC(),
    pMotor(),
    vorKinController(),
    trajektorie_xf(ts), trajektorie_yf(ts), trajektorie_zf(ts),
    deMux_Pf0(), deMux_xSoll(), deMuxConstInput(), deMux_Saturation(),
    pDV_xf(0.222*98696.0, 0.05*454.0, 0.05, ts), pDV_yf(0.222*98696.0, 0.055*454.0, 0.05, ts), pDV_zf(0.222*98696.0, 0.05*454.0, 0.05, ts),
    //    pDV_xf(224.0, 440.0*0.5, 0.05, ts), pDV_yf(224.0, 440.0*0.5, 0.05, ts), pDV_zf(224.0, 440.0*0.5, 0.05, ts),
    //    pDV_xf(45.0, 88.0, 0.05, ts), pDV_yf(45.0, 88.0, 0.05, ts), pDV_zf(224.0, 440.0, 0.05, ts),
    mux_FRegler(),
    constInput_f(),
    timedomain("Main time domain", ts, true){
  
      
//Encoder und Vorwärtskinematik     
/*    alpha1.setValue(0.0);
    beta1.setValue(0.0);
    gamma1.setValue(0.0);
      
        
    vorKin.getIn_alpha1().connect(alpha1.getOut());
    vorKin.getIn_beta1().connect(beta1.getOut());
    vorKin.getIn_gamma1().connect(gamma1.getOut()); 
    vorKin.getIn_enc().connect(encoder.getOut_enc());
      
    
    //Add Block to run-Methode
    timedomain.addBlock(&encoder);   
    timedomain.addBlock(&alpha1);
    timedomain.addBlock(&beta1);
    timedomain.addBlock(&gamma1);
    timedomain.addBlock(&vorKin);*/

     
      
//Motor ansteuern (Motormodell)      
/*    F_Soll.setValue({1,1,1}); //M1R M2R M3R 
    motorModell.getIn_FM_Soll().connect(F_Soll.getOut());
    saturation.getIn().connect(motorModell.getOut_IM_Soll());
    i2DAC.getIn_Voltage().connect(saturation.getOut()); 
    
    timedomain.addBlock(&F_Soll);
    timedomain.addBlock(&saturation);
    timedomain.addBlock(&motorModell);
    timedomain.addBlock(&i2DAC);*/
    

//Anschlagsregelung
/*    F_Soll.setValue({0,0,0}); //M1R M2R M3R
   pMotor.getIn_FSoll().connect(F_Soll.getOut());
   pMotor.getIn_enc().connect(encoder.getOut_enc());
   pMotor.getIn_d_enc().connect(encoder.getOut_d_enc());
   motorModell.getIn_FM_Soll().connect(pMotor.getOut_FSollMot());
   saturation.getIn().connect(motorModell.getOut_IM_Soll());
   
   i2DAC.getIn_Voltage().connect(saturation.getOut()); 
   
   
   timedomain.addBlock(&encoder);
   timedomain.addBlock(&F_Soll);
   timedomain.addBlock(&pMotor);
   timedomain.addBlock(&motorModell);
   timedomain.addBlock(&saturation);
   timedomain.addBlock(&i2DAC);*/
      
      
//Vorwärtkinematik mit Kraft   
/*      F_Soll.setValue({0.0,0,0}); //x,y,z
      alpha1.setValue(0);
      beta1.setValue(0);
      gamma1.setValue(0);
      vorKinController.getIn_F_Fuss_vec().connect(F_Soll.getOut());
      vorKinController.getIn_alpha1().connect(alpha1.getOut());
      vorKinController.getIn_beta1().connect(beta1.getOut());
      vorKinController.getIn_gamma1().connect(gamma1.getOut());
      vorKinController.getIn_enc().connect(encoder.getOut_enc());
      
      pMotor.getIn_FSoll().connect(vorKinController.getOut_FMsoll());
      pMotor.getIn_enc().connect(encoder.getOut_enc());
      pMotor.getIn_d_enc().connect(encoder.getOut_d_enc());
      motorModell.getIn_FM_Soll().connect(pMotor.getOut_FSollMot());
      saturation.getIn().connect(motorModell.getOut_IM_Soll());
      
      i2DAC.getIn_Voltage().connect(saturation.getOut());
      
      
      
      
      
            
      timedomain.addBlock(&F_Soll);
      timedomain.addBlock(&alpha1);
      timedomain.addBlock(&beta1);
      timedomain.addBlock(&gamma1);    
      timedomain.addBlock(&encoder);
      timedomain.addBlock(&vorKinController);     
      
      timedomain.addBlock(&pMotor);
      timedomain.addBlock(&motorModell);
      timedomain.addBlock(&saturation);
      timedomain.addBlock(&i2DAC);
      */ 
      


//Vorwärtskinemtaik mit Trajektiorenregelung

      x_Soll.setValue({0.0, 0.0, -0.6}); //x,y,z
      alpha1.setValue(0.0);
      beta1.setValue(0.0);
      gamma1.setValue(0.0);
      T_sprung.setValue(0.4);
      constInput_f.getIn_xSoll().connect(x_Soll.getOut());
      constInput_f.getIn_xStart().connect(vorKinController.getOut_Pf_0());
      //T_sprung der Trajektorie setzen
      trajektorie_xf.getIn_T_sprung().connect(T_sprung.getOut());
      trajektorie_yf.getIn_T_sprung().connect(T_sprung.getOut());
      trajektorie_zf.getIn_T_sprung().connect(T_sprung.getOut());
      //demux Pf0
      deMux_Pf0.getIn().connect(vorKinController.getOut_Pf_0());
      deMuxConstInput.getIn().connect(constInput_f.getOut_xStartConst());
      //demux x_Soll
      deMux_xSoll.getIn().connect(x_Soll.getOut());
      //startwert setzen
      trajektorie_xf.getIn_xf_start().connect(deMuxConstInput.getOut(0));
      trajektorie_yf.getIn_xf_start().connect(deMuxConstInput.getOut(1));
      trajektorie_zf.getIn_xf_start().connect(deMuxConstInput.getOut(2));
      //endwert setzen
      trajektorie_xf.getIn_xf_end().connect(deMux_xSoll.getOut(0));
      trajektorie_yf.getIn_xf_end().connect(deMux_xSoll.getOut(1));
      trajektorie_zf.getIn_xf_end().connect(deMux_xSoll.getOut(2));
      //Regler
      pDV_xf.getIn_xIst_0().connect(deMux_Pf0.getOut(0));
      pDV_xf.getIn_xSoll_0().connect(trajektorie_xf.getOut_x_d());
      pDV_yf.getIn_xIst_0().connect(deMux_Pf0.getOut(1));
      pDV_yf.getIn_xSoll_0().connect(trajektorie_yf.getOut_x_d());
      pDV_zf.getIn_xIst_0().connect(deMux_Pf0.getOut(2));
      pDV_zf.getIn_xSoll_0().connect(trajektorie_zf.getOut_x_d());
      //Mux Regler Kraft
      mux_FRegler.getIn(0).connect(pDV_xf.getOut_F_0());
      mux_FRegler.getIn(1).connect(pDV_yf.getOut_F_0());
      mux_FRegler.getIn(2).connect(pDV_zf.getOut_F_0());      
      //vorwärtskimematik
      vorKinController.getIn_F_Fuss_vec().connect(mux_FRegler.getOut());
      vorKinController.getIn_alpha1().connect(alpha1.getOut());
      vorKinController.getIn_beta1().connect(beta1.getOut());
      vorKinController.getIn_gamma1().connect(gamma1.getOut());
      vorKinController.getIn_enc().connect(encoder.getOut_enc());
      //Anschlagsregelung
      deMux_vorKin.getIn().connect(vorKinController.getOut_FMsoll());
      pMotor.getIn_FSoll().connect(vorKinController.getOut_FMsoll());
      pMotor.getIn_enc().connect(encoder.getOut_enc());
      pMotor.getIn_d_enc().connect(encoder.getOut_d_enc());
      //Motormodell
      motorModell.getIn_FM_Soll().connect(pMotor.getOut_FSollMot());
      saturation.getIn().connect(motorModell.getOut_IM_Soll());   
      deMux_Saturation.getIn().connect(saturation.getOut());
      i2DAC.getIn_Voltage().connect(saturation.getOut());
      
      
      
      
      
            
      timedomain.addBlock(&x_Soll);
      timedomain.addBlock(&alpha1);
      timedomain.addBlock(&beta1);
      timedomain.addBlock(&gamma1);
      timedomain.addBlock(&deMuxConstInput);
      timedomain.addBlock(&vorKinController);
      timedomain.addBlock(&deMux_vorKin);
      timedomain.addBlock(&T_sprung);
      timedomain.addBlock(&constInput_f);
      timedomain.addBlock(&trajektorie_xf);
      timedomain.addBlock(&trajektorie_yf);
      timedomain.addBlock(&trajektorie_zf);
      timedomain.addBlock(&deMux_Pf0);
      timedomain.addBlock(&deMux_xSoll);
      timedomain.addBlock(&pDV_xf);
      timedomain.addBlock(&pDV_yf);
      timedomain.addBlock(&pDV_zf);
      timedomain.addBlock(&mux_FRegler);

      timedomain.addBlock(&encoder);
   
      
      timedomain.addBlock(&pMotor);
      timedomain.addBlock(&motorModell);
      timedomain.addBlock(&saturation);
      timedomain.addBlock(&deMux_Saturation);
      timedomain.addBlock(&i2DAC);


    
   printf("\nAdd Block to run-Methode\n");
    
    
   }
  
void CSControll::start(){
    timedomain.start();
    printf("start CSControll\n");
}  


void CSControll::stop(){
    timedomain.stop();
}
 

void CSControll::join(){
    timedomain.join();
} 
 