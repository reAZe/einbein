#ifndef EINBEIN_VORKIN_HPP
#define EINBEIN_VORKIN_HPP


#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>


using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


namespace einbein{
  class VorKin : public eeros::control::Block{
    
    public:
	      VorKin();
	      virtual ~VorKin();
	     
	      //define inputs
	      virtual eeros::control::Input<double>& getIn_alpha1(){return in_alpha1;}
	      virtual eeros::control::Input<double>& getIn_beta1(){return in_beta1;}
	      virtual eeros::control::Input<double>& getIn_gamma1(){return in_gamma1;}
	      virtual eeros::control::Input<Vector3>& getIn_enc(){return in_enc;}	    
	      virtual eeros::control::Input<Vector3>& getIn_F_Fuss_vec(){return in_F_Fuss_vec;}
	      
	      
	      //define outputs
	      virtual eeros::control::Output<Vector3>& getOut_Pf_IMU(){return out_Pf_IMU;}
	      virtual eeros::control::Output<Vector3>& getOut_Pf_0(){return out_Pf_0;}
	      virtual eeros::control::Output<Vector3>& getOut_FMsoll(){return out_FMsoll;}	      
	      
	      
	    
    protected: 
	      //define inputs
	      eeros::control::Input<> in_alpha1;	    
	      eeros::control::Input<> in_beta1;			  
	      eeros::control::Input<> in_gamma1;	  
	      eeros::control::Input<Vector3> in_enc;	    
	      eeros::control::Input<Vector3> in_F_Fuss_vec;
	      
	      //define outputs
	      eeros::control::Output<Vector3> out_Pf_IMU;	      
	      eeros::control::Output<Vector3> out_Pf_0;
	      eeros::control::Output<Vector3> out_FMsoll;
  
	      
    private:  
//-----------------calculateGeoData Methode------------------------------------------------	      
	      virtual void calculateGeoData(Vector3 &P2i_Mi, Vector3 &P3i_Mi,  Vector3 &P6i_Mi,Vector3 &eP2P6_Mi, double &hi, double &sigma_i, double enc_i);
	      //Variablen für Methode calculateGeoData
	      double gamma, Lambda, l, beta, kappa, alpha, z, chi, eta;	
	      eeros::math::Matrix<3,3> R_1_P1_R;
	      eeros::math::Matrix<3,3> R_P5_P6_R;
	      eeros::math::Matrix<3,1> rP2_p1_Mi;
	      
	      

//-----------------calculateP3i2pf Methode------------------------------------------------	 
	      virtual void calculateP3i2pf(Vector3& Pf_IMU, Vector3& ek1_IMU, Vector3& ek2_IMU, Vector3& ek3_IMU, Vector3 P31_IMU,Vector3 P32_IMU, Vector3 P33_IMU);
	      //Variablen für Methode calculateP3i2pf
	      double x31, y31, z31, x32, y32, z32, x33, y33, z33;
	      double x41, y41, z41, x42, y42, z42, x43, y43, z43;
	      double x1_,x2_, x3_, y1_, y2_, y3_, z1_, z2_, z3_;	   
	      double a1, a2, b1, b2, c11, c21, c12, c22;
	      double lambda_1, lambda_2, lambda_3, lambda_4;
	      double p, q, r, xf, yf, zf, zf_positiv, zf_negativ; 
	      eeros::math::Matrix<3,1>  P41_IMU, P42_IMU, P43_IMU;
	      eeros::math::Matrix<3,1> r43_1, r43_2, r43_3;
	      double l_1, l_2, l_3;
	      

    
	      
	      
	      
//-------------------------FPf2F3i Methode------------------------------------------------	      
	      virtual void calculateFPf2F3i(Vector3 &F31_IMU, Vector3 &F32_IMU, Vector3 &F33_IMU, Vector3 &F_Fuss_vec, Vector3 ek1_IMU,Vector3 ek2_IMU, Vector3 ek3_IMU);
	      double F_x, F_y, F_z;
	      double a11, a12, a13, a21, a22, a23, a31, a32, a33;
	      eeros::math::Matrix<3,3> A, B, A_invers;
	      double det_A; 
	      Vector3 F3_skalar;


	      
//-------------------------F3i2FMi Methode------------------------------------------------
	      virtual void calculateF3i2FMi(double &FMi, Vector3 F31_Mi, double hi, double sigma_i);
	      eeros::math::Matrix<3,3> R_Mi_P1_R;
	      Vector3 rP3iP1i_Mi, tau1_Mi_vec;
	      double tau_1;

	      
	      
	      
//-----------------Rotationsmatrix R_0_IMU_R------------------------------------------------
	      eeros::math::Matrix<3,3> R_0_IMU_R;
	      eeros::math::Matrix<3,3> R_0_IMU_R_rotX;
	      eeros::math::Matrix<3,3> R_0_IMU_R_rotY;
	      eeros::math::Matrix<3,3> R_0_IMU_R_rotZ;
	      
//-----------------Rotationsmatrix R_IMU_FF_R------------------------------------------------
	      eeros::math::Matrix<3,3> R_IMU_FF_R, R_0_FF_R;
	      eeros::math::Matrix<3,3> R_IMU_FF_R_rotX;
	      eeros::math::Matrix<3,3> R_IMU_FF_R_rotY;
     
	      
		
//-----------------run Methode------------------------------------------------	      
	      //run
	      virtual void run();	     	      
	      //Input
	      double alpha1, beta1, gamma1, enc1, enc2, enc3;
	      
	      //Werte von geoData
	      //M1
	      Vector3 P11_M1, P21_M1, P31_M1, P51_M1, P61_M1, eP2P6_M1;
	      double h1, sigma_1;
	      //M2
	      Vector3 P12_M2, P22_M2, P32_M2, P52_M2, P62_M2, eP2P6_M2;
	      double h2, sigma_2;
	      //M3
	      Vector3 P13_M3, P23_M3, P33_M3, P53_M3, P63_M3, eP2P6_M3;
	      double h3, sigma_3;
	      //IMU
	      Vector3 P31_IMU, P32_IMU, P33_IMU;
	      //Fusspunkt im KS{IMU}
	      Vector3 Pf_IMU, ek1_IMU, ek2_IMU, ek3_IMU;
	      //Fusspunkt Im KS{0}
	      Vector3 Pf_0;
	      //Winkel 
	      double alpha2_IMU, beta2_IMU, alpha2_0, beta2_0;
	      
	      //Kräfte
	      Vector3 F_Fuss_vec,F_Fuss_vec_0,F31_IMU, F32_IMU, F33_IMU;
	      Vector3 F31_M1, F32_M2, F33_M3, FMsoll;
	      double FM1, FM2, FM3;
	   

    
  };//end class VorKIn
  
  
}//end namspace einbein

#endif // EINBEIN_VORKIN_HPP
