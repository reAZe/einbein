#ifndef EINBEIN_CONSTANTZUSTBESTHPP
#define EINBEIN_CONSTANTZUSTBESTHPP



namespace einbein{
  /* ----------------------------------------------------------
   * | Konstanten für die Zustandsbestimmung des Hüpfroboters |
   * ----------------------------------------------------------
   */
  

  //Variabeln für Zustandsbestimmung zwischen Boden und Luft (Absprung/Landung)
  static const double toleranzLuft    		  = 0.13;         
  static const double toleranzBodenGeschw         = -0.8;
  static const double toleranzScheitelBodenGeschw = 0.008;		//maximal detektierbare Geschwindigkeit bei Ts = 1ms => 4.4 mm/s
  static const double time_Landung 		  = 0.002;			//Wartezeit Landung in sec
  static const double time_Scheitelpunkt 	  = 0.015;			//Wartezeit Scheitelpunkt in sec
  static const double time_Absprung 	          = 0.002;			//Wartezeit Absprung in sec
  
  //0.005 //0.015
}


#endif // EINBEIN_CONSTANTZUSTBESTHPP