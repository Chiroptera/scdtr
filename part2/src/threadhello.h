# ifndef THREADHELLO_H
# define THREADHELLO_H

using namespace std;

// create arduino CLASS. The parameters are the five values read.
class Arduino{
 private: int LL = 0; 	// decimal code of the LDR 00 -> 1MOhm (dark) 99 -> 1KOhm (bright)
  int PP = 0; 	// decimal code of the proximity measure, in centimeters
  int TT = 0; 	// decimal code of the temperature, in degrees centigrade
  int CC = 0; 	// hexadecimal code of the fan control duty cycle 00->0%  FF->100%
  int DD = 0;	// hexadecimal code of the high power LED duty cycle 00->0%  FF->100%

 public:  void set_parameters(std::string parameters);
  void print();
  string getString();
};

#endif
