#ifndef EINBEIN_CONSTANTPMOTOR_HPP
#define EINBEIN_CONSTANTPMOTOR_HPP


//includes
#include <eeros/math/Matrix.hpp>
using namespace eeros::math;

    static const double a_max 	= 5;
    static const Vector3 x_max1 = {0.01,0.01,0.01};
    static const Vector3 x_max2 = {0.09,0.09,0.09};
    static const Vector3 kp1	= {150.0, 150.0, 150.0};
    static const Vector3 kp2	= {150.0, 150.0, 150.0};


#endif // end CONSTANTPMOTOR