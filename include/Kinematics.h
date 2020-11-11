#ifndef __KINEMATICS__
#define __KINEMATICS__
#include "Eigen/Dense"
#include <stdio.h>
using namespace Eigen;


typedef class Kinematics{
	private:
			
	public:
		double posX;
		double posY;
		double posZ;
		
		double angle[3];
		void inputPos(); // Kinematics 클래스 안에 함수를 넣은 것.
		void invKinematics();
		void f_Kinematics(); //forward kineamtics
}invKIN; //이 함수를 간단히 이렇게 부르는 거 ???
#endif
