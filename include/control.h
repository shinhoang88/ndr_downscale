#ifndef __CONTROL__
#define __CONTROL__
#include "Eigen/Dense"
using namespace Eigen;


 typedef class Control{
	private:
		double errp[3];
		double angle[3];
	public:
		double enc2[3];
		double joint[3];
		double err[3];

		void PDcontrol(invKIN *A);
}pdCON;
#endif
