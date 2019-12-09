#pragma once
#include "common/PidController.hpp"

class PidControl : msr::airlib::PidController
{
public:
	PidControl(float p, float i, float d) {
		kProportional_ = p;
		kIntegral_ = i;
		kDerivative_ = d;
	};
	void update(double  current_time, double current_velocity, double desired_velocity) {


	};
	~PidControl();
};
