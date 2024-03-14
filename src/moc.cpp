#include <iostream>
#include "Motor.hpp"
#include "Foc.hpp"

#include <locale>

/**
 * simple rate limiter
 *
 * @param desiredSpeed target speed
 * @param currentSpeed actual speed
 * @param step max difference between actual speed and output
 * @return ramped speed
 */
float rampSpeed(const float desiredSpeed, const float currentSpeed, const float step){

	float sign = (desiredSpeed >= currentSpeed) ? 1.f : -1.f;
	float delta = std::abs(desiredSpeed - currentSpeed);
	delta = (delta > step) ? step : delta;

	return currentSpeed + sign*delta;
}

int main(void) {

	const uint32_t timebase				= 10000000;		// 1/timebase = elapsed time/tick in sec (100ns)
	const uint32_t motorUpdatePeriod    = 1;			//100ns
	const uint32_t pwmUpdatePeriod      = 1;			//100ns
	const uint32_t focUpdatePeriod	    = 100;			//10us
	const uint32_t displayPeriod	    = 1000;			//100us
	const uint32_t pwmFreq				= 20000;		//20KHz
	//const uint32_t pwmPeriod			= NS100 / pwmFreq; <- calculated by pwm driver


	//max angle acceleration rate, limited by motor at around 400 rad/s^2 (electrical)
	const float accelerationRate_radps = 300;
	const float delta_per_focPeriod = accelerationRate_radps*(float)focUpdatePeriod/(float)timebase;

	const uint64_t simSec 				= 4;			//4s

	//set initial angle speed request to 200 1/s (electrical)
	float desiredSpeed = 200.f;
	float rampedSpeedRequest = 0;

	//helper objects
	Motor::Output MO;
	PwmDriver::VoltageVector v;


	//instantiate components
	Motor m(0, (float)timebase, (float)motorUpdatePeriod);
	Foc f;
	PwmDriver p(timebase, pwmFreq);

	//set id = 0, iq != 0 (i.e. create torque), max possible is around 15A@36V
	//f.setDesiredCurrent(0, 14.5);

	//set speed reference & put controller to speed control mode
	f.setDesiredSpeed(desiredSpeed);

	//start PWM driver and print header for CSV
	p.start();
	std::cout<<"time;fi;omega;U;V;W;error_d;error_q;error_omega;speed;i_d;i_q;T;omega_req;i_d_req;i_q_req;"<<std::endl;

	//main simulation loop
	for(uint64_t i=0; i<timebase*simSec; ++i){

		//update FOC model
		if((i % focUpdatePeriod) == 0){
			rampedSpeedRequest = rampSpeed(desiredSpeed, rampedSpeedRequest, delta_per_focPeriod);
			f.setDesiredSpeed(rampedSpeedRequest);
			f.update(MO, p);
		}

		//update PWM controller
		if((i % pwmUpdatePeriod) == 0)
			p.update(v);

		//update motor model
		if((i % motorUpdatePeriod) == 0)
			m.update(v.U, v.V, v.W, 0.f, MO);

		//print data
		if(i % displayPeriod == 0){
			float U, V, W;
			p.getDuty(U, V, W);
			std::cout<<(float)i/(float)timebase<<";"<<MO.fi*360/(2*M_PI)<<";"<<MO.omega<<";"
					<<U*Motor::BRIDGE_VOLTAGE<<";"<<V*Motor::BRIDGE_VOLTAGE<<";"<<W*Motor::BRIDGE_VOLTAGE<<";"
					<<f.lastDerror<<";"<<f.lastQerror<<";"<<f.lastOmegaError<<";"<<1.8f*MO.omega_mech*0.2<<";"
					<<f.lastDcurrent<<";"<<f.lastQcurrent<<";"<<MO.T<<";"
					<<rampedSpeedRequest<<";"<<f.lastDcurrentRequest<<";"<<f.lastQcurrentRequest<<";"<<std::endl;
		}

		//modify speed after 2.5 sec
		if(i == timebase*2.5f){
			desiredSpeed = 120.f;
			f.setDesiredSpeed(desiredSpeed);
		}

	}

	return 0;
}
