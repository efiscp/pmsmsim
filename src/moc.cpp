#include <iostream>
#include "Motor.hpp"
#include "Foc.hpp"

#include <locale>

int main(void) {
	const uint32_t motorUpdatePeriod    = 100;			//100 ns
	const uint32_t pwmUpdatePeriod      = 100;			//100 ns
	const uint32_t focUpdatePeriod	    = 10000;		//10 us
	const uint32_t displayPeriod	    = 1000000;		//1 ms
	const uint32_t NS_IN_S				= 1000000000;	//1s = 1 billion ns
	const uint32_t pwmPeriod			= 20000;		//20KHz

	const uint64_t simSec 				= 4;			//4s

	//set decimal separator according to locale
	std::locale::global(std::locale( "" ));
	std::cout.imbue(std::locale( "" ));

	Motor m(0, (float)motorUpdatePeriod/(float)NS_IN_S);
	Motor::Output MO;
	Foc f;
	PwmDriver p(NS_IN_S/pwmUpdatePeriod, pwmPeriod);

	//set id = 0, iq != 0 (i.e. create torque)
	f.setDesiredCurrent(0, 10);

	PwmDriver::VoltageVector v;

	//start PWM driver and print header for CSV
	p.start();
	std::cout<<"time;fi;omega;U;V;W;"<<std::endl;

	//main simulation loop
	for(uint64_t i=0; i<NS_IN_S*simSec; ++i){

		//update FOC model
		if((i % focUpdatePeriod) == 0)
			f.update(MO, p);

		//update PWM controller
		if((i % pwmUpdatePeriod) == 0)
			p.update(v);

		//update motor model
		if((i % motorUpdatePeriod) == 0)
			m.update(v.U, v.V, v.W, 0.001, MO);

		//print some data
		if(i % displayPeriod == 0){
			uint16_t U, V, W;
			p.getDuty(U, V, W);
			std::cout<<(float)i/(float)NS_IN_S<<";"<<MO.fi*360/(2*M_PI)<<";"<<MO.omega*10<<";"<<U<<";"<<V<<";"<<W<<";"<<std::endl;
		}
	}

	return 0;
}
