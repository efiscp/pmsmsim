#ifndef PWMDRIVER_HPP_
#define PWMDRIVER_HPP_

#include <cstdint>

class PwmDriver{
public:
	struct VoltageVector{
		bool U;
		bool V;
		bool W;
	};

	PwmDriver(uint32_t callFreq, uint32_t pwmFreq):callFrequency(callFreq),pwmFrequency(pwmFreq){

		//calculate tick count based on PWM frequency and call frequency
		//note: value is divided by 2 because full PWM period is split into upcounting and downcounting phases
		pwmPeriodCnt = callFreq/pwmFreq/2;	//-1 in STM32
	}

	/**
	 * set timing parameters that will be fetched at start of new PWM cycle
	 *
	 * note: despite its name, these are not duty cycles
	 *
	 * @param dutyU
	 * @param dutyV
	 * @param dutyW
	 */
	void setNewDuty(uint16_t dutyU, uint16_t dutyV, uint16_t dutyW){
		dutyShadow_U = dutyU;
		dutyShadow_V = dutyV;
		dutyShadow_W = dutyW;

		//refresh timing info if possible
		if(pwmTick == 0)
			pullNewDuty();
	}

	/**
	 * return duty cycle for logging
	 *
	 * @param U
	 * @param V
	 * @param W
	 */
	void getDuty(float& U, float& V, float& W){
		U = ((float)(pwmPeriodCnt - duty_U))/(float)pwmPeriodCnt;
		V = ((float)(pwmPeriodCnt - duty_V))/(float)pwmPeriodCnt;
		W = ((float)(pwmPeriodCnt - duty_W))/(float)pwmPeriodCnt;
	}

	/**
	 * start PWM pattern generator
	 */
	void start(void){
		pullNewDuty();
		pwmTick = 0;
		pwmState = COUNTING_UP;
	}

	/**
	 * refresh PWM output
	 *
	 * @param vector
	 */
	void update(VoltageVector& vector){
		vector.U = 0;
		vector.V = 0;
		vector.W = 0;

		//up-down counting counter for center-aligned PWM
		if(pwmState == COUNTING_UP){
			++pwmTick;

			//high limit reached - start counting down from next call
			if(pwmTick >= pwmPeriodCnt)
				pwmState = COUNTING_DOWN;

		}else if(pwmState == COUNTING_DOWN){
			--pwmTick;

			//low limit reached - start counting up from next call
			if(pwmTick == 0){
				pwmState = COUNTING_UP;
				//also, refresh pattern timing
				pullNewDuty();
			}
		}else
			return;

		//apply phase voltage if internal tick is above threshold
		if(duty_U <= pwmTick)
			vector.U = 1;
		if(duty_V <= pwmTick)
			vector.V = 1;
		if(duty_W <= pwmTick)
			vector.W = 1;
	}

	/**
	 * PWM cycle in ticks for PWM timing calculations in FOC module
	 * @return
	 */
	uint16_t getPwmPeriodCnt(void){
		return pwmPeriodCnt;
	}
private:

	/**
	 * refresh timing info from shadow buffers
	 */
	void pullNewDuty(void){
		duty_U = dutyShadow_U;
		duty_V = dutyShadow_V;
		duty_W = dutyShadow_W;
	}

	uint32_t callFrequency;
	uint32_t pwmFrequency;

	//time info for PWM pattern generation
	uint16_t duty_U = 0;
	uint16_t duty_V = 0;
	uint16_t duty_W = 0;

	//timing info is updated at the end of the PWM period - use double buffering
	uint16_t dutyShadow_U;
	uint16_t dutyShadow_V;
	uint16_t dutyShadow_W;

	//main counter for PWM pattern generation
	uint16_t pwmTick;

	enum PwmState{
		OFF,
		COUNTING_UP,
		COUNTING_DOWN,
	};

	PwmState pwmState = OFF;

	uint16_t pwmPeriodCnt;

};



#endif /* PWMDRIVER_HPP_ */
