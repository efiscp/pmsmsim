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

	}

	/**
	 * get pattern timing info
	 *
	 * TODO: return real duty cycle calculated from these values
	 *
	 * @param U
	 * @param V
	 * @param W
	 */
	void getDuty(uint16_t& U, uint16_t& V, uint16_t& W){
		U = duty_U;
		V = duty_V;
		W = duty_W;
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

			if(pwmTick >= pwmPeriodCnt)
				pwmState = COUNTING_DOWN;

		}else if(pwmState == COUNTING_DOWN){
			--pwmTick;

			if(pwmTick == 0){
				pwmState = COUNTING_UP;

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

	uint16_t getPwmPeriodCnt(void){
		return pwmPeriodCnt;
	}
private:

	void pullNewDuty(void){
		duty_U = dutyShadow_U;
		duty_V = dutyShadow_V;
		duty_W = dutyShadow_W;
	}

	uint32_t callFrequency;
	uint32_t pwmFrequency;

	uint16_t duty_U;
	uint16_t duty_V;
	uint16_t duty_W;

	uint16_t dutyShadow_U;
	uint16_t dutyShadow_V;
	uint16_t dutyShadow_W;

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
