#ifndef FOC_HPP_
#define FOC_HPP_

#include "Motor.hpp"
#include "pwmdriver.hpp"

class Foc{

public:

	float lastDerror = 0, lastQerror = 0;

	/**
	 * store desired DQ current
	 *
	 * @param desiredD
	 * @param desiredQ
	 */
	void setDesiredCurrent(float desiredD, float desiredQ){
		desiredDcurrent = desiredD;
		desiredQcurrent = desiredQ;
	}

	/**
	 * calculate next voltage vector
	 *
	 * @param feedback motor output
	 * @param pwm driver which manages voltage vector request
	 */
	void update(Motor::Output& feedback, PwmDriver& pwm){
		float i_alpha, i_beta, i_d, i_q;

		//do clarke/park transformation
		clarke(feedback.I_U, feedback.I_V, feedback.I_W, i_alpha, i_beta);
		park(feedback.fi, i_alpha, i_beta, i_d, i_q);

		//run PI controller for DQ currents
		PI(i_d, i_q);

		//do inverse park transform
		inversePark(feedback.fi, i_d, i_q, i_alpha, i_beta);

		//calculate space vector for PWM driver
		foc(i_alpha, i_beta, pwm);
	}

private:

	/**
	 * classic (non-power invariant) clarke transformation algorithm
	 *
	 * @param i_U phase current input
	 * @param i_V phase current input
	 * @param i_W phase current input
	 * @param i_alpha current output
	 * @param i_beta current output
	 */
	void clarke(float i_U, float i_V, float i_W, float& i_alpha, float& i_beta){
		//TODO: optimize
		i_alpha = 2.f/3.f * (i_U - 0.5*i_V - 0.5*i_W);
		i_beta = 2.f/3.f * (SQRT3_2 * i_V - SQRT3_2 * i_W);
	}

	/**
	 * park transformation
	 *
	 * @param fi rotor angle
	 * @param i_alpha current input in alpha-beta reference frame
	 * @param i_beta current input in alpha-beta reference frame
	 * @param i_d direct component output
	 * @param i_q quadratic component output
	 */
	void park(float fi, float i_alpha, float i_beta, float& i_d, float& i_q){
		i_d = std::cos(fi)*i_alpha + std::sin(fi)*i_beta;
		i_q = -std::sin(fi)*i_alpha + std::cos(fi)*i_beta;
	}

	/**
	 * inverse park algorithm
	 *
	 * @param fi rotor angle input
	 * @param i_d direct component input
	 * @param i_q quadratic component input
	 * @param i_alpha output
	 * @param i_beta output
	 */
	void inversePark(float fi, float i_d, float i_q, float& i_alpha, float& i_beta){
		i_alpha = std::cos(fi)*i_d -std::sin(fi)*i_q;
		i_beta = std::sin(fi)*i_d + std::cos(fi)*i_q;
	}

	/**
	 * DQ controller (P controller so far)
	 *
	 * TODO: add integrator
	 *
	 * @param i_d direct component
	 * @param i_q quadratic component
	 */
	void PI(float& i_d, float& i_q){
		//error
		float error_d = desiredDcurrent - i_d;
		float error_q = desiredQcurrent - i_q;

		//integral
		PI_integral_d += PI_I*error_d;
		PI_integral_q += PI_I*error_q;

		//anti-windup limiter
		PI_integral_d = (PI_integral_d > PI_integralMaxAction) ? PI_integralMaxAction : PI_integral_d;
		PI_integral_d = (PI_integral_d < PI_integralMinAction) ? PI_integralMinAction : PI_integral_d;
		PI_integral_q = (PI_integral_q > PI_integralMaxAction) ? PI_integralMaxAction : PI_integral_q;
		PI_integral_q = (PI_integral_q < PI_integralMinAction) ? PI_integralMinAction : PI_integral_q;

		//calculate action
		i_d = error_d*PI_P + PI_integral_d;
		i_q = error_q*PI_P + PI_integral_q;

		//log
		lastDerror = error_d;
		lastQerror = error_q;
	}

	/**
	 * calculate space vector for PWM input
	 *
	 * @param alpha voltage component
	 * @param beta voltage component
	 * @param pwmDriver
	 */
	void foc(float alpha, float beta, PwmDriver& pwmDriver){

		uint8_t sector = 0;

		//calculate vector magnitude
		float m = sqrtf(alpha*alpha + beta*beta);

		if(m > Vmax)
			m = Vmax;

		//calculate vector length
		float angRad = atan2f(beta, alpha);
		if(angRad < 0.f) angRad += 2*M_PI;

		//calculate sector for PWM pattern selection
		for(; sector < 6; ++sector){

			if(angRad - PI_3 <= 0.f)
				break;

			angRad -= PI_3;
		}

		alpha = m*std::cos(angRad);		//rotated vector into sector 0
		beta = m*std::sin(angRad);		//rotated vector into sector 0

		//calculate PWM pattern timing
		float t2_delta = beta * SQRT3_2_INV;
		float t1_delta = pwmDriver.getPwmPeriodCnt()*(alpha - t2_delta*0.5f);		//alpha - v2*cos(60)
		t2_delta *= pwmDriver.getPwmPeriodCnt();

		float t0 = 0.5*(pwmDriver.getPwmPeriodCnt() - t1_delta - t2_delta);

		//simple check for PWM timing violation (TODO: improve)
		if(t0 < 0)
			std::cout<<"PWM ERROR"<<std::endl;

		//select pattern and set duty cycle
		switch(sector){
		case 0:
			pwmDriver.setNewDuty(t0, t0+t1_delta, t0+t1_delta+t2_delta);
			break;
		case 1:
			pwmDriver.setNewDuty(t0+t2_delta, t0, t0+t1_delta+t2_delta);
			break;
		case 2:
			pwmDriver.setNewDuty(t0+t1_delta+t2_delta, t0, t0+t1_delta);
			break;
		case 3:
			pwmDriver.setNewDuty(t0+t1_delta+t2_delta, t0+t2_delta, t0);
			break;
		case 4:
			pwmDriver.setNewDuty(t0+t1_delta, t0+t1_delta+t2_delta, t0);
			break;
		case 5:
			pwmDriver.setNewDuty(t0, t0+t1_delta+t2_delta, t0+t2_delta);
			break;
		default:
			while(true);
		}

	}

	static constexpr float SQRT3_2 = sqrtf(3)/2;		//cos 30
	static constexpr float SQRT3 = sqrtf(3);
	static constexpr float PI_3 = M_PI/3.f;				//60 deg
	static constexpr float SQRT3_2_INV = 1/SQRT3_2;		//1/cos(30)

	float desiredDcurrent = 0;
	float desiredQcurrent = 0;

	float PI_integral_d = 0;
	float PI_integral_q = 0;

	static constexpr float PI_P = 0.6;									//PI controller proportional factor
	static constexpr float PI_I = 0.05;									//PI controller integral factor
	static constexpr float PI_integralMaxAction = 0.5;
	static constexpr float PI_integralMinAction = -0.5;

	static constexpr float Vmax = 0.4f;									//max space vector length (TODO: tune)
};



#endif /* FOC_HPP_ */
