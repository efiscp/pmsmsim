#ifndef FOC_HPP_
#define FOC_HPP_

#include "Motor.hpp"
#include "pwmdriver.hpp"

class Foc{

public:

	//public members for logging output
	float lastDerror = 0, lastQerror = 0, lastOmegaError = 0, lastFiError = 0,
		lastDcurrent = 0, lastQcurrent = 0,
		lastDcurrentRequest = 0, lastQcurrentRequest = 0;

	/**
	 * store desired DQ current
	 *
	 * @param desiredD
	 * @param desiredQ
	 */
	void setDesiredCurrent(float desiredD, float desiredQ){
		desiredDcurrent = desiredD;
		desiredQcurrent = desiredQ;
		controlMode = CURRENT_CONTROL;
	}

	/**
	 * store angle speed reference
	 *
	 * note: the control loop will overwrite desired current in speed control mode
	 *
	 * @param angleSpeed
	 */
	void setDesiredSpeed(float angleSpeed){
		desiredAngleSpeed = angleSpeed;
		controlMode = SPEED_CONTROL;
	}

	/**
	 * store position reference
	 *
	 * note: the control loop will overwrite both the desired angle speed and current in this mode
	 *
	 * not tested
	 *
	 * @param position
	 */
	void setDesiredPosition(float position){
		desiredPosition = position;
		controlMode = POSITION_CONTROL;
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

		//store last known currents for logging
		lastDcurrent = i_d;
		lastQcurrent = i_q;

		//run control loops
		switch(controlMode){
		case POSITION_CONTROL:
			//PI controller for position control - not tested
			lastFiError = PI(desiredPosition, feedback.fi, PI_P_position, PI_I_position, PI_integralMinAction_position, PI_integralMaxAction_position,
									PI_integral_position, desiredAngleSpeed);
			//intentional fall-through
		case SPEED_CONTROL:
			//calculate Q current request with angle speed controller
			lastOmegaError = PI(desiredAngleSpeed, feedback.omega, PI_P_speed, PI_I_speed, PI_integralMinAction_speed, PI_integralMaxAction_speed,
									PI_integral_speed, desiredQcurrent);

			//no field weakening
			desiredDcurrent = 0;

			//store last known current request for logging
			lastDcurrentRequest = desiredDcurrent;
			lastQcurrentRequest = desiredQcurrent;
			//intentional fall-through
		case CURRENT_CONTROL:
			//run PI controller for DQ currents
			lastDerror = PI(desiredDcurrent, i_d, PI_P, PI_I, PI_integralMinAction, PI_integralMaxAction, PI_integral_d, i_d);
			lastQerror = PI(desiredQcurrent, i_q, PI_P, PI_I, PI_integralMinAction, PI_integralMaxAction, PI_integral_q, i_q);
			//intentional fall-through
		default:
			//nothing to do here
			break;
		}

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
	 * stateless PI controller
	 *
	 * @param reference ref signal
	 * @param actual measured signal
	 * @param Kp proportional factor
	 * @param Ki integral factor
	 * @param windupLow integral limiter - low limit
	 * @param windupHigh integral limiter - high limit
	 * @param integral in/output for integral
	 * @param output action
	 * @return error (reference - actual)
	 */
	float PI(const float reference, const float actual, const float Kp, const float Ki,
			const float windupLow, const float windupHigh, float& integral, float& output){
		float error = reference - actual;

		integral += Ki*error;
		integral = (integral > windupHigh) ? windupHigh : integral;
		integral = (integral < windupLow) ? windupLow : integral;

		output = Kp*error + integral;

		return error;
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
			m = Vmax - EPSILON;

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
			std::cout<<"PWM ERROR, t0: "<<t0<<std::endl;

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

	//constants for reference frame transformations
	static constexpr float SQRT3_2 = sqrtf(3)/2;		//cos 30
	static constexpr float SQRT3 = sqrtf(3);
	static constexpr float PI_3 = M_PI/3.f;				//60 deg
	static constexpr float SQRT3_2_INV = 1/SQRT3_2;		//1/cos(30)

	//reference signals
	float desiredDcurrent = 0;
	float desiredQcurrent = 0;
	float desiredAngleSpeed = 0;
	float desiredPosition = 0;

	//integrals for PI controllers
	float PI_integral_d = 0;
	float PI_integral_q = 0;
	float PI_integral_speed = 0;
	float PI_integral_position = 0;


	static constexpr float PI_P = 0.6;								//PI current controller proportional factor
	static constexpr float PI_I = 0.08;								//PI current controller integral factor
	static constexpr float PI_integralMaxAction = 0.8;				//PI current integral limiter
	static constexpr float PI_integralMinAction = -0.8;				//PI current integral limiter

	static constexpr float PI_P_position = 0.6;						//PI position controller proportional factor (not tuned)
	static constexpr float PI_I_position = 0.05;					//PI position controller integral factor (not tuned)
	static constexpr float PI_integralMaxAction_position = 0.5;		//PI position integral limiter
	static constexpr float PI_integralMinAction_position = -0.5;	//PI position integral limiter

	static constexpr float PI_P_speed = 15;							//PI angle speed controller proportional factor
	static constexpr float PI_I_speed = 2;							//PI angle speed controller integral factor
	static constexpr float PI_integralMaxAction_speed = 3;			//PI angle speed integral limiter
	static constexpr float PI_integralMinAction_speed = -3;			//PI angle speed integral limiter

	static constexpr float EPSILON = 0.00001f;						//for handling float inaccuracy
	static constexpr float Vmax = sqrtf(3)/2;						//max space vector length without overmodulation

	enum ControlMode{
		OFF,
		CURRENT_CONTROL,
		SPEED_CONTROL,
		POSITION_CONTROL,
	};

	ControlMode controlMode = OFF;	//global mode control
};



#endif /* FOC_HPP_ */
