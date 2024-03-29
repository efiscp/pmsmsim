#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include <iostream>
#include <cmath>

class Motor{

private:
	float TIME_DELTA;
	float timebase;
public:
	// +++ electrical params +++
	//grid
	static constexpr float BRIDGE_VOLTAGE = 36.f;
	static constexpr float HALF_BRIDGE_VOLTAGE = BRIDGE_VOLTAGE / 2;
	static constexpr float QUATER_BRIDGE_VOLTAGE = HALF_BRIDGE_VOLTAGE / 2;

	//motor
	static constexpr uint8_t POLEPAIR = 15;
	static constexpr float R_COIL = 0.294f;		//coil resistance in ohm
	static constexpr float L_COIL = 0.000352f;	//coil inductance in H

	//measured: 18Vpp@34.3Hz (electric rotation/s): KV =  ((18/2)V / (34.3Hz * 2PI))
	static constexpr float KE_A_ELECT = -0.041760772f;			//EMF constant - peak from mean [V/(rad/s)]

	// +++ mechanical params +++
	static constexpr float KT_A_ELECT = KE_A_ELECT;				//torque constant - assuem no losses
	static constexpr float B = 0.f;							//viscous damping ratio (oscillation decay rate)

	//value got from https://discourse.odriverobotics.com/t/project-hoverarm/441
	static constexpr float J = 0.00701;					//moment of inertia (Nms^2 or kgm^2)

	//output struct with aggregated info
	struct Output{
		//current
		float I_U = 0;
		float I_V = 0;
		float I_W = 0;

		//torque
		float T = 0;

		//angle & angle speed
		float fi = 0;
		float omega = 0;

		//mechanical angle & angle speed
		float fi_mech = 0;
		float omega_mech = 0;

		//print some debug info if needed
		void print(void){
			std::cout<<std::cout.precision(8);
			std::cout<<"I_U:        "<<I_U<<std::endl;						//A
			std::cout<<"I_V:        "<<I_V<<std::endl;						//A
			std::cout<<"I_W:        "<<I_W<<std::endl;						//A
			std::cout<<"T:          "<<T<<std::endl;						//Nm
			std::cout<<"fi:         "<<radToDeg(fi)<<std::endl;          //deg
			std::cout<<"omega:      "<<omega<<std::endl;					//rad/s
			std::cout<<"fi_mech:    "<<radToDeg(fi_mech)<<std::endl;		//deg
			std::cout<<"omega_mech: "<<omega_mech<<std::endl;				//rad/s
			std::cout<<std::endl;
		}
	};

	/**
	 * constructor
	 *
	 * @param f initial electrical angle (should be set to 0)
	 * @param period refresh rate of motor model
	 */
	Motor(float f, float tb, float period):fi(f),fi_prev(f),fi_mech(f/POLEPAIR),timebase(tb),TIME_DELTA(period){}

	/**
	 * refresh motor model
	 *
	 * @param U phase voltage input
	 * @param V phase voltage input
	 * @param W phase voltage input
	 * @param torque_L load torque input
	 * @param O electrical & mechanical output of the motor
	 */
	void update(bool U, bool V, bool W, float torque_L, Output& O){

		//prepare integrators
		fi_prev = fi;
		omega_prev = omega;

		I_PREV_U = I_U;
		I_PREV_V = I_V;
		I_PREV_W = I_W;

		//calculate phase voltages
		VoltageVector v = VOLTAGE_TABLE[getVector(U, V, W)];

		//store phase voltages
		V_U = v.U;
		V_V = v.V;
		V_W = v.W;

		//save load torque
		T_Load = torque_L;

		//calculate internal model
		calcBemf();
		calcCurrent();
		calcTorque();
		calcMech();

		//prepare output info
		O.I_U = I_U;
		O.I_V = I_V;
		O.I_W = I_W;
		O.T = T_Electromagnetic;
		O.fi = fi;
		O.omega = omega;
		O.fi_mech = fi_mech;
		O.omega_mech = omega_mech;
	}

private:

	/**
	 * convert U, V, W to bitfield
	 *
	 * @param U
	 * @param V
	 * @param W
	 * @return
	 */
	inline uint8_t getVector(bool U, bool V, bool W){
		return U<<2 | V << 1 | W;
	}

	/**
	 * equation for phase back EMF voltage calculation according to the following formula:
	 *
	 * Vemf = Ke*omega*sin(fi)
	 *
	 * where Ke is the EMF constant
	 *
	 * @param angle rotor angle
	 * @param angvel rotor angle speed
	 * @return
	 */
	inline float calcPhaseBemf(float angle, float angvel){
		return KE_A_ELECT * angvel * sin(angle);
	}

	/**
	 * calculate back EMF voltages for each phase considering phase angles
	 */
	void calcBemf(void){
		BEMF_U = calcPhaseBemf(fi - PHASE_OFFSET_U, omega);
		BEMF_V = calcPhaseBemf(fi - PHASE_OFFSET_V, omega);
		BEMF_W = calcPhaseBemf(fi - PHASE_OFFSET_W, omega);
	}

	//V = R*i + L*di/dt + e
	/**
	 * calculate single phase current according to the well known motor voltage equation:
	 *
	 * V = R*i + L*di/dt + EMF
	 *
	 * rearranging the formula yields the phase current
	 *
	 * di = 1/L * (V - R*i - EMF)dt			//integrate
	 *
	 * i = 1/L * integral(V - R*i - EMF)dt	//move to discrete domain
	 *
	 * i = accumulated current + 1/L * (V - R*i - EMF)*TIME_DELTA
	 *
	 * @param voltage total voltage of the phase
	 * @param bemf calculated back EMF voltage
	 * @param prevCurrent accumulated current for the integrator
	 * @return
	 */
	inline float calcPhaseCurrent(float voltage, float bemf, float prevCurrent){
		float tmp = voltage - R_COIL*prevCurrent - bemf;	//add
		return prevCurrent + tmp/L_COIL*TIME_DELTA/timebase;					//accumulate
	}

	/**
	 * calculate phase currents
	 */
	void calcCurrent(void){
		I_PREV_U = I_U;
		I_PREV_V = I_V;
		I_PREV_W = I_W;

		I_U = calcPhaseCurrent(V_U, BEMF_U, I_PREV_U);
		I_V = calcPhaseCurrent(V_V, BEMF_V, I_PREV_V);
		I_W = calcPhaseCurrent(V_W, BEMF_W, I_PREV_W);
	}

	/**
	 * calculate torque generated by phase according to the torque equation:
	 *
	 * T = Kt * i * sin(fi)
	 *
	 * where Kt is the torque constant
	 *
	 * @param phaseCurrent
	 * @param angle
	 * @return
	 */
	inline float calcPhaseTorque(float phaseCurrent, float angle){
		return KT_A_ELECT * phaseCurrent * sin(angle);
	}

	/**
	 * calculate total torque
	 */
	void calcTorque(void){
		T_U = calcPhaseTorque(I_U, fi - PHASE_OFFSET_U);
		T_V = calcPhaseTorque(I_V, fi - PHASE_OFFSET_V);
		T_W = calcPhaseTorque(I_W, fi - PHASE_OFFSET_W);

		T_Electromagnetic = T_U + T_V + T_W;
	}

	/**
	 * calculate mechanical variables according to the following equation
	 *
	 * T = TL + J * d/dt * omega + B * omega
	 * where
	 *     T is the total electric torque
	 *     TL is the load torque
	 *     J is the moment of inertia (resistance to angle acceleration)
	 *     B is the viscous damping
	 *
	 * in order to get the angle speed, the equation is re-arranged:
	 *
	 * d omega = 1/J * (T - TL - B * omega) * dt
	 * omega = 1/J * integral(T - TL - B * omega)*dt
	 *
	 * discrete case:
	 * omega = omega_prev + 1/J * (T - TL - B * omega_prev)*TIME_DELTA
	 *
	 * finally, angle is the integral of omega:
	 *
	 * fi = fi_prev + omega*TIME_DELTA
	 */
	void calcMech(void){
		float tmp = T_Electromagnetic - T_Load - B*omega_prev;
		omega = omega_prev + tmp/J*TIME_DELTA/timebase;
		fi = fi_prev + omega*TIME_DELTA/timebase;
		fi_mech += (fi - fi_prev)/POLEPAIR;
		omega_mech = omega/POLEPAIR;

		fi = normalizeRad(fi);
		fi_mech = normalizeRad(fi_mech);
	}

	/**
	 * reduce angle into 0, 2pi rad range
	 * @param rad
	 * @return
	 */
	static float normalizeRad(float rad){
		while(rad >= 2*M_PI)
			rad -= 2*M_PI;
		while(rad < 0.f)
			rad += 2*M_PI;

		return rad;
	}

	/**
	 * reduce angle into 0-360 deg range
	 * @param deg
	 * @return
	 */
	static float normalizeDeg(float deg){
		while(deg >= 360.0)
			deg -= 360.f;
		while(deg < 0.f)
			deg += 360.f;

		return deg;
	}

	/**
	 * convert radians to degrees
	 *
	 * @param rad
	 * @return
	 */
	static float radToDeg(float rad){
		return rad * 360.f/(2*M_PI);
	}

	float fi = 0;
	float fi_prev = 0;
	float omega = 0;
	float omega_prev = 0;

	float fi_mech = 0;
	float omega_mech = 0;

	float BEMF_U = 0;
	float BEMF_V = 0;
	float BEMF_W = 0;

	float V_U = 0;
	float V_V = 0;
	float V_W = 0;

	float I_U = 0;
	float I_V = 0;
	float I_W = 0;

	float I_PREV_U = 0;
	float I_PREV_V = 0;
	float I_PREV_W = 0;

	float T_U = 0;
	float T_V = 0;
	float T_W = 0;

	float T_Electromagnetic = 0;
	float T_Load = 0;

	static constexpr float PHASE_OFFSET_U = 0.f;
	static constexpr float PHASE_OFFSET_V = 2.f*M_PI/3;
	static constexpr float PHASE_OFFSET_W = 4.f*M_PI/3;

	struct VoltageVector{
		float U;
		float V;
		float W;
	};

	//voltages for different vectors
	static constexpr VoltageVector VOLTAGE_TABLE[] = {
			{0, 0, 0},																//000
			{-QUATER_BRIDGE_VOLTAGE, -QUATER_BRIDGE_VOLTAGE, HALF_BRIDGE_VOLTAGE},	//001
			{-QUATER_BRIDGE_VOLTAGE, HALF_BRIDGE_VOLTAGE, -QUATER_BRIDGE_VOLTAGE},	//010
			{-HALF_BRIDGE_VOLTAGE, QUATER_BRIDGE_VOLTAGE, QUATER_BRIDGE_VOLTAGE},	//011
			{HALF_BRIDGE_VOLTAGE, -QUATER_BRIDGE_VOLTAGE, -QUATER_BRIDGE_VOLTAGE},	//100
			{QUATER_BRIDGE_VOLTAGE, -HALF_BRIDGE_VOLTAGE, QUATER_BRIDGE_VOLTAGE},	//101
			{QUATER_BRIDGE_VOLTAGE, QUATER_BRIDGE_VOLTAGE, -HALF_BRIDGE_VOLTAGE},	//110
			{BRIDGE_VOLTAGE, BRIDGE_VOLTAGE, BRIDGE_VOLTAGE},						//111
	};
};



#endif /* MOTOR_HPP_ */
