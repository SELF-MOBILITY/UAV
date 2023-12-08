/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Code by Jon Challinger
//  Modified by Paul Riseborough
//

#include <AP_HAL/AP_HAL.h>
#include "AP_RollController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RollController::var_info[] = {
	// @Param: TCONST
	// @DisplayName: Roll Time Constant
	// @Description: Time constant in seconds from demanded to achieved roll angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.
	// @Range: 0.4 1.0
	// @Units: s
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("TCONST",      0, AP_RollController, gains.tau,       0.5f),

	// @Param: P
	// @DisplayName: Proportional Gain
	// @Description: Proportional gain from roll angle demands to ailerons. Higher values allow more servo response but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: Standard
	AP_GROUPINFO("P",        1, AP_RollController, gains.P,        1.0f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: Damping gain from roll acceleration to ailerons. Higher values reduce rolling in turbulence, but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 0.2
	// @Increment: 0.01
	// @User: Standard
	AP_GROUPINFO("D",        2, AP_RollController, gains.D,        0.08f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: Integrator gain from long-term roll angle offsets to ailerons. Higher values "trim" out offsets faster but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 1.0
	// @Increment: 0.05
	// @User: Standard
	AP_GROUPINFO("I",        3, AP_RollController, gains.I,        0.3f),

	// @Param: RMAX
	// @DisplayName: Maximum Roll Rate
	// @Description: Maximum roll rate that the roll controller demands (degrees/sec) in ACRO mode.
	// @Range: 0 180
	// @Units: deg/s
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("RMAX",   4, AP_RollController, gains.rmax,       0),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: Limit of roll integrator gain in centi-degrees of servo travel. Servos are assumed to have +/- 4500 centi-degrees of travel, so a value of 3000 allows trim of up to 2/3 of servo travel range.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",      5, AP_RollController, gains.imax,        3000),

	// @Param: FF
	// @DisplayName: Feed forward Gain
	// @Description: Gain from demanded rate to aileron output. 
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: Standard
	AP_GROUPINFO("FF",        6, AP_RollController, gains.FF,          0.0f),

    // @Param: SRMAX
    // @DisplayName: Servo slew rate limit
    // @Description: Sets an upper limit on the servo slew rate produced by the D-gain (roll rate feedback). If the amplitude of the control action produced by the roll rate feedback exceeds this value, then the D-gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive D-gain. The parameter should be set to no more than 25% of the servo's specified slew rate to allow for inertia and aerodynamic load effects. Note: The D-gain will not be reduced to less than 10% of the nominal value. A valule of zero will disable this feature.
    // @Units: deg/s
    // @Range: 0 500
    // @Increment: 10.0
    // @User: Advanced
    AP_GROUPINFO("SRMAX", 7, AP_RollController, _slew_rate_max, 150.0f),

    // @Param: SRTAU
    // @DisplayName: Servo slew rate decay time constant
    // @Description: This sets the time constant used to recover the D-gain after it has been reduced due to excessive servo slew rate.
    // @Units: s
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SRTAU", 8, AP_RollController, _slew_rate_tau, 1.0f),

    AP_GROUPEND
};



/* calculate sign function
 */
float AP_RollController::sign(float x)
{
	if (x > 0.0000001f) {
		return 1.0f;
	} else if (x < -0.0000001f) {
		return -1.0f;
	} else {
		return 0.0f;
	}
	return 0.0f;
}

/*
  internal rate controller, called by attitude and rate controller
  public functions
*/
int32_t AP_RollController::_get_rate_out(float desired_rate, float scaler, bool disable_integrator)
{
	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	
	// Calculate equivalent gains so that values for K_P and K_I can be taken across from the old PID law
    // No conversion is required for K_D
	float ki_rate = gains.I * gains.tau;
    float eas2tas = _ahrs.get_EAS2TAS();
	float kp_ff = MAX((gains.P - gains.I * gains.tau) * gains.tau  - gains.D , 0) / eas2tas;
    float k_ff = gains.FF / eas2tas;
	float delta_time    = (float)dt * 0.001f;
    // Get body rate vector (radians/sec)
	float omega_x = _ahrs.get_gyro().x;
	
	// Calculate the roll rate error (deg/sec) and apply gain scaler
    float achieved_rate = ToDeg(omega_x);
    _pid_info.error = desired_rate - achieved_rate;
    float rate_error = _pid_info.error * scaler;
    _pid_info.target = desired_rate;
    _pid_info.actual = achieved_rate;



	// calculate the cost
	static uint32_t num0= 0;
	if(0==num0)
	{
		num0++;
	}
	static uint32_t count = 0;
	static double sum      = 0;
	static double sum0     = 0;
	double  cost           = 0;
	double  cost0           = 0;
	uint32_t k = 0;
	static float error_collect[7000];
	if(num0 == 1)
	{
		error_collect[count] = desired_rate;
	}
	count++;
	//if (count > 130 && count <= 2800)
		if (tnow > 69000 && count <= 2800)
	{
		sum0 += (fabs(desired_rate) + 0.1*fabs(rate_error))/53.42;
	}
	if (count > 2800)//)2800                 2800: constant mass     3574: mass change
//	if(tnow > 68100)  //103000
	{
		sum += (fabs(desired_rate) + 0.1*fabs(rate_error))/0.51;
//		sum += (fabs(desired_rate) + 0.1*fabs(rate_error))   /      1.0934     ;// ; 0.471       24.91: constant mass     25.154: mass change
	}  
//	printf("11111111111    roll    count %d, time %d\n",count,AP_HAL::millis() );
	if(tnow > 158000 && num0==1)
	{
		cost0 = sum0 /(2800-130);
		cost = sum /(count - 2800);        // here should be constantce
//		cost += 0.5*cost0;
		//printf("11111111111    roll   count %d\n",count);
		printf("Roll: _asmc_alfa %f, _sat_eps %f, _eta %f, cost %f, cost0 %f\n", (double)_asmc_alfa, (double)_sat_eps, (double)_eta, cost, cost0);
		//printf("Roll: I %f, P %f, D %f, cost %f\n", (double)gains.I, (double)gains.P, (double)gains.D, cost);
		AP::vehicle()->update_pso_cost(cost, 3);
		num0++;
		printf("roll error_collect = [...] and total  number is %d\n",count);
		for(k=0;k<count;k++)
		{
			printf("%f ",error_collect[k]);
		}
		printf("\n");
	}
	uint32_t _switch =  0   ;  //  0  original,     1  adaptive      2  finite-time





	
	
	// Get an airspeed estimate - default to zero if none available
	float aspeed;
	if (!_ahrs.airspeed_estimate(aspeed)) {
        aspeed = 0.0f;
    }

	// Multiply roll rate error by _ki_rate, apply scaler and integrate
	// Scaler is applied before integrator so that integrator state relates directly to aileron deflection
	// This means aileron trim offset doesn't change as the value of scaler changes with airspeed
	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	if (!disable_integrator && ki_rate > 0) {
		//only integrate if gain and time step are positive and airspeed above min value.
		if (dt > 0 && aspeed > float(aparm.airspeed_min)) {
		    float integrator_delta = rate_error * ki_rate * delta_time * scaler;
			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -45) {
                integrator_delta = MAX(integrator_delta , 0);
            } else if (_last_out > 45) {
                // prevent the integrator from decreasing if surface defln demand  is below the lower limit
                 integrator_delta = MIN(integrator_delta, 0);
            }
			_pid_info.I += integrator_delta;
		}
	} else {
		_pid_info.I = 0;
	}
	
    // Scale the integration limit
    float intLimScaled = gains.imax * 0.01f;

    // Constrain the integrator state
    _pid_info.I = constrain_float(_pid_info.I, -intLimScaled, intLimScaled);
	
	// Calculate the demanded control surface deflection
	// Note the scaler is applied again. We want a 1/speed scaler applied to the feed-forward
	// path, but want a 1/speed^2 scaler applied to the rate error path. 
	// This is because acceleration scales with speed^2, but rate scales with speed.
    _pid_info.D = rate_error * gains.D * scaler;
    _pid_info.P = desired_rate * kp_ff * scaler;
    _pid_info.FF = desired_rate * k_ff * scaler;

    if (dt > 0 && _slew_rate_max > 0) {
        // Calculate the slew rate amplitude produced by the unmodified D term
        // calculate a low pass filtered slew rate
        float Dterm_slew_rate = _slew_rate_filter.apply((fabsf(_pid_info.D - _last_pid_info_D)/ delta_time), delta_time);

        // rectify and apply a decaying envelope filter
        float alpha = 1.0f - constrain_float(delta_time/_slew_rate_tau, 0.0f , 1.0f);
        _slew_rate_amplitude = fmaxf(fabsf(Dterm_slew_rate), alpha * _slew_rate_amplitude);
        _slew_rate_amplitude = fminf(_slew_rate_amplitude, 10.0f*_slew_rate_max);

        // Calculate and apply the D gain adjustment
        _pid_info.Dmod = _D_gain_modifier = _slew_rate_max / fmaxf(_slew_rate_amplitude, _slew_rate_max);
        _pid_info.D *= _D_gain_modifier;
    }

    _last_pid_info_D = _pid_info.D;

    _last_out = _pid_info.D + _pid_info.FF + _pid_info.P;
//	static float e0 = _pid_info.FF + _pid_info.P;
//	static float e0_dot = _pid_info.D;
//    _last_out = _last_out - e0 - e0_dot;

    if (autotune.running && aspeed > aparm.airspeed_min) {
        // let autotune have a go at the values 
        // Note that we don't pass the integrator component so we get
        // a better idea of how much the base PD controller
        // contributed
        autotune.update(desired_rate, achieved_rate, _last_out);
    }

	_last_out += _pid_info.I;

    // Add adaptive-robust rule term
	//float sumPid = (fabs(gains.D * scaler) < 1e-6) ? _last_out : (_last_out/(gains.D * scaler));
    //float errorInt = (fabs(ki_rate*scaler) < 1e-6) ? _pid_info.I : (_pid_info.I/(ki_rate*scaler));
	if (_switch == 1  )     //                  && tnow > 57320)
	{
	//	float PID =  constrain_float(_last_out, -45, 45);

		float adpative_robust_value = _update_roll_adaptive_robust_rule(_last_out,    _pid_info.P, _pid_info.D, _pid_info.I, delta_time);
		if (_last_out < -45)   //_last_out still will exceed limitation range
		{
			adpative_robust_value = MAX(adpative_robust_value , 0);
		}
		else if (_last_out > 45)
		{
			adpative_robust_value = MIN(adpative_robust_value, 0);
		} 
		_last_out += adpative_robust_value;
	}
	
	if (_switch == 2 ) 
	{
	//	float adpative_finite_time_value = _update_roll_finite_time_adaptive_rule(_last_out, PID*(_pid_info.FF + _pid_info.P)/_last_out, PID*(_pid_info.D)/_last_out,PID*(_pid_info.I)/_last_out, delta_time);
		float adpative_finite_time_value = _update_roll_finite_time_adaptive_rule(_last_out, _pid_info.P, _pid_info.D, _pid_info.I, delta_time);
		if (_last_out < -45)   //_last_out still will exceed limitation range
		{
			adpative_finite_time_value = MAX(adpative_finite_time_value , 0);
		}
		else if (_last_out > 45)
		{
			adpative_finite_time_value = MIN(adpative_finite_time_value, 0);
		}
		_last_out += adpative_finite_time_value; 
	}
	
	// Convert to centi-degrees and constrain
	return constrain_float(_last_out * 100, -4500, 4500);
}

/* calculate saturation value to avoid chattering
 */
float AP_RollController::saturation(float x)
{
	if (x > 1.0f) {
		return 1.0f;
	} else if (x < -1.0f) {
		return -1.0f;
	} else {
		return x;
	}
	return x;
}

/*
  Get roll finite-time adaptive rule term
 */
float AP_RollController::_update_roll_finite_time_adaptive_rule(float pid_sum, float error, float error_dot, float error_int, float delta_time)
{
	// Calculate the adaptive_robust_rule to better deal with the uncertainties.
	// tau = s + rho*sign(s); rho = K0 + K1*||xi|| + K2*||xi||^2; K_i_dot = ||s|| * ||xi||^i - alfa * K_i^v, i = 0,1,2;
	// xi = [error, error_dot, error_int];

	float error_delta = 0;
	float alpha1 = 0;
    float alpha2 = 0;
	if (fabs(error)>_varepsRoll)
	{
		error_delta = pow(fabs(error), _gammaRoll)*sign(error);
	}
	else
	{
		alpha1 = (2-_gammaRoll)* pow(_varepsRoll, _gammaRoll-1);
	    alpha2 = (_gammaRoll-1)* pow(_varepsRoll, _gammaRoll-2);
		error_delta = alpha1*error + alpha2*sign(error)*pow(error, 2);
	}

	
	float s       = pid_sum + _lambda3Roll*error_delta; // s = error +  error_dot +  error_int + _lambda3Roll*error_delta;
	float norm_xi = sqrt((fabs(error)+pow(fabs(error), _gammaRoll))*(fabs(error)+pow(fabs(error), _gammaRoll)) + error_dot*error_dot + error_int*error_int);
	float norm_s  = fabs(s);
	
	// Calculate sign(s), but avoid chattering
	//float throttle_damp = _thrDamp;
	//if (_flags.is_doing_auto_land && !is_zero(_land_throttle_damp)) {
	//	throttle_damp = _land_throttle_damp;
	//}
	//float sign_s = saturation(s * (throttle_damp / 1) * _sat_eps);
	float sat_s = saturation(s / _upsilonRoll);
 	float rho = _intK0Roll + _intK1Roll * norm_xi + _intK2Roll * norm_xi*norm_xi;
 //   float rho = 10;
//    float rho = _intK0Thr;


	if (delta_time > 0) {
		float intK0_delta = (norm_s - _betaRoll * pow(_intK0Roll,_vRoll)) * delta_time;
		float intK1_delta = (norm_s * norm_xi - _betaRoll * pow(_intK1Roll,_vRoll)) * delta_time;
		float intK2_delta = (norm_s * norm_xi*norm_xi - _betaRoll * pow(_intK2Roll,_vRoll)) * delta_time;
		// prevent the integrator from increasing if surface defln demand is above the upper limit
	/*	if (_last_out < -44) {
			intK0_delta = MAX(intK0_delta * sat_s, 0)/sat_s;  //kepp nagative     ruguo  lianggezhi xiangjia zhihou, meiyou fashengbaohe,  name jiubuyong decrease
			intK1_delta = MAX(intK1_delta , 0);
			intK2_delta = MAX(intK2_delta , 0);
		} else if (_last_out > 44) {
			// prevent the integrator from decreasing if surface defln demand  is below the lower limit
			intK0_delta = MIN(intK0_delta, 0);
			intK1_delta = MIN(intK1_delta, 0);  //negative
			intK2_delta = MIN(intK2_delta, 0);
		}
		*/
		// prevent the integrator from increasing if surface defln demand is above the upper limit
	if (_last_out < -45 || _last_out > 45) {
		float intK0_delta_temp = - _betaRoll * pow(_intK0Roll,_vRoll) * delta_time;  //keep nagative     ruguo  lianggezhi xiangjia zhihou, meiyou fashengbaohe,  name jiubuyong decrease
		float intK1_delta_temp = - _betaRoll * pow(_intK1Roll,_vRoll) * delta_time;
		float intK2_delta_temp = - _betaRoll * pow(_intK2Roll,_vRoll) * delta_time;
		float	_intK0Roll_temp = _intK0Roll;  _intK0Roll_temp += intK0_delta_temp;
		float _intK1Roll_temp = _intK1Roll;  _intK1Roll_temp  +=  intK1_delta_temp;
		float _intK2Roll_temp = _intK2Roll;  _intK2Roll_temp  +=  intK2_delta_temp;
		float rho_temp = _intK0Roll_temp + _intK1Roll_temp * norm_xi + _intK2Roll_temp * norm_xi*norm_xi;
		float _last_out_temp = _lambda3Roll*error_delta +  rho_temp * sat_s + _sigmaRoll*pow(fabs(s),_vRoll)*sign(s);   //  printf("roll 111111111111\n");
		if (_last_out_temp < -45 || _last_out_temp > 45)
		{
			intK0_delta  = intK0_delta_temp;
			intK1_delta  = intK1_delta_temp;
			intK2_delta  = intK2_delta_temp;
		}
	}
		_intK0Roll += intK0_delta;
		_intK1Roll += intK1_delta;
		_intK2Roll += intK2_delta;
		if (_intK0Roll < 0) {
		_intK0Roll = 0;
		}
		if (_intK1Roll < 0) {
		_intK1Roll = 0;
		}
		if (_intK2Roll < 0) {
		_intK2Roll = 0;
		}
	}

  /*  // Calculate rho
    _intK0Roll = _intK0Roll + (norm_s - _betaRoll * pow(_intK0Roll,_vRoll)) * delta_time;
    _intK1Roll = _intK1Roll + (norm_s * norm_xi - _betaRoll * pow(_intK1Roll,_vRoll)) * delta_time;
    _intK2Roll = _intK2Roll + (norm_s * norm_xi*norm_xi - _betaRoll * pow(_intK2Roll,_vRoll)) * delta_time;
	// integrator saturation
	if (_intK0Roll > _satRoll * gains.I * gains.tau) {
		_intK0Roll = _satRoll * gains.I * gains.tau;
	} else if (_intK0Roll < 0) {
		_intK0Roll = 0;
	}
	if (_intK1Roll > _satRoll * gains.I * gains.tau) {
		_intK1Roll = _satRoll * gains.I * gains.tau;
	} else if (_intK1Roll < 0) {
		_intK1Roll = 0;
	}
	if (_intK2Roll > _satRoll * gains.I * gains.tau) {
		_intK2Roll = _satRoll * gains.I * gains.tau;
	} else if (_intK2Roll < 0) {
		_intK2Roll = 0;
	}  */
	//float rho = _intK0Thr + _intK1Thr * norm_xi + _intK2Thr * norm_xi*norm_xi;

	return _lambda3Roll*error_delta + rho*sat_s + _sigmaRoll*pow(fabs(s),_vRoll)*sign(s);
}

/*
  Get roll adaptive robust rule term
 */
float AP_RollController::_update_roll_adaptive_robust_rule(float pid_sum, float error, float error_dot, float error_int, float delta_time)
{
	// Calculate the adaptive_robust_rule to better deal with the uncertainties.
	// tau = s + rho*sign(s); rho = K0 + K1*||xi|| + K2*||xi||^2; K_i_dot = ||s|| * ||xi||^i - alfa * K_i, i = 0,1,2;
	// xi = [error, error_dot, error_int];
	
	
	
	
	
	

	float s       = pid_sum;
	float norm_xi = sqrt(error*error + error_dot*error_dot +  error_int*error_int);
	float norm_s  = fabs(s);
	
	// Calculate sign(s), but avoid chattering
	//float eas2tas = _ahrs.get_EAS2TAS();
	//float k_ff = gains.FF / eas2tas;
	//float kp_ff = MAX((gains.P - gains.I * gains.tau) * gains.tau  - gains.D , 0) / eas2tas;
	//float sign_s = saturation(s * gains.D / (kp_ff+k_ff) * eas2tas * _sat_eps);
	float sign_s = saturation(s / _sat_eps);
	float rho = _intK0Roll + _intK1Roll * norm_xi + _intK2Roll * norm_xi*norm_xi;
//    float rho = 80;
//	float rho = _intK0Roll;

    // Calculate rho
	float intK0_delta = _eta*(norm_s - _asmc_alfa * _intK0Roll) * delta_time;
	float intK1_delta = _eta*(norm_s * norm_xi - _asmc_alfa * _intK1Roll) * delta_time;
	float intK2_delta = _eta*(norm_s * norm_xi*norm_xi - _asmc_alfa * _intK2Roll) * delta_time;
	// prevent the integrator from increasing if surface defln demand is above the upper limit
	if (_last_out < -45 || _last_out > 45) {
		float intK0_delta_temp = _eta*( - _asmc_alfa * _intK0Roll) * delta_time;  //keep nagative     ruguo  lianggezhi xiangjia zhihou, meiyou fashengbaohe,  name jiubuyong decrease
		float intK1_delta_temp = _eta*( - _asmc_alfa * _intK1Roll) * delta_time;
		float intK2_delta_temp = _eta*(  - _asmc_alfa * _intK2Roll) * delta_time;
		float	_intK0Roll_temp = _intK0Roll;  _intK0Roll_temp += intK0_delta_temp;
		float _intK1Roll_temp = _intK1Roll;  _intK1Roll_temp  +=  intK1_delta_temp;
		float _intK2Roll_temp = _intK2Roll;  _intK2Roll_temp  +=  intK2_delta_temp;
		float rho_temp = _intK0Roll_temp + _intK1Roll_temp * norm_xi + _intK2Roll_temp * norm_xi*norm_xi;
		float _last_out_temp = rho_temp * sign_s;   //  printf("roll 111111111111\n");
		if (_last_out_temp < -45 || _last_out_temp > 45)
		{
			intK0_delta  = intK0_delta_temp;
			intK1_delta  = intK1_delta_temp;
			intK2_delta  = intK2_delta_temp;
		}
	}
	
	_intK0Roll += intK0_delta;
	_intK1Roll += intK1_delta;
	_intK2Roll += intK2_delta;
	if (_intK0Roll < 0) {
	_intK0Roll = 0;
	}
	if (_intK1Roll < 0) {
	_intK1Roll = 0;
	}
	if (_intK2Roll < 0) {
	_intK2Roll = 0;
	}
	
	
	
	
	
/*    _intK0Roll = _intK0Roll + _eta*(norm_s - _asmc_alfa * _intK0Roll) * delta_time;
    _intK1Roll = _intK1Roll + _eta*(norm_s * norm_xi - _asmc_alfa * _intK1Roll) * delta_time;
    _intK2Roll = _intK2Roll + _eta*(norm_s * norm_xi*norm_xi - _asmc_alfa * _intK2Roll) * delta_time;
	// integrator saturation
	if (_intK0Roll > _satRoll * gains.I * gains.tau) {
		_intK0Roll = _satRoll * gains.I * gains.tau;
	} else if (_intK0Roll < 0) {
		_intK0Roll = 0;
	}
	if (_intK1Roll > _satRoll * gains.I * gains.tau) {
		_intK1Roll = _satRoll * gains.I * gains.tau;
	} else if (_intK1Roll < 0) {
		_intK1Roll = 0;
	}
	if (_intK2Roll > _satRoll * gains.I * gains.tau) {
		_intK2Roll = _satRoll * gains.I * gains.tau;
	} else if (_intK2Roll < 0) {
		_intK2Roll = 0;
	}
	//float rho = _intK0Roll + _intK1Roll * norm_xi + _intK2Roll * norm_xi*norm_xi;
	*/
	
	

	return rho*sign_s;
}


/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) desired roll rate in degrees/sec
 2) control gain scaler = scaling_speed / aspeed
*/
int32_t AP_RollController::get_rate_out(float desired_rate, float scaler)
{
    return _get_rate_out(desired_rate, scaler, false);
}

/*
 Function returns an equivalent aileron deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) demanded bank angle in centi-degrees
 2) control gain scaler = scaling_speed / aspeed
 3) boolean which is true when stabilise mode is active
 4) minimum FBW airspeed (metres/sec)
*/
int32_t AP_RollController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator)
{
    if (gains.tau < 0.1f) {
        gains.tau.set(0.1f);
    }
	
	// Calculate the desired roll rate (deg/sec) from the angle error
	float desired_rate = angle_err * 0.01f / gains.tau;
    _last_sumPid = angle_err;
	
    // Limit the demanded roll rate
    if (gains.rmax && desired_rate < -gains.rmax) {
        desired_rate = - gains.rmax;
    } else if (gains.rmax && desired_rate > gains.rmax) {
        desired_rate = gains.rmax;
    }

    return _get_rate_out(desired_rate, scaler, disable_integrator);
}

void AP_RollController::reset_I()
{
	_pid_info.I = 0;
}
