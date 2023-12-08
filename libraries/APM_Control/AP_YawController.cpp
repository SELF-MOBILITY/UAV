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
//  Modified by Paul Riseborough to implement a three loop autopilot
//  topology
//
#include <AP_HAL/AP_HAL.h>
#include "AP_YawController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_YawController::var_info[] = {

	// @Param: SLIP
	// @DisplayName: Sideslip control gain
	// @Description: Gain from lateral acceleration to demanded yaw rate for aircraft with enough fuselage area to detect lateral acceleration and sideslips. Do not enable for flying wings and gliders. Actively coordinates flight more than just yaw damping. Set after YAW2SRV_DAMP and YAW2SRV_INT are tuned.
	// @Range: 0 4
	// @Increment: 0.25
    // @User: Advanced
	AP_GROUPINFO("SLIP",    0, AP_YawController, _K_A,    0),

	// @Param: INT
	// @DisplayName: Sideslip control integrator
	// @Description: Integral gain from lateral acceleration error. Effectively trims rudder to eliminate long-term sideslip.
	// @Range: 0 2
	// @Increment: 0.25
    // @User: Advanced
	AP_GROUPINFO("INT",    1, AP_YawController, _K_I,    0),

	// @Param: DAMP
	// @DisplayName: Yaw damping
	// @Description: Gain from yaw rate to rudder. Most effective at yaw damping and should be tuned after KFF_RDDRMIX. Also disables YAW2SRV_INT if set to 0.
	// @Range: 0 2
	// @Increment: 0.25
    // @User: Advanced
//	AP_GROUPINFO("DAMP",   2, AP_YawController, _K_D,    0),
	AP_GROUPINFO("DAMP",   2, AP_YawController, _K_D,    0.01),

	// @Param: RLL
	// @DisplayName: Yaw coordination gain
	// @Description: Gain to the yaw rate required to keep it consistent with the turn rate in a coordinated turn. Corrects for yaw tendencies after the turn is established. Increase yaw into the turn by raising. Increase yaw out of the turn by decreasing. Values outside of 0.9-1.1 range indicate airspeed calibration problems.
	// @Range: 0.8 1.2
	// @Increment: 0.05
    // @User: Advanced
	AP_GROUPINFO("RLL",   3, AP_YawController, _K_FF,   1),

    /*
      Note: index 4 should not be used - it was used for an incorrect
      AP_Int8 version of the IMAX in the 2.74 release
     */


	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: Limit of yaw integrator gain in centi-degrees of servo travel. Servos are assumed to have +/- 4500 centi-degrees of travel, so a value of 1500 allows trim of up to 1/3 of servo travel range.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",  5, AP_YawController, _imax,        1500),

	AP_GROUPEND
};




/* calculate sign function
 */
float AP_YawController::sign(float x)
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

int32_t AP_YawController::get_servo_out(float scaler, bool disable_integrator)
{
	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	

    int16_t aspd_min = aparm.airspeed_min;
    if (aspd_min < 1) {
        aspd_min = 1;
    }
	
	float delta_time = (float) dt / 1000.0f;
	
	// Calculate yaw rate required to keep up with a constant height coordinated turn
	float aspeed;
	float rate_offset;
	float bank_angle = _ahrs.roll;
	// limit bank angle between +- 80 deg if right way up
	if (fabsf(bank_angle) < 1.5707964f)	{
	    bank_angle = constrain_float(bank_angle,-1.3962634f,1.3962634f);
	}
	if (!_ahrs.airspeed_estimate(aspeed)) {
	    // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aspd_min) + float(aparm.airspeed_max));
	}
    rate_offset = (GRAVITY_MSS / MAX(aspeed , float(aspd_min))) * sinf(bank_angle) * _K_FF;

    // Get body rate vector (radians/sec)
	float omega_z = _ahrs.get_gyro().z;
	
	// Get the accln vector (m/s^2)
	float accel_y = AP::ins().get_accel().y;

	// Subtract the steady turn component of rate from the measured rate
	// to calculate the rate relative to the turn requirement in degrees/sec
	float rate_hp_in = ToDeg(omega_z - rate_offset);
	
	// Apply a high-pass filter to the rate to washout any steady state error
	// due to bias errors in rate_offset
	// Use a cut-off frequency of omega = 0.2 rad/sec
	// Could make this adjustable by replacing 0.9960080 with (1 - omega * dt)
	float rate_hp_out = 0.9960080f * _last_rate_hp_out + rate_hp_in - _last_rate_hp_in;
	_last_rate_hp_out = rate_hp_out;
	_last_rate_hp_in = rate_hp_in;

	//Calculate input to integrator
	float integ_in = - _K_I * (_K_A * accel_y + rate_hp_out);


	// calculate the cost
	static uint32_t count = 0;
	static uint32_t num   = 0, num0 = 0;
	static double sum0      = 0;
	static double sum      = 0;
	double  cost           = 0;
	double  cost0           = 0;
	uint32_t k = 0;
	static float error_collect[7000];
	if(num0 == 0)
	{
		error_collect[count] = rate_hp_out;
	}
	count++;
	//sum += (fabs(rate_hp_out) + 0.1*fabs(_K_A * accel_y))/0.557;
	if (count > 130 && count <= 2800)
	{
		sum0 += (fabs(rate_hp_out) + 0.1*fabs(accel_y))  / 1;
	}
	if (count >  2800)//2800)              2800: constant mass     3574: mass change
//		if(tnow > 68100)
	{
		sum += (fabs(rate_hp_out)  )/0.048;     //0.44;//  /0.657
//		sum += (fabs(rate_hp_out) )    /    0.3506        ;//  ; 0.171    0.199: constant mass     0.195: mass change
	}
//	printf("11111111111    roll  sum  %f,  count %d, time %d\n",sum,count,AP_HAL::millis() );
	if(tnow > 158000 && num0 == 0)
	{
		cost0 = sum0 /(2800-130);
		cost = sum /(count  -  2800);
	//	cost += 0.5*cost0;
		printf("Yaw: _asmc_alfa %f, _sat_eps %f, _eta %f, cost %f, cost0 %f\n", (double)_asmc_alfa, (double)_sat_eps, (double)_eta, cost, cost0);
		AP::vehicle()->update_pso_cost(cost, 5);
		//printf("Yaw: _K_A %f, _K_I %f, _K_D %f, _K_FF %f, cost %f\n", (double)_K_A, (double)_K_I, (double)_K_D, (double)_K_FF, cost);
		num0++;
		printf("yaw error_collect = [...] and total  number is %d\n",count);
		for(k=0;k<count;k++)
		{
			printf("%f ",error_collect[k]);
		}
		printf("\n");
	}
	if(0 == num%10)
	{
	    num = 0;
		//we will start to calculate the optimization cost after 60 seconds
	}
	num++;
	uint32_t _switch =  0  ;  //  0  original,     1  adaptive      2  finite-time


	
	
	// Apply integrator, but clamp input to prevent control saturation and freeze integrator below min FBW speed
	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	// Don't integrate if _K_D is zero as integrator will keep winding up
	if (!disable_integrator && _K_D > 0) {
		//only integrate if airspeed above min value
		if (aspeed > float(aspd_min))
		{
			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -45) {
                _integrator += MAX(integ_in * delta_time , 0);
            } else if (_last_out > 45) {
                // prevent the integrator from decreasing if surface defln demand  is below the lower limit
                _integrator += MIN(integ_in * delta_time , 0);
			} else {
                _integrator += integ_in * delta_time;
            }
		}
	} else {
		_integrator = 0;
	}

    if (_K_D < 0.0001f) {
        // yaw damping is disabled, and the integrator is scaled by damping, so return 0
        return 0;
    }
	
    // Scale the integration limit
    float intLimScaled = _imax * 0.01f / (_K_D * scaler * scaler);

    // Constrain the integrator state
    _integrator = constrain_float(_integrator, -intLimScaled, intLimScaled);
	
	// Protect against increases to _K_D during in-flight tuning from creating large control transients
	// due to stored integrator values
	if (_K_D > _K_D_last && _K_D > 0) {
	    _integrator = _K_D_last/_K_D * _integrator;
	}
	_K_D_last = _K_D;
	
	// Calculate demanded rudder deflection, +Ve deflection yaws nose right
	// Save to last value before application of limiter so that integrator limiting
	// can detect exceedance next frame
	// Scale using inverse dynamic pressure (1/V^2)
	_pid_info.I = _K_D * _integrator * scaler * scaler; 
	_pid_info.D = _K_D * (-rate_hp_out) * scaler * scaler;
//	static float e0_dot = _pid_info.D;
//	_pid_info.D -= e0_dot;
	_last_out =  _pid_info.I + _pid_info.D;

    // Add adaptive-robust rule term
	if (_switch == 1 )
	{
	//	float PID =  constrain_float(_last_out, -45, 45);
		float adpative_robust_value = _update_yaw_adaptive_robust_rule(_last_out, _pid_info.D, _pid_info.I, delta_time);
		if (_last_out < -45)
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
	//	float adpative_finite_time_value = _update_yaw_finite_time_adaptive_rule(PID, PID*(_pid_info.D)/_last_out, PID*(_pid_info.I)/_last_out, delta_time);
		float adpative_finite_time_value = _update_yaw_finite_time_adaptive_rule(_last_out, _pid_info.D, _pid_info.I, delta_time);
		if (_last_out < -45)
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
float AP_YawController::saturation(float x)
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
  Get pitch finite-time adaptive rule term
 */
float AP_YawController::_update_yaw_finite_time_adaptive_rule(float pid_sum, float error_dot, float error_int, float delta_time)
{
	// Calculate the adaptive_robust_rule to better deal with the uncertainties.
	// tau = s + rho*sign(s); rho = K0 + K1*||xi|| + K2*||xi||^2; K_i_dot = ||s|| * ||xi||^i - alfa * K_i^v, i = 0,1,2;
	// xi = [error, error_dot, error_int];

	float error_delta = 0;
/*	float alpha1 = 0;
    float alpha2 = 0;
	if (fabs(error)>_varepsYaw)
	{
		error_delta = pow(fabs(error), _gammaYaw)*sign(error);
	}
	else
	{
		alpha1 = (2-_gammaYaw)* pow(_varepsYaw, _gammaYaw-1);
	    alpha2 = (_gammaYaw-1)* pow(_varepsYaw, _gammaYaw-2);
		error_delta = alpha1*error + alpha2*sign(error)*pow(error, 2);
	}  */

	
	float s       = pid_sum + _lambda3Yaw*error_delta; // s = error +  error_dot +  error_int + _lambda3Yaw*error_delta;
	float norm_xi = sqrt( error_dot*error_dot + error_int*error_int);
	float norm_s  = fabs(s);
	
	// Calculate sign(s), but avoid chattering
	//float throttle_damp = _thrDamp;
	//if (_flags.is_doing_auto_land && !is_zero(_land_throttle_damp)) {
	//	throttle_damp = _land_throttle_damp;
	//}
	//float sign_s = saturation(s * (throttle_damp / 1) * _sat_eps);
	float sat_s = saturation(s / _upsilonYaw);
 	float rho = _intK0Yaw + _intK1Yaw * norm_xi + _intK2Yaw * norm_xi*norm_xi;
 //   float rho = 10;
//    float rho = _intK0Thr;

	if (delta_time > 0) {
		float intK0_delta = (norm_s - _betaYaw * pow(_intK0Yaw,_vYaw)) * delta_time;
		float intK1_delta = (norm_s * norm_xi - _betaYaw * pow(_intK1Yaw,_vYaw)) * delta_time;
		float intK2_delta = (norm_s * norm_xi*norm_xi - _betaYaw * pow(_intK2Yaw,_vYaw)) * delta_time;
		// prevent the integrator from increasing if surface defln demand is above the upper limit
	/*	if (_last_out < -44) {
			intK0_delta = MAX(intK0_delta , 0);
			intK1_delta = MAX(intK1_delta , 0);
			intK2_delta = MAX(intK2_delta , 0);
		} else if (_last_out > 44) {
			// prevent the integrator from decreasing if surface defln demand  is below the lower limit
			intK0_delta = MIN(intK0_delta, 0);
			intK1_delta = MIN(intK1_delta, 0);
			intK2_delta = MIN(intK2_delta, 0);
		}
		*/
		if (_last_out < -45 || _last_out > 45) {
		float intK0_delta_temp = - _betaYaw * pow(_intK0Yaw,_vYaw) * delta_time;  //keep nagative     ruguo  lianggezhi xiangjia zhihou, meiyou fashengbaohe,  name jiubuyong decrease
		float intK1_delta_temp = - _betaYaw * pow(_intK1Yaw,_vYaw) * delta_time;
		float intK2_delta_temp = - _betaYaw * pow(_intK2Yaw,_vYaw) * delta_time;
		float _intK0Yaw_temp = _intK0Yaw; _intK0Yaw_temp += intK0_delta_temp;
		float _intK1Yaw_temp = _intK1Yaw; _intK1Yaw_temp += intK1_delta_temp;
		float _intK2Yaw_temp = _intK2Yaw; _intK2Yaw_temp += intK2_delta_temp;
		float rho_temp = _intK0Yaw_temp + _intK1Yaw_temp * norm_xi + _intK2Yaw_temp * norm_xi*norm_xi;
		float _last_out_temp = _lambda3Yaw*error_delta + rho_temp * sat_s + _sigmaYaw*pow(fabs(s),_vYaw)*sign(s);   // printf("yaw 111111111111\n");
		if (_last_out_temp < -45 || _last_out_temp > 45)
		{
			intK0_delta  = intK0_delta_temp;
			intK1_delta  = intK1_delta_temp;
			intK2_delta  = intK2_delta_temp;
		}
	}  
		_intK0Yaw += intK0_delta;
		_intK1Yaw += intK1_delta;
		_intK2Yaw += intK2_delta;
		if (_intK0Yaw < 0) {
			_intK0Yaw = 0;
		}
		if (_intK1Yaw < 0) {
			_intK1Yaw = 0;
		}
		if (_intK2Yaw < 0) {
			_intK2Yaw = 0;
		}
	}

 /*   // Calculate rho
    _intK0Yaw = _intK0Yaw + (norm_s - _betaYaw * pow(_intK0Yaw,_vYaw)) * delta_time;
    _intK1Yaw = _intK1Yaw + (norm_s * norm_xi - _betaYaw * pow(_intK1Yaw,_vYaw)) * delta_time;
    _intK2Yaw = _intK2Yaw + (norm_s * norm_xi*norm_xi - _betaYaw * pow(_intK2Yaw,_vYaw)) * delta_time;
	// integrator saturation
	if (_intK0Yaw > _satYaw * _K_I) {
		_intK0Yaw = _satYaw * _K_I;
	} else if (_intK0Yaw < 0) {
		_intK0Yaw = 0;
	}
	if (_intK1Yaw > _satYaw *_K_I) {
		_intK1Yaw = _satYaw * _K_I;
	} else if (_intK1Yaw < 0) {
		_intK1Yaw = 0;
	}
	if (_intK2Yaw > _satYaw * _K_I) {
		_intK2Yaw = _satYaw * _K_I;
	} else if (_intK2Yaw < 0) {
		_intK2Yaw = 0;
	}     */
	//float rho = _intK0Thr + _intK1Thr * norm_xi + _intK2Thr * norm_xi*norm_xi;

	return _lambda3Yaw*error_delta + rho*sat_s + _sigmaYaw*pow(fabs(s),_vYaw)*sign(s);
}



/*
  Get pitch adaptive robust rule term
 */
float AP_YawController::_update_yaw_adaptive_robust_rule(float pid_sum, float error_dot, float error_int, float delta_time)
{
	// Calculate the adaptive_robust_rule to better deal with the uncertainties.
	// tau = s + rho*sign(s); rho = K0 + K1*||xi|| + K2*||xi||^2; K_i_dot = ||s|| * ||xi||^i - alfa * K_i, i = 0,1,2;
	// xi = [error, error_dot, error_int];
	
	
	
	
	

	float s       = pid_sum;
	float norm_xi = sqrt(error_dot*error_dot + error_int*error_int);
	float norm_s  = fabs(s);
	
	// Calculate sign(s), but avoid chattering
	//float sign_s = saturation(s * _K_D / _K_I * _sat_eps);
	float sign_s = saturation(s / _sat_eps);
	float rho = _intK0Yaw + _intK1Yaw * norm_xi + _intK2Yaw * norm_xi*norm_xi;
  //  float rho = 10;
//    float rho = _intK0Yaw;


    // Calculate rho
	float intK0_delta = _eta*(norm_s - _asmc_alfa * _intK0Yaw) * delta_time;
	float intK1_delta = _eta*(norm_s * norm_xi - _asmc_alfa * _intK1Yaw) * delta_time;
	float intK2_delta = _eta*(norm_s * norm_xi*norm_xi - _asmc_alfa * _intK2Yaw) * delta_time;
	if (_last_out < -45 || _last_out > 45) {
		float intK0_delta_temp = _eta*( - _asmc_alfa * _intK0Yaw) * delta_time;  //keep nagative     ruguo  lianggezhi xiangjia zhihou, meiyou fashengbaohe,  name jiubuyong decrease
		float intK1_delta_temp = _eta*( - _asmc_alfa * _intK1Yaw) * delta_time;
		float intK2_delta_temp = _eta*( - _asmc_alfa * _intK2Yaw) * delta_time;
		float _intK0Yaw_temp = _intK0Yaw; _intK0Yaw_temp += intK0_delta_temp;
		float _intK1Yaw_temp = _intK1Yaw; _intK1Yaw_temp += intK1_delta_temp;
		float _intK2Yaw_temp = _intK2Yaw; _intK2Yaw_temp += intK2_delta_temp;
		float rho_temp = _intK0Yaw_temp + _intK1Yaw_temp * norm_xi + _intK2Yaw_temp * norm_xi*norm_xi;
		float _last_out_temp = rho_temp * sign_s;   // printf("yaw 111111111111\n");
		if (_last_out_temp < -45 || _last_out_temp > 45)
		{
			intK0_delta  = intK0_delta_temp;
			intK1_delta  = intK1_delta_temp;
			intK2_delta  = intK2_delta_temp;
		}
	}  
	_intK0Yaw += intK0_delta;
	_intK1Yaw += intK1_delta;
	_intK2Yaw += intK2_delta;
	if (_intK0Yaw < 0) {
		_intK0Yaw = 0;
	}
	if (_intK1Yaw < 0) {
		_intK1Yaw = 0;
	}
	if (_intK2Yaw < 0) {
		_intK2Yaw = 0;
	}
	
	
	
	
	
	/*
    _intK0Yaw = _intK0Yaw + _eta*(norm_s - _asmc_alfa * _intK0Yaw) * delta_time;
    _intK1Yaw = _intK1Yaw + _eta*(norm_s * norm_xi - _asmc_alfa * _intK1Yaw) * delta_time;
    _intK2Yaw = _intK2Yaw + _eta*(norm_s * norm_xi*norm_xi - _asmc_alfa * _intK2Yaw) * delta_time;
	// integrator saturation
	if (_intK0Yaw > _satYaw * _K_I) {
		_intK0Yaw = _satYaw * _K_I;
	} else if (_intK0Yaw < 0) {
		_intK0Yaw = 0;
	}
	if (_intK1Yaw > _satYaw * _K_I) {
		_intK1Yaw = _satYaw * _K_I;
	} else if (_intK1Yaw < 0) {
		_intK1Yaw = 0;
	}
	if (_intK2Yaw > _satYaw * _K_I) {
		_intK2Yaw = _satYaw * _K_I;
	} else if (_intK2Yaw < 0) {
		_intK2Yaw = 0;
	}
	//float rho = _intK0Yaw + _intK1Yaw * norm_xi + _intK2Yaw * norm_xi*norm_xi;
*/

	return rho*sign_s;
}

void AP_YawController::reset_I()
{
	_integrator = 0;
}