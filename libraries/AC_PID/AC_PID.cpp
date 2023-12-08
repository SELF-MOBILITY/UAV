/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PID.h"

const AP_Param::GroupInfo AC_PID::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P", 0, AC_PID, _kp, 0),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I", 1, AC_PID, _ki, 0),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("D", 2, AC_PID, _kd, 0),

    // 3 was for uint16 IMAX

    // @Param: FF
    // @DisplayName: FF FeedForward Gain
    // @Description: FF Gain which produces an output value that is proportional to the demanded input
    AP_GROUPINFO("FF", 4, AC_PID, _kff, 0),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 5, AC_PID, _kimax, 0),

    // 6 was for float FILT

    // 7 is for float ILMI and FF

    // index 8 was for AFF

    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTT", 9, AC_PID, _filt_T_hz, AC_PID_TFILT_HZ_DEFAULT),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTE", 10, AC_PID, _filt_E_hz, AC_PID_EFILT_HZ_DEFAULT),

    // @Param: FLTD
    // @DisplayName: PID Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTD", 11, AC_PID, _filt_D_hz, AC_PID_DFILT_HZ_DEFAULT),

    // @Param: SMAX
    // @DisplayName: Slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("SMAX", 12, AC_PID, _slew_rate_max, 0),

    AP_GROUPEND
};

// Constructor
AC_PID::AC_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz,
               float dt, float initial_srmax, float initial_srtau):
    _dt(dt),
    _integrator(0.0f),
    _error(0.0f),
    _derivative(0.0f)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp = initial_p;
    _ki = initial_i;
    _kd = initial_d;
    _kff = initial_ff;
    _kimax = fabsf(initial_imax);
    filt_T_hz(initial_filt_T_hz);
    filt_E_hz(initial_filt_E_hz);
    filt_D_hz(initial_filt_D_hz);
    _slew_rate_max.set(initial_srmax);
    _slew_rate_tau.set(initial_srtau);

    // reset input filter to first value received
    _flags._reset_filter = true;

    memset(&_pid_info, 0, sizeof(_pid_info));
}

// set_dt - set time step in seconds
void AC_PID::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
}

// filt_T_hz - set target filter hz
void AC_PID::filt_T_hz(float hz)
{
    _filt_T_hz.set(fabsf(hz));
}

// filt_E_hz - set error filter hz
void AC_PID::filt_E_hz(float hz)
{
    _filt_E_hz.set(fabsf(hz));
}

// filt_D_hz - set derivative filter hz
void AC_PID::filt_D_hz(float hz)
{
    _filt_D_hz.set(fabsf(hz));
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
float AC_PID::update_all(float target, float measurement, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _target += get_filt_T_alpha() * (target - _target);
        _error += get_filt_E_alpha() * ((_target - measurement) - _error);

        // calculate and filter derivative
        if (_dt > 0.0f) {
            float derivative = (_error - error_last) / _dt;
            _derivative += get_filt_D_alpha() * (derivative - _derivative);
        }
    }

    // update I term
    update_i(limit);

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);

    // calculate slew limit modifier for P+D
    _pid_info.Dmod = _slew_limiter.modifier(_pid_info.P + _pid_info.D, _dt);

    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;

    _pid_info.target = _target;
    _pid_info.actual = measurement;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;
//	printf("111111111111111111111111111111     _kp  %f,  _kd  %f,  _ki  %f \n", (double)_kp, (double)_kd, (double)_ki);
    return P_out + _integrator + D_out;
}



/* calculate saturation value to avoid chattering
 */
float AC_PID::saturation(float x)
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
  Get throttle adaptive robust rule term
 */
float AC_PID::_update_adaptive_robust_rule(float pid, float error, float error_dot, float error_int)
{
	// Calculate the adaptive_robust_rule to better deal with the uncertainties.
	// tau = s + rho*sign(s); rho = K0 + K1*||xi|| + K2*||xi||^2; K_i_dot = ||s|| * ||xi||^i - alfa * K_i, i = 0,1,2;
	// xi = [error, error_dot, error_int];
	

	float s = pid;
	float norm_xi = sqrt(error*error +  error_dot*error_dot +  error_int*error_int);
	float norm_s  = fabs(s);
	
	float sign_s = saturation(s / _eps);
 	float rho = _intK0 + _intK1 * norm_xi + _intK2 * norm_xi*norm_xi;
	float adaptive_term = 0;
//printf("_intK0 %f, _intK1%f, _intK2 %f \n" , _intK0, _intK1,  _intK2);
	if (is_positive(_dt))
	{
		if(!_shrink)
		{
			float intK0_delta =   (norm_s - _alpha * _intK0) * _dt;
			float intK1_delta =   (norm_s * norm_xi - _alpha * _intK1) * _dt;
			float intK2_delta =   (norm_s * norm_xi*norm_xi - _alpha * _intK2) * _dt;
			_intK0 += intK0_delta;
			_intK1 += intK1_delta;
			_intK2 += intK2_delta;
			
			
		}
	}
	else
	{
		_intK0 = 0;
		_intK1 = 0;
		_intK2 = 0;
	}
	
	if (_intK0 < 0) {
		_intK0 = 0;
	}
	if (_intK1 < 0) {
		_intK1 = 0;
	}
	if (_intK2 < 0) {
		_intK2 = 0;
	}

	adaptive_term = rho*sign_s;
		
//	printf("pid %f, e %f, e_dot %f,  e_int %f,   rho  %f, _intK0  %f,   _intK1 %f,  _intK2 %f,  sat_s %f,  xi %f,   adaptive_term %f\n" , \
//		(double)pid, (double)error, (double)error_dot, (double)error_int ,  rho ,  _intK0 , _intK1 ,  _intK2, sign_s, (double)norm_xi, (double)adaptive_term);

	return adaptive_term;
}

float AC_PID::get_adaptive_term(float scaler)
{
	float adaptive_term = 0;
	adaptive_term = _update_adaptive_robust_rule((_pid_info.P+_pid_info.I+_pid_info.D)*scaler, _pid_info.P*scaler, _pid_info.D*scaler, _pid_info.I*scaler);

	// calculate cost
	uint32_t tnow = AP_HAL::millis();
	if(tnow>47000)
	{
		sum += (fabs(_error) + 0.1*fabs(_derivative));
		count++;
		_cost = sum / count;
	//	printf("PID111111111111111111count %d, sum %f\n", count, sum);
	}

    return adaptive_term;
}

float AC_PID::get_cost()
{
	return _cost;
}

float AC_PID::get_error()
{
	return _error;
}


//  update_error - set error input to PID controller and calculate outputs
//  target is set to zero and error is set and filtered
//  the derivative then is calculated and filtered
//  the integral is then updated based on the setting of the limit flag
//  Target and Measured must be set manually for logging purposes.
// todo: remove function when it is no longer used.
float AC_PID::update_error(float error, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(error)) {
        return 0.0f;
    }

    _target = 0.0f;

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _error = error;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _error += get_filt_E_alpha() * (error - _error);

        // calculate and filter derivative
        if (_dt > 0.0f) {
            float derivative = (_error - error_last) / _dt;
            _derivative += get_filt_D_alpha() * (derivative - _derivative);
        }
    }

    // update I term
    update_i(limit);

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);

    // calculate slew limit modifier for P+D
    _pid_info.Dmod = _slew_limiter.modifier(_pid_info.P + _pid_info.D, _dt);

    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;
    
    _pid_info.target = 0.0f;
    _pid_info.actual = 0.0f;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;

    return P_out + _integrator + D_out;
}

//  update_i - update the integral
//  If the limit flag is set the integral is only allowed to shrink
void AC_PID::update_i(bool limit)
{
    if (!is_zero(_ki) && is_positive(_dt)) {
        // Ensure that integrator can only be reduced if the output is saturated
		_shrink = true;
        if (!limit || ((is_positive(_integrator) && is_negative(_error)) || (is_negative(_integrator) && is_positive(_error)))) {
            _integrator += ((float)_error * _ki) * _dt;
            _integrator = constrain_float(_integrator, -_kimax, _kimax);
			_shrink = false;
        }
    } else {
        _integrator = 0.0f;
    }
    _pid_info.I = _integrator;
}

float AC_PID::get_p() const
{
    return _error * _kp;
}

float AC_PID::get_i() const
{
    return _integrator;
}

float AC_PID::get_d() const
{
    return _kd * _derivative;
}

float AC_PID::get_ff()
{
    _pid_info.FF = _target * _kff;
    return _target * _kff;
}

void AC_PID::reset_I()
{
    _integrator = 0;
}

void AC_PID::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _kff.load();
    _kimax.load();
    _kimax = fabsf(_kimax);
    _filt_T_hz.load();
    _filt_E_hz.load();
    _filt_D_hz.load();
}

// save_gains - save gains to eeprom
void AC_PID::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _kff.save();
    _kimax.save();
    _filt_T_hz.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
}

/// Overload the function call operator to permit easy initialisation
void AC_PID::operator()(float p_val, float i_val, float d_val, float ff_val, float imax_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz, float dt)
{
    _kp = p_val;
    _ki = i_val;
    _kd = d_val;
    _kff = ff_val;
    _kimax = fabsf(imax_val);
    _filt_T_hz = input_filt_T_hz;
    _filt_E_hz = input_filt_E_hz;
    _filt_D_hz = input_filt_D_hz;
    _dt = dt;
}

// get_filt_T_alpha - get the target filter alpha
float AC_PID::get_filt_T_alpha() const
{
    return get_filt_alpha(_filt_T_hz);
}

// get_filt_E_alpha - get the error filter alpha
float AC_PID::get_filt_E_alpha() const
{
    return get_filt_alpha(_filt_E_hz);
}

// get_filt_D_alpha - get the derivative filter alpha
float AC_PID::get_filt_D_alpha() const
{
    return get_filt_alpha(_filt_D_hz);
}

// get_filt_alpha - calculate a filter alpha
float AC_PID::get_filt_alpha(float filt_hz) const
{
    return calc_lowpass_alpha_dt(_dt, filt_hz);
}

void AC_PID::set_integrator(float target, float measurement, float i)
{
    set_integrator(target - measurement, i);
}

void AC_PID::set_integrator(float error, float i)
{
    _integrator = constrain_float(i - error * _kp, -_kimax, _kimax);
    _pid_info.I = _integrator;
}

void AC_PID::set_integrator(float i)
{
    _integrator = constrain_float(i, -_kimax, _kimax);
    _pid_info.I = _integrator;
}
