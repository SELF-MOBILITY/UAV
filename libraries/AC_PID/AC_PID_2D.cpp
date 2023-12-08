/// @file	AC_PID_2D.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PID_2D.h"

#define AC_PID_2D_FILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_PID_2D_FILT_HZ_MIN      0.01f   // minimum input filter frequency
#define AC_PID_2D_FILT_D_HZ_DEFAULT  10.0f   // default input filter frequency
#define AC_PID_2D_FILT_D_HZ_MIN      0.005f   // minimum input filter frequency

const AP_Param::GroupInfo AC_PID_2D::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_PID_2D, _kp, 0),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I",    1, AC_PID_2D, _ki, 0),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 2, AC_PID_2D, _imax, 0),

    // @Param: FILT
    // @DisplayName: PID Input filter frequency in Hz
    // @Description: Input filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FILT", 3, AC_PID_2D, _filt_hz, AC_PID_2D_FILT_HZ_DEFAULT),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("D",    4, AC_PID_2D, _kd, 0),

    // @Param: D_FILT
    // @DisplayName: D term filter frequency in Hz
    // @Description: D term filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("D_FILT", 5, AC_PID_2D, _filt_d_hz, AC_PID_2D_FILT_D_HZ_DEFAULT),

    AP_GROUPEND
};

// Constructor
AC_PID_2D::AC_PID_2D(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float initial_filt_d_hz, float dt) :
    _dt(dt)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp = initial_p;
    _ki = initial_i;
    _kd = initial_d;
    _imax = fabsf(initial_imax);
    filt_hz(initial_filt_hz);
    filt_d_hz(initial_filt_d_hz);

    // reset input filter to first value received and derivitive to zero
    reset_filter();
}

// set_dt - set time step in seconds
void AC_PID_2D::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
    calc_filt_alpha();
    calc_filt_alpha_d();
}

// filt_hz - set input filter hz
void AC_PID_2D::filt_hz(float hz)
{
    _filt_hz.set(fabsf(hz));

    // sanity check _filt_hz
    _filt_hz = MAX(_filt_hz, AC_PID_2D_FILT_HZ_MIN);

    // calculate the input filter alpha
    calc_filt_alpha();
}

// filt_d_hz - set input filter hz
void AC_PID_2D::filt_d_hz(float hz)
{
    _filt_d_hz.set(fabsf(hz));

    // sanity check _filt_hz
    _filt_d_hz = MAX(_filt_d_hz, AC_PID_2D_FILT_D_HZ_MIN);

    // calculate the input filter alpha
    calc_filt_alpha_d();
}

// set_input - set input to PID controller
//  input is filtered before the PID controllers are run
//  this should be called before any other calls to get_p, get_i or get_d
void AC_PID_2D::set_input(const Vector2f &input)
{
    // don't process inf or NaN
    if (!isfinite(input.x) || !isfinite(input.y)) {
        return;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _input = input;
    }

    // update filter and calculate derivative
    const Vector2f input_delta = (input - _input) * _filt_alpha;
    _input += input_delta;

    set_input_filter_d(input_delta);
}

// set_input_filter_d - set input to PID controller
//  only input to the D portion of the controller is filtered
//  this should be called before any other calls to get_p, get_i or get_d
void AC_PID_2D::set_input_filter_d(const Vector2f& input_delta)
{
    // don't process inf or NaN
    if (!isfinite(input_delta.x) && !isfinite(input_delta.y)) {
        return;
    }

    // update filter and calculate derivative
    if (is_positive(_dt)) {
        const Vector2f derivative = input_delta / _dt;
        const Vector2f delta_derivative = (derivative - _derivative) * _filt_alpha_d;
        _derivative += delta_derivative;
    }
}

Vector2f AC_PID_2D::get_p() const
{
    return (_input * _kp);
}

Vector2f AC_PID_2D::get_i()
{
    if (!is_zero(_ki) && !is_zero(_dt)) {
        _integrator += (_input * _ki) * _dt;
        const float integrator_length = _integrator.length();
		_integrator_scaler = 1.0;
        if ((integrator_length > _imax) && is_positive(integrator_length)) {
            _integrator *= (_imax / integrator_length);
			_integrator_scaler = (_imax / integrator_length);
        }
        return _integrator;
    }
    return Vector2f();
}

// get_i_shrink - get_i but do not allow integrator to grow in length (it may shrink)
Vector2f AC_PID_2D::get_i_shrink()
{
    if (!is_zero(_ki) && !is_zero(_dt)) {
        const float integrator_length_orig = MIN(_integrator.length(), _imax);
        _integrator += (_input * _ki) * _dt;
		_integrator_scaler = 1.0;
        const float integrator_length_new = _integrator.length();
        if ((integrator_length_new > integrator_length_orig) && is_positive(integrator_length_new)) {
            _integrator *= (integrator_length_orig / integrator_length_new);
			_integrator_scaler =(integrator_length_orig / integrator_length_new);
        }
        return _integrator;
    }
    return Vector2f();
}

Vector2f AC_PID_2D::get_d()
{
    // derivative component
    return Vector2f(_kd * _derivative.x, _kd * _derivative.y);
}




/* calculate saturation value to avoid chattering
 */
float AC_PID_2D::saturation(float x)
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
float AC_PID_2D::_update_adaptive_robust_rule_x(float pid, float error, float error_dot, float error_int)
{
	// Calculate the adaptive_robust_rule to better deal with the uncertainties.
	// tau = s + rho*sign(s); rho = K0 + K1*||xi|| + K2*||xi||^2; K_i_dot = ||s|| * ||xi||^i - alfa * K_i, i = 0,1,2;
	// xi = [error, error_dot, error_int];
	

	float s = pid;
	float norm_xi = sqrt(error*error +  error_dot*error_dot +  error_int*error_int);
	float norm_s  = fabs(s);
	
	float sign_s = saturation(s / _eps);
 	float rho = _intK0_x + _intK1_x * norm_xi + _intK2_x * norm_xi*norm_xi;
	float adaptive_term = 0;
//printf("_intK0_x %f, _intK1_x  %f, _intK2_x  %f,  _integrator_scaler %f \n" , _intK0_x, _intK1_x,  _intK2_x, _integrator_scaler);
	if (!is_zero(_dt))
	{
		float intK0_delta =  (norm_s - _alpha * _intK0_x) * _dt;
		float intK1_delta =  (norm_s * norm_xi - _alpha * _intK1_x) * _dt;
		float intK2_delta =  (norm_s * norm_xi*norm_xi - _alpha * _intK2_x) * _dt;
		_intK0_x += intK0_delta;  
		_intK1_x += intK1_delta;
		_intK2_x += intK2_delta;
		
		_intK0_x *= _integrator_scaler;
		_intK1_x *= _integrator_scaler;
		_intK2_x *= _integrator_scaler;
		
	}
	
	if (_intK0_x < 0) {
		_intK0_x = 0;
	}
	if (_intK1_x < 0) {
		_intK1_x = 0;
	}
	if (_intK2_x < 0) {
		_intK2_x = 0;
	}

	adaptive_term = rho*sign_s;
		
//	printf("pid %f, e %f, e_dot %f,  e_int %f,   rho  %f, _intK0_x  %f,   _intK1_x %f,  _intK2_x %f,  sat_s %f,  xi %f,   adaptive_term %f\n" , \
//		(double)pid, (double)error, (double)error_dot, (double)error_int ,  rho ,  _intK0_x , _intK1_x ,  _intK2_x, sign_s, (double)norm_xi, (double)adaptive_term);

	return adaptive_term;
}

float AC_PID_2D::_update_adaptive_robust_rule_y(float pid, float error, float error_dot, float error_int)
{
	// Calculate the adaptive_robust_rule to better deal with the uncertainties.
	// tau = s + rho*sign(s); rho = K0 + K1*||xi|| + K2*||xi||^2; K_i_dot = ||s|| * ||xi||^i - alfa * K_i, i = 0,1,2;
	// xi = [error, error_dot, error_int];
	

	float s = pid;
	float norm_xi = sqrt(error*error +  error_dot*error_dot +  error_int*error_int);
	float norm_s  = fabs(s);
	
	float sign_s = saturation(s / _eps);
 	float rho = _intK0_y + _intK1_y * norm_xi + _intK2_y * norm_xi*norm_xi;
	float adaptive_term = 0;
//printf("_intK0_y %f, _intK1_y  %f, _intK2_y  %f,  _integrator_scaler %f \n" , _intK0_y, _intK1_y,  _intK2_y, _integrator_scaler);
	if (!is_zero(_dt))
	{
		float intK0_delta =  (norm_s - _alpha * _intK0_y) * _dt;
		float intK1_delta =  (norm_s * norm_xi - _alpha * _intK1_y) * _dt;
		float intK2_delta =  (norm_s * norm_xi*norm_xi - _alpha * _intK2_y) * _dt;
		_intK0_y += intK0_delta;  
		_intK1_y += intK1_delta;
		_intK2_y += intK2_delta;
//		printf("y      intK0_delta %f, intK1_delta %f, intK2_delta %f\n", intK0_delta, intK1_delta, intK2_delta);
		_intK0_y *= _integrator_scaler;
		_intK1_y *= _integrator_scaler;
		_intK2_y *= _integrator_scaler;
		
	}
	
	if (_intK0_y < 0) {
		_intK0_y = 0;
	}
	if (_intK1_y < 0) {
		_intK1_y = 0;
	}
	if (_intK2_y < 0) {
		_intK2_y = 0;
	}

	adaptive_term = rho*sign_s;
		
//	printf("pid %f, e %f, e_dot %f,  e_int %f,   rho  %f, _intK0_y  %f,   _intK1_y %f,  _intK2_y %f,  sat_s %f,  xi %f,   adaptive_term %f\n" , \
//		(double)pid, (double)error, (double)error_dot, (double)error_int ,  rho ,  _intK0_y , _intK1_y ,  _intK2_y, sign_s, (double)norm_xi, (double)adaptive_term);

	return adaptive_term;
}


Vector2f AC_PID_2D::get_adaptive_term(float scaler)
{
	float x = 0;
	float y = 0;
	x = _update_adaptive_robust_rule_x(get_pid().x*scaler, _kp * _input.x * scaler, _kd * _derivative.x * scaler, _integrator.x * scaler);
	y = _update_adaptive_robust_rule_y(get_pid().y*scaler, _kp * _input.y * scaler, _kd * _derivative.y * scaler, _integrator.y * scaler);
//		printf("PID222222222222222222222222\n");
	// calculate cost
	uint32_t tnow = AP_HAL::millis();
	if(tnow>47000)
	{
		sum += (fabs(_input.x) + fabs(_input.y) + 0.1*fabs(_derivative.x) + 0.1*fabs(_derivative.y));
		count++;
		_cost = sum / count;
	}
	
	return Vector2f(x,y);
}

float AC_PID_2D::get_cost()
{
	return _cost;
}



Vector2f AC_PID_2D::get_pid()
{
    return get_p() + get_i() + get_d();
}

void AC_PID_2D::reset_I()
{
    _integrator.zero();
}

void AC_PID_2D::reset_filter()
{
    _flags._reset_filter = true;
    _derivative.x = 0.0f;
    _derivative.y = 0.0f;
    _integrator.zero();
}

void AC_PID_2D::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _imax.load();
    _imax = fabsf(_imax);
    _filt_hz.load();
    _filt_d_hz.load();

    // calculate the input filter alpha
    calc_filt_alpha();
    calc_filt_alpha_d();
}

// save_gains - save gains to eeprom
void AC_PID_2D::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _imax.save();
    _filt_hz.save();
    _filt_d_hz.save();
}

// calc_filt_alpha - recalculate the input filter alpha
void AC_PID_2D::calc_filt_alpha()
{
    if (is_zero(_filt_hz)) {
        _filt_alpha = 1.0f;
        return;
    }
  
    // calculate alpha
    const float rc = 1/(M_2PI*_filt_hz);
    _filt_alpha = _dt / (_dt + rc);
}

// calc_filt_alpha - recalculate the input filter alpha
void AC_PID_2D::calc_filt_alpha_d()
{
    if (is_zero(_filt_d_hz)) {
        _filt_alpha_d = 1.0f;
        return;
    }

    // calculate alpha
    const float rc = 1/(M_2PI*_filt_d_hz);
    _filt_alpha_d = _dt / (_dt + rc);
}
