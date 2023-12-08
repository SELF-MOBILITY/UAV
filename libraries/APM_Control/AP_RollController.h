#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "AP_AutoTune.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

class AP_RollController {
public:
	void pso_read(void)
	{
	    FILE *fp;
	//	AP_Vehicle::st data;
//	    AP_Vehicle::stASMC data;
		AP_Vehicle::stFinite data_finite;
		
	    fp=fopen("data_orig.txt","rb");
	    if(!fp)
	    {
	        printf("pso_read open data_orig.txt error!\n");
	        fclose(fp);
	        exit(-1);
	    }

	    if(0==fread(&data_finite ,sizeof(data_finite ),1,fp))
	    {
	        printf("pso_read read data_orig.txt error");
	        fclose(fp);
	        exit(-1);
	    }
	    fclose(fp);
	    
/*	    printf("Roll data:1=%f,2=%f,3=%f,4=%f,5=%f,6=%f,7=%f,8=%f,9=%f,10=%f,11=%f,12=%f,13=%f\n", \
	        data.tecs_thrDamp,data.tecs_integGain,data.tecs_ptchDamp,data.pitch_I,data.pitch_P,data.pitch_D,data.Roll_I,data.Roll_P,data.Roll_D,data.Yaw_A,data.Yaw_I,data.Yaw_D,data.Yaw_Rll);
		gains.I =  data.Roll_I;
		gains.P =  data.Roll_P;
	    gains.D =  data.Roll_D;*/

/*	   printf("Roll data:1=%f,2=%f,3=%f,4=%f,5=%f,6=%f,7=%f,8=%f,9=%f,10=%f,11=%f\n", \
	        data.sat_Eps,data.tecs_thrAlfa,data.tecs_ptchAlfa,data.tecs_thrEta,data.tecs_ptchEta,data.pitch_Alfa,data.pitch_Eta,data.Roll_Alfa,data.Roll_Eta,data.Yaw_Alfa,data.Yaw_Eta);
		_asmc_alfa = data.Roll_Alfa;//1000.0;
		_sat_eps = data.Roll_Eta;//10.0;
	//	_eta = data.Roll_Eta;//1.3959;*/

		
		_gammaRoll = data_finite.gamma;
		_varepsRoll = data_finite.vareps;
		_lambda3Roll = data_finite.lambda3;//28.52;

		_vRoll = data_finite.roll_v;
		_sigmaRoll = data_finite.roll_sigma;
		
	    return;
	}
    AP_RollController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms)
        : aparm(parms)
        , autotune(gains, AP_AutoTune::AUTOTUNE_ROLL, parms)
        , _ahrs(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _slew_rate_filter.set_cutoff_frequency(10.0f);
        _slew_rate_filter.reset(0.0f);
		
	//	pso_read();
    }

    /* Do not allow copies */
    AP_RollController(const AP_RollController &other) = delete;
    AP_RollController &operator=(const AP_RollController&) = delete;

	int32_t get_rate_out(float desired_rate, float scaler);
	int32_t get_servo_out(int32_t angle_err, float scaler, bool disable_integrator);

	void reset_I();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I() {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
    }
    
    void autotune_start(void) { autotune.start(); }
    void autotune_restore(void) { autotune.stop(); }

	float saturation(float x);
	float _update_roll_adaptive_robust_rule(float pid_sum, float error, float error_dot, float error_int, float delta_time);
	float sign(float x);
	float _update_roll_finite_time_adaptive_rule(float pid_sum, float error, float error_dot, float error_int, float delta_time);
	
    const       AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];


    // tuning accessors
    void kP(float v) { gains.P.set(v); }
    void kI(float v) { gains.I.set(v); }
    void kD(float v) { gains.D.set(v); }
    void kFF(float v) { gains.FF.set(v); }

    AP_Float &kP(void) { return gains.P; }
    AP_Float &kI(void) { return gains.I; }
    AP_Float &kD(void) { return gains.D; }
    AP_Float &kFF(void) { return gains.FF; }

private:
    const AP_Vehicle::FixedWing &aparm;
    AP_AutoTune::ATGains gains;
    AP_AutoTune autotune;
	uint32_t _last_t;
	float _last_out;

	float _last_sumPid = 0;
	float _satRoll=1000.0f;
	float _asmc_alfa=158.298;//    1000.0;// 1000.0;// 1000.0;//  79.746391;//               5.169132;//0.326;//170.75;//170.751556;//0.01f;//0.00284f;
	float _sat_eps= 1000.0;//1000.0;// 88.044846;//   100;// 100.0;       //100.0;//30.018;//4.125;//4.125474;//1.0f;
	float _eta = 1;// 1.01;             //0.000003;//   0.01;//0.000004;//               0.00000;//000001;//0.000001;//0.00000001;//0.000066f;

	
/*	float _asmc_alfa=0.01f;//0.00284f;
	float _sat_eps=1.0f;
	float _eta = 0.00000001;//0.000066f;*/
	
	float _lambda3Roll =   0.001;//                       0.002512;
	float _gammaRoll =    0.001;//                          0.002492;         //no optimizatin 0.5
    float _varepsRoll =   10.0;//                            0.048459;        //no optimizatin 0.5
	float _upsilonRoll =   1000.0;//                         2.545184;
	float _betaRoll =     158.298;//                            11.014763;
	float _vRoll =     0.99;//                                0.999;             //no optimizatin  0.07
	float _sigmaRoll =    0.001;//                            1.849223;
	

	// roll ASMC controller integraor parameter, the upper saturation limit can be tuned by _satRoll, the lower limit is 0
	float _intK0Roll = 0.00001;
	float _intK1Roll = 0.00001;
	float _intK2Roll = 0.00001;

    AP_Logger::PID_Info _pid_info;

	int32_t _get_rate_out(float desired_rate, float scaler, bool disable_integrator);

	AP_AHRS &_ahrs;

    // D gain limit cycle control
    float _last_pid_info_D;                 // value of the D term (angular rate control feedback) from the previous time step (deg)
    LowPassFilterFloat _slew_rate_filter;   // LPF applied to the derivative of the control action generated by the angular rate feedback
    float _slew_rate_amplitude;             // Amplitude of the servo slew rate produced by the angular rate feedback (deg/sec)
    float _D_gain_modifier = 1.0f;          // Gain modifier applied to the angular rate feedback to prevent excessive slew rate
    AP_Float _slew_rate_max;                // Maximum permitted angular rate control feedback servo slew rate (deg/sec)
    AP_Float _slew_rate_tau;                // Time constant used to recover gain after a slew rate exceedance (sec)

};
