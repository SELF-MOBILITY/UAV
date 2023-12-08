#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>
#include <cmath>

class AP_YawController {
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
	    
/*	    printf("Yaw data:tecs_thrDamp=%f,tecs_integGain=%f,tecs_ptchDamp=%f,pitch_I=%f,pitch_P=%f,pitch_D=%f,Roll_I=%f,Roll_P=%f,Roll_D=%f,Yaw_A=%f,Yaw_I=%f,Yaw_D=%f,Yaw_Rll=%f\n", \
	        data.tecs_thrDamp,data.tecs_integGain,data.tecs_ptchDamp,data.pitch_I,data.pitch_P,data.pitch_D,data.Roll_I,data.Roll_P,data.Roll_D,data.Yaw_A,data.Yaw_I,data.Yaw_D,data.Yaw_Rll);
	//	_K_A = data.Yaw_A;//2.79;
		_K_I = data.Yaw_I;//0.00;
	    _K_D =  data.Yaw_D;
	//	_K_FF = 0.84;//data.Yaw_Rll;*/

/*	    printf("Yaw data:1=%f,2=%f,3=%f,4=%f,5=%f,6=%f,7=%f,8=%f,9=%f,10=%f,11=%f\n", \
	        data.sat_Eps,data.tecs_thrAlfa,data.tecs_ptchAlfa,data.tecs_thrEta,data.tecs_ptchEta,data.pitch_Alfa,data.pitch_Eta,data.Roll_Alfa,data.Roll_Eta,data.Yaw_Alfa,data.Yaw_Eta);
		_asmc_alfa = data.Yaw_Alfa;//985.0996;
		_sat_eps = data.Yaw_Eta;//10.0;
	//	_eta = data.Yaw_Eta;//1000.0;  */
		
		_gammaYaw = data_finite.gamma;
		_varepsYaw = data_finite.vareps;
		_lambda3Yaw = data_finite.lambda3;//28.52;

		_vYaw = data_finite.yaw_v;
		_sigmaYaw = data_finite.yaw_sigma;
		
		
		
	    return;
	}
	
    AP_YawController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms)
        : aparm(parms)
        , _ahrs(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _pid_info.target = 0;
        _pid_info.FF = 0;
        _pid_info.P = 0;

	//	pso_read();
    }

    /* Do not allow copies */
    AP_YawController(const AP_YawController &other) = delete;
    AP_YawController &operator=(const AP_YawController&) = delete;

	int32_t get_servo_out(float scaler, bool disable_integrator);

	void reset_I();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I() {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
    }

	float saturation(float x);
	float sign(float x);
	float _update_yaw_adaptive_robust_rule(float pid_sum, float error, float error_dot, float delta_time);
	float _update_yaw_finite_time_adaptive_rule(float pid_sum, float error_dot, float error_int, float delta_time);
    
	const AP_Logger::PID_Info& get_pid_info(void) const {return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];

private:
    const AP_Vehicle::FixedWing &aparm;
	AP_Float _K_A;
	AP_Float _K_I;
	AP_Float _K_D;
	AP_Float _K_FF;
    AP_Int16 _imax;
	uint32_t _last_t;
	float _last_out;
	float _last_rate_hp_out;
	float _last_rate_hp_in;
	float _K_D_last;
	float _satYaw=1000.0f;
	float _asmc_alfa=  0.001;//0.021514;//   0.000108;//0.005431;//0.002019;//1.138;//220.0f;//221.96;//0.007f;//0.006779f;
	float _sat_eps=0.001;//0.000102;//0.0001;// 0.003361;//30.018;//4.125;//4.125474;//1.0f;
	float _eta = 1;//0.0001;//0.03f;//85.607f;//85.607552f;
	float _last_sumPid = 0;
	
/*	float _asmc_alfa=220.0f;//221.96;//0.007f;//0.006779f;
	float _sat_eps=1.0f;
	float _eta = 0.03f;//85.607f;//85.607552f;*/
	
	float _lambda3Yaw =   0.001;//                          0.002512;
	float _gammaYaw =   0.001;//                             0.002492;         //no optimizatin 0.5
    float _varepsYaw =   10.0;//                              0.048459;        //no optimizatin 0.5
	float _upsilonYaw =   0.001;//                                   2.545184;
	float _betaYaw =    0.001;//                            11.014763;
	float _vYaw =   0.472179;//                               0.999;             //no optimizatin  0.07
	float _sigmaYaw =  1.4;//  14.872527;//                     1.849223;

	
	// yaw ASMC controller integraor parameter, the upper saturation limit can be tuned by _satYaw, the lower limit is 0
	float _intK0Yaw = 0.00001;
	float _intK1Yaw = 0.00001;
	float _intK2Yaw = 0.00001;

	float _integrator;

	AP_Logger::PID_Info _pid_info;

	AP_AHRS &_ahrs;
};
