/// @file    AP_TECS.h
/// @brief   Combined Total Energy Speed & Height Control. This is a instance of an
/// AP_SpdHgtControl class

/*
 *  Written by Paul Riseborough 2013 to provide:
 *  - Combined control of speed and height using throttle to control
 *    total energy and pitch angle to control exchange of energy between
 *    potential and kinetic.
 *    Selectable speed or height priority modes when calculating pitch angle
 *  - Fallback mode when no airspeed measurement is available that
 *    sets throttle based on height rate demand and switches pitch angle control to
 *    height priority
 *  - Underspeed protection that demands maximum throttle switches pitch angle control
 *    to speed priority mode
 *  - Relative ease of tuning through use of intuitive time constant, trim rate and damping parameters and the use
 *    of easy to measure aircraft performance data
 */
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_Landing/AP_Landing.h>

class AP_TECS : public AP_SpdHgtControl {
public:
	void pso_read(void)
	{
	    FILE *fp;
	//	AP_Vehicle::st data;
	//    AP_Vehicle::stASMC data;
		AP_Vehicle::stFinite data_finite;
		
	    fp=fopen("data_orig.txt","rb");
	    if(!fp)
	    {
	        printf("pso_read open data_orig.txt error!\n");
	        fclose(fp);
	        exit(-1);
	    }

	     if(0==fread(&data_finite,sizeof(data_finite),1,fp))
	     {
	         printf("pso_read read data_orig.txt error");
	         fclose(fp);
	         exit(-1);
	     }
		
	    fclose(fp);
		
		
		
	    
/*	    printf("TECS data:1=%f,2=%f,3=%f,4=%f,5=%f,6=%f,7=%f,8=%f,9=%f,10=%f,11=%f,12=%f,13=%f\n", \
	        data.tecs_thrDamp,data.tecs_integGain,data.tecs_ptchDamp,data.pitch_I,data.pitch_P,data.pitch_D,data.Roll_I,data.Roll_P,data.Roll_D,data.Yaw_A,data.Yaw_I,data.Yaw_D,data.Yaw_Rll);
		_thrDamp =  data.tecs_thrDamp;
		_integGain =  data.tecs_integGain;
	    _ptchDamp =  data.tecs_ptchDamp;*/
	    
/*		printf("TECS data:1=%f,2=%f,3=%f,4=%f,5=%f,6=%f,7=%f,8=%f,9=%f,10=%f,11=%f\n", \
	        data.sat_Eps,data.tecs_thrAlfa,data.tecs_ptchAlfa,data.tecs_thrEta,data.tecs_ptchEta,data.pitch_Alfa,data.pitch_Eta,data.Roll_Alfa,data.Roll_Eta,data.Yaw_Alfa,data.Yaw_Eta);
		_asmc_thrAlfa = data.tecs_thrAlfa;//56.22;
		_asmc_ptchAlfa = data.tecs_ptchAlfa;//0.01;
		_sat_thrEps = data.tecs_thrEta;//1;
		_sat_pitchEps = data.tecs_ptchEta;//28.52;
		_gainThr =data.sat_Eps;//1197.97;
		*/
		
	/*	printf("TECS data: gamma=%f, vareps = %f, upsilon = %f, lambda3Throttle=%f,betaThrottle=%f,vThrottle=%f,sigmaThrottle=%f,lambda3Pitch=%f,betaPitch=%f,vPitch=%f,sigmaPitch=%f   \n", \
	        data_finite.tecs_gamma, data_finite.tecs_vareps, data_finite.tecs_upsilon, data_finite.tecs_lambda3Throttle,data_finite.tecs_betaThrottle,data_finite.tecs_vThrottle,data_finite.tecs_sigmaThrottle,data_finite.tecs_lambda3Pitch,data_finite.tecs_betaPitch,data_finite.tecs_vPitch,data_finite.tecs_sigmaPitch);
		_gammaThrottle = data_finite.tecs_gamma;
		_gammaPitch = data_finite.tecs_gamma;
		_varepsThrottle = data_finite.tecs_vareps;
		_varepsPitch = data_finite.tecs_vareps;
		_upsilonThrottle = data_finite.tecs_upsilon;
		_upsilonPitch = data_finite.tecs_upsilon;*/

		_lambda3Throttle = data_finite.lambda3;//56.22;
		_gammaThrottle = data_finite.gamma;//1;
		_varepsThrottle = data_finite.vareps;
		_vThrottle = data_finite.tecsthr_v;
		_sigmaThrottle =data_finite.tecsthr_sigma;//1197.97;
		
		_lambda3Pitch = data_finite.lambda3;//28.52;
		_gammaPitch = data_finite.gamma;
		_varepsPitch = data_finite.vareps;
		_vPitch = data_finite.tecspitch_v;
		_sigmaPitch = data_finite.tecspitch_sigma;    
	    return;
    }

AP_TECS(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms, const AP_Landing &landing)
        : _ahrs(ahrs)
        , aparm(parms)
        , _landing(landing)
    {
        AP_Param::setup_object_defaults(this, var_info);
	//	pso_read();
    }

    /* Do not allow copies */
    AP_TECS(const AP_TECS &other) = delete;
    AP_TECS &operator=(const AP_TECS&) = delete;

    // Update of the estimated height and height rate internal state
    // Update of the inertial speed rate internal state
    // Should be called at 50Hz or greater
    void update_50hz(void) override;

    // Update the control loop calculations
    void update_pitch_throttle(int32_t hgt_dem_cm,
                               int32_t EAS_dem_cm,
                               enum AP_Vehicle::FixedWing::FlightStage flight_stage,
                               float distance_beyond_land_wp,
                               int32_t ptchMinCO_cd,
                               int16_t throttle_nudge,
                               float hgt_afe,
                               float load_factor) override;

    // demanded throttle in percentage
    // should return -100 to 100, usually positive unless reverse thrust is enabled via _THRminf < 0
    int32_t get_throttle_demand(void) override {
        return int32_t(_throttle_dem * 100.0f);
    }

    // demanded pitch angle in centi-degrees
    // should return between -9000 to +9000
    int32_t get_pitch_demand(void) override {
        return int32_t(_pitch_dem * 5729.5781f);
    }

    // Rate of change of velocity along X body axis in m/s^2
    float get_VXdot(void) override {
        return _vel_dot;
    }

    // return current target airspeed
    float get_target_airspeed(void) const override {
        return _TAS_dem_adj / _ahrs.get_EAS2TAS();
    }

    // return maximum climb rate
    float get_max_climbrate(void) const override {
        return _maxClimbRate;
    }

    // added to let SoaringContoller reset pitch integrator to zero
    void reset_pitch_I(void) override {
        _integSEB_state = 0.0f;
    }
    
    // return landing sink rate
    float get_land_sinkrate(void) const override {
        return _land_sink;
    }

    // return landing airspeed
    float get_land_airspeed(void) const override {
        return _landAirspeed;
    }

    // return height rate demand, in m/s
    float get_height_rate_demand(void) const {
        return _hgt_rate_dem;
    }

    // set path_proportion
    void set_path_proportion(float path_proportion) override {
        _path_proportion = constrain_float(path_proportion, 0.0f, 1.0f);
    }

    // set soaring flag
    void set_gliding_requested_flag(bool gliding_requested) override {
        _flags.gliding_requested = gliding_requested;
    }

    // set propulsion failed flag
    void set_propulsion_failed_flag(bool propulsion_failed) override {
        _flags.propulsion_failed = propulsion_failed;
    }


    // set pitch max limit in degrees
    void set_pitch_max_limit(int8_t pitch_limit) {
        _pitch_max_limit = pitch_limit;
    }

    // force use of synthetic airspeed for one loop
    void use_synthetic_airspeed(void) {
        _use_synthetic_airspeed_once = true;
    }

    // reset on next loop
    void reset(void) override {
        _need_reset = true;
    }

    // this supports the TECS_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    // Last time update_50Hz was called
    uint64_t _update_50hz_last_usec;

    // Last time update_speed was called
    uint64_t _update_speed_last_usec;

    // Last time update_pitch_throttle was called
    uint64_t _update_pitch_throttle_last_usec;

    // reference to the AHRS object
    AP_AHRS &_ahrs;

    const AP_Vehicle::FixedWing &aparm;

    // reference to const AP_Landing to access it's params
    const AP_Landing &_landing;
    
    // TECS tuning parameters
    AP_Float _hgtCompFiltOmega;
    AP_Float _spdCompFiltOmega;
    AP_Float _maxClimbRate;
    AP_Float _minSinkRate;
    AP_Float _maxSinkRate;
    AP_Float _timeConst;
    AP_Float _landTimeConst;
    AP_Float _ptchDamp;
    AP_Float _land_pitch_damp;
    AP_Float _landDamp;
    AP_Float _thrDamp;
    AP_Float _land_throttle_damp;
    AP_Float _integGain;
    AP_Float _integGain_takeoff;
    AP_Float _integGain_land;
    AP_Float _vertAccLim;
    AP_Float _rollComp;
    AP_Float _spdWeight;
    AP_Float _spdWeightLand;
    AP_Float _landThrottle;
    AP_Float _landAirspeed;
    AP_Float _land_sink;
    AP_Float _land_sink_rate_change;
    AP_Int8  _pitch_max;
    AP_Int8  _pitch_min;
    AP_Int8  _land_pitch_max;
    AP_Float _maxSinkRate_approach;
    AP_Int32 _options;
	float _satThrottle = 1000.0f;
	float _satPitch = 1000.0f;
	float _asmc_thrAlfa = 0.00115;   // 0.011238;//0.003098;//0.009;   //0.00001;//  0.001146;//           0.000032;//0.003282;//0.00065;//0.000554;//0.07; //0.432f;//0.431868f;       increase  thrAlpha and  decrease eta, then we can derease the height to active  adaptive term
	float _asmc_ptchAlfa = 4.2155;//   0.069039;//0.01041;// 1.09;//1.08;          //1000.0;//  1.087006;//           0.00019;//0.000011;// 0.00514;//0.013057;//0.01f;//0.009908f;
	float _sat_thrEps =  10.407 ;//    10.407
	float _sat_pitchEps = 44.339;//   0.583554;// 200.540253;//  85.678867;          //88.044846;//   100;// 100.0;//            100.0;//30.018;//4.125;//4.125474;//5.320279;//4.349751;//1.0f;
	float _thrEta = 1;//2.8;         //5.879673;//  1.4;//              100.0;//0.005;//0.004789;//1.0;//0.02f;//0.004f;//0.003907f;          5.5                     tune   eta can  confirm  altitude,    alpha   can  tune   the cost
	float _pitchEta =1;//1.32;          //0.000051;//  80.0;//             1.318043;// 2.58;//0.315;//0.31523;//1.0;//0.00000001f;//0.000119f;
	float _last_sumThrPid = 0;
	float _last_sumPithPid = 0;
	float _gainThr =  0;
	
	uint32_t _saturationFlag = 2;

/*	float _asmc_thrAlfa = 0.07; //0.432f;//0.431868f;
	float _asmc_ptchAlfa = 0.01f;//0.009908f;
	float _sat_eps = 1.0f;
	float _thrEta = 0.02f;//0.004f;//0.003907f;
	float _pitchEta = 0.00000001f;//0.000119f;*/
	
	
		// finite-time control
	float _lambda3Throttle =  0.001;
	float _gammaThrottle =  0.001;      //no optimizatin 0.5
	float _varepsThrottle = 10.0;     //no optimizatin 0.5
	float _upsilonThrottle =    10.407;  //  22.407
	float _betaThrottle = 0.00115;   ///    0.00115
	float _vThrottle = 0.013785;//  0.013767;           //no optimizatin  0.07
	float _sigmaThrottle = 0.038116;// 0.03878;  //  0.03878

	float _lambda3Pitch =  0.001;
	float _gammaPitch =  0.001;         //no optimizatin 0.5
    float _varepsPitch = 10.0;        //no optimizatin 0.5
	float _upsilonPitch =   44.339;
	float _betaPitch =   4.2155;
	float _vPitch =   0.471472 ;             //no optimizatin  0.07
	float _sigmaPitch =   0.05;// 0.001 ;
	
	

    enum {
        OPTION_GLIDER_ONLY=(1<<0),
    };

    AP_Float _pitch_ff_v0;
    AP_Float _pitch_ff_k;

    // temporary _pitch_max_limit. Cleared on each loop. Clear when >= 90
    int8_t _pitch_max_limit = 90;
    
    // current height estimate (above field elevation)
    float _height;

    // throttle demand in the range from -1.0 to 1.0, usually positive unless reverse thrust is enabled via _THRminf < 0
    float _throttle_dem;

    // pitch angle demand in radians
    float _pitch_dem;

    // estimated climb rate (m/s)
    float _climb_rate;

	// throttle ASMC controller integraor parameter, the upper saturation limit can be tuned by _satThrottle, the lower limit is 0
	float _intK0Thr = 0.00001;
    float _intK1Thr = 0.00001;
    float _intK2Thr = 0.00001;

	// pitch ASMC controller integraor parameter, the upper saturation limit can be tuned by _satPitch, the lower limit is 0
	float _intK0Pitch = 0.00001;
    float _intK1Pitch = 0.00001;
    float _intK2Pitch = 0.00001;

    /*
      a filter to estimate climb rate if we don't have it from the EKF
     */
    struct {
        // height filter second derivative
        float dd_height;

        // height integration
        float height;
    } _height_filter;

    // Integrator state 4 - airspeed filter first derivative
    float _integDTAS_state;

    // Integrator state 5 - true airspeed
    float _TAS_state;

    // Integrator state 6 - throttle integrator
    float _integTHR_state;

    // Integrator state 6 - pitch integrator
    float _integSEB_state;

    // throttle demand rate limiter state
    float _last_throttle_dem;

    // pitch demand rate limiter state
    float _last_pitch_dem;

    // Rate of change of speed along X axis
    float _vel_dot;

    // Equivalent airspeed
    float _EAS;

    // True airspeed limits
    float _TASmax;
    float _TASmin;

    // Current true airspeed demand
    float _TAS_dem;

    // Equivalent airspeed demand
    float _EAS_dem;

    // height demands
    float _hgt_dem;
    float _hgt_dem_in_old;
    float _hgt_dem_adj;
    float _hgt_dem_adj_last;
    float _hgt_rate_dem;
    float _hgt_dem_prev;
    float _land_hgt_dem;

    // Speed demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_dem_adj;

    // Speed rate demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_rate_dem;

    // Total energy rate filter state
    float _STEdotErrLast;

    struct flags {
        // Underspeed condition
        bool underspeed:1;

        // Bad descent condition caused by unachievable airspeed demand
        bool badDescent:1;

        // true when plane is in auto mode and executing a land mission item
        bool is_doing_auto_land:1;

        // true when we have reached target speed in takeoff
        bool reached_speed_takeoff:1;

        // true if the soaring feature has requested gliding flight
        bool gliding_requested:1;

        // true when we are in gliding flight, in one of three situations;
        //   - THR_MAX=0
        //   - gliding has been requested e.g. by soaring feature
        //   - engine failure detected (detection not implemented currently)
        bool is_gliding:1;

        // true if a propulsion failure is detected.
        bool propulsion_failed:1;
    };
    union {
        struct flags _flags;
        uint8_t _flags_byte;
    };

    // time when underspeed started
    uint32_t _underspeed_start_ms;

    // auto mode flightstage
    enum AP_Vehicle::FixedWing::FlightStage _flight_stage;

    // pitch demand before limiting
    float _pitch_dem_unc;

    // Maximum and minimum specific total energy rate limits
    float _STEdot_max;
    float _STEdot_min;

    // Maximum and minimum floating point throttle limits
    float _THRmaxf;
    float _THRminf;

    // Maximum and minimum floating point pitch limits
    float _PITCHmaxf;
    float _PITCHminf;

    // Specific energy quantities
    float _SPE_dem;
    float _SKE_dem;
    float _SPEdot_dem;
    float _SKEdot_dem;
    float _SPE_est;
    float _SKE_est;
    float _SPEdot;
    float _SKEdot;

    // Specific energy error quantities
    float _STE_error;

    // Time since last update of main TECS loop (seconds)
    float _DT;

    // counter for demanded sink rate on land final
    uint8_t _flare_counter;

    // slew height demand lag filter value when transition to land
    float hgt_dem_lag_filter_slew;

    // percent traveled along the previous and next waypoints
    float _path_proportion;

    float _distance_beyond_land_wp;

    float _land_pitch_min = -90;

    // need to reset on next loop
    bool _need_reset;

    // internal variables to be logged
    struct {
        float SKE_weighting;
        float SPE_error;
        float SKE_error;
        float SEB_delta;
    } logging;

    AP_Int8 _use_synthetic_airspeed;
    
    // use synthetic airspeed for next loop
    bool _use_synthetic_airspeed_once;
    
    // Update the airspeed internal state using a second order complementary filter
    void _update_speed(float load_factor);

    // Update the demanded airspeed
    void _update_speed_demand(void);

    // Update the demanded height
    void _update_height_demand(void);

    // Detect an underspeed condition
    void _detect_underspeed(void);

    // Update Specific Energy Quantities
    void _update_energies(void);

	// Get throttle adaptive robust rule term
	float _update_throttle_adaptive_robust_rule(float pid_sum, float error, float error_dot, float error_int);

	// Get pitch adaptive robust rule term
	float _update_pitch_adaptive_robust_rule(float pid_sum, float error, float error_dot, float error_int);
	
	// Get throttle finite-time adaptive rule term
	float _update_throttle_finite_time_adaptive_rule(float pid_sum, float error, float error_dot, float error_int);

	// Get pitch finite-time adaptive rule term
	float _update_pitch_finite_time_adaptive_rule(float pid_sum, float error, float error_dot, float error_int);

    // Update Demanded Throttle
    void _update_throttle_with_airspeed(void);

    // Update Demanded Throttle Non-Airspeed
    void _update_throttle_without_airspeed(int16_t throttle_nudge);

    // get integral gain which is flight_stage dependent
    float _get_i_gain(void);

    // Detect Bad Descent
    void _detect_bad_descent(void);

    // Update Demanded Pitch Angle
    void _update_pitch(void);

    // Initialise states and variables
    void _initialise_states(int32_t ptchMinCO_cd, float hgt_afe);

    // Calculate specific total energy rate limits
    void _update_STE_rate_lim(void);

    // declares a 5point average filter using floats
    AverageFilterFloat_Size5 _vdot_filter;

    // current time constant
    float timeConstant(void) const;

	// calculate saturation value
	float saturation(float x);
	
	// calculate sign functino
	float sign(float x);
};
