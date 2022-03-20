// Implementation of multiple different guidance laws 
// Pure Propotional Navigation Guidance Law
// Modified Propotional Navigation Guidance Law
// Augmented Proportional Navigation Guidance Law
// Differential Geometry Guidance Law
// The intialization of the parameters varies in different application scenarios 
// Writtenr by Pengfei Peter Kan, Feb 15, 2018
/////////////////////////////////////////////////////////////////////////
#ifndef _IHYPERSONIC_INTERCEPTOR_H
#define _IHYPERSONIC_INTERCEPTOR_H

#if defined(_WINDOWS) && defined(_APP_MODE_DLL)
#define DLLDIR __declspec(dllexport)
#else
#define DLLDIR
#endif

#include <string>

struct objectstate
{
	int                    id;

	double                 time;
	bool                   state_in_geo;

	double                 x_or_lat;
	double                 y_or_lon;
	double                 z_or_alt;

	double                 x_vel;
	double                 y_vel;
	double                 z_vel;
};

class Interceptor_Template; 

class DLLDIR iInterceptor
{
private:

public:
	iInterceptor();
	~iInterceptor();

	bool initialize(const std::string& xi_init_file_name, std::string& xo_message);
	bool reinitialize();

	void           update_target_state(const objectstate& xi_target);
	bool           get_state(objectstate& xo_state);
	bool           is_started(double xi_current_time);
	bool           is_terminated(bool& xo_interceptor, bool& xo_target, int& xo_target_id);
	void		   get_csv();

private:
	Interceptor_Template* my_ptr;
};

#endif // !_IHYPERSONIC_INTERCEPTOR_H

