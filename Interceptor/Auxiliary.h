#pragma once

#include <Interceptor/i_Interceptor.hpp>
#include <string>
#include <array>

using namespace std;

void Read_CSV(double, string, objectstate&);
void Calc_Local(objectstate, array<double, 2>&);
void Calc_Geo(objectstate&, array<double, 2>, objectstate);
bool Calc_Distance(objectstate Interceptor_State, objectstate Target_State, double& xo_dist, double& xo_angle);
void Limit(double&, double, double);
double signum(double);

double sind(double);
double cosd(double);
double tand(double);

double rad_to_deg(double);
double deg_to_rad(double);

double Read_Param(double, double, double, int&);
double deg_to_long(double);
double deg_to_lat(double);

class Interceptor_Template {
public:
    virtual ~Interceptor_Template() {};
    virtual bool Initialize(const std::string&, std::string&) = 0;
    virtual void Reinitialize() = 0;
    virtual void Update_Target_State(objectstate) = 0;
    virtual bool Get_State(objectstate&) = 0;
    virtual void Get_Record() = 0;
    virtual bool Is_Started() = 0;
    virtual bool Is_Terminated(bool&, bool&, int&) = 0;
};

