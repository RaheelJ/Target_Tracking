#define _USE_MATH_DEFINES               /* Adds math constants */ 

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include "Auxiliary.h"

using namespace GeographicLib;

void Read_CSV(double t_meas, string file_name, objectstate& Target_State)
{
    fstream fin;                        // File pointer
    int time = 0, count = 0;

    fin.open(file_name);                // Open an existing file

    vector<string> row;                 // Read the data from the file as String Vector
    string line, word;
    getline(fin, line, '\n');

    while (fin) {
        row.clear();
        getline(fin, line, '\n');       // read an entire row and store it in a string variable 'line'
        stringstream s(line);           // used for breaking words

        // read every column data of a row and store it in a string variable, 'word'
        while (getline(s, word, ',')) {
            row.push_back(word);        // add all the column data of a row to a vector
        }

        time = stoi(row[0]);            // convert string to integer for comparision
        if (time == (int)t_meas) {      // Compare the time
            count = 1;
            Target_State.x_or_lat = stod(row[2]);
            Target_State.y_or_lon = stod(row[3]);
            Target_State.z_or_alt = stod(row[4]);
            Target_State.x_vel = stod(row[5]);
            Target_State.y_vel = stod(row[6]);
            Target_State.z_vel = stod(row[7]);
            Target_State.time = t_meas;
            break;
        }
    }
    if (count == 0)
        cout << "Record not found\n";
}
void Calc_Local(objectstate Geo_Position, array<double, 2>& Local_Position) {    /* Calculate Local Cartesian Coordinates from Geographic Coordinates */
    double temp_alt;
    Geocentric Earth(Constants::WGS84_a(), Constants::WGS84_f());
    LocalCartesian Local(0, 0, 0, Earth);                                       /* Reference Position for Local Cartesian Coordinates */
    Local.Forward(Geo_Position.x_or_lat, Geo_Position.y_or_lon, 0, Local_Position[0], Local_Position[1], temp_alt);
}
void Calc_Geo(objectstate& Geo_Position, array<double, 2> Local_Position, objectstate Reference) {
    double temp_alt;
    Geocentric Earth(Constants::WGS84_a(), Constants::WGS84_f());
    LocalCartesian Local(Reference.x_or_lat, Reference.y_or_lon, 0, Earth);     /* Reference Position for Local Cartesian Coordinates */
    Local.Reverse(Local_Position[0], Local_Position[1], 0, Geo_Position.x_or_lat, Geo_Position.y_or_lon, temp_alt);
}
bool Calc_Distance(objectstate Interceptor_State, objectstate Target_State, double& xo_dist, double& xo_angle) {
	Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
    double angle2;
	geod.Inverse(Interceptor_State.x_or_lat, Interceptor_State.y_or_lon, Target_State.x_or_lat, Target_State.y_or_lon, xo_dist, xo_angle, angle2);
	return true;
}

void Limit(double& in, double upper_lim, double lower_lim)
{
    if (in <= lower_lim) { in = lower_lim; }
    else if (in >= upper_lim) { in = upper_lim; }
}
double signum(double in) {
    double out = ((in > 0) - (in < 0));
    return out;
}

double sind(double in) {                /* Sine Function with Argument in Degrees */
    return sin(fmod((in), 360) * M_PI / 180);
}
double cosd(double in) {                /* Cosine Function with Argument in Degrees */
    return cos(fmod((in), 360) * M_PI / 180);
}
double tand(double in) {                /* Cosine Function with Argument in Degrees */
    return tan(fmod((in), 360) * M_PI / 180);
}

double rad_to_deg(double in) {
    return 180 * fmod((in), 2 * M_PI) / M_PI;
}
double deg_to_rad(double in) {
    return M_PI * fmod((in), 360) / 180;
}

double Read_Param(double param, double lower_lim, double upper_lim, int& error)
{
    double out = 0;
    if (param <= upper_lim && param >= lower_lim) {
        out = param;
    }
    else {
        error = error + 1;
    }
    return out;
}
double deg_to_long(double in) {
    double out = 0;
    if (in <= -180) {
        out = in + 360;
    }
    else if (in >= 180) {
        out = out - 360;
    }
    else {
        out = in;
    }
    return out;
}
double deg_to_lat(double in) {
    double out;
    if (-270 <= in && in <= -90) {
        out = -180 - in;
    }
    else if (in <= -270) {
        out = 360 + in;
    }
    else if (90 <= in && in <= 270) {
        out = 180 - in;
    }
    else if (in >= 270) {
        out = -360 + in;
    }
    else {
        out = in;
    }
    return out;
}

