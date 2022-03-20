#include "Hypersonic_Interceptor.h"
#include "Fractional_Interceptor.h"
#include "NonLinear_Proportional_Interceptor.h"
#include "Coordinated_Interceptor.h"
#include "xml/pugixml.hpp"

iInterceptor::iInterceptor():my_ptr(NULL)
{
}

iInterceptor::~iInterceptor()
{
    if (my_ptr) {
        delete my_ptr;
    }
}

bool iInterceptor::initialize(const std::string& xi_init_file_name, std::string& xo_message)
{
    pugi::xml_document Doc;
    pugi::xml_parse_result Result = Doc.load_file(xi_init_file_name.c_str());	
    if (Result) {}
    else {
        xo_message = "File not found or formatting error!";
        return false;
    }
    pugi::xml_node Root = Doc.child("Parameters");                              /* Accessing Root Node */
    if (Root) {}
    else {
        xo_message = "Root Node (Parameters) not found!";
        return false;
    }

    string my_algo = Root.child_value("Method");
    int method = 1 * (int)(my_algo == "Hypersonic") + 2 * (int)(my_algo == "Fractional") + 3 * (int)(my_algo == "NonLinear_Proportional") + 4 * (int)(my_algo == "Coordinated");
    switch (method) {
        case 1:
            my_ptr = new Hypersonic::Interceptor();
            break;
        case 2:
            my_ptr = new Fractional::Interceptor;
            break;
        case 3:
            my_ptr = new NonLinear_Proportional::Interceptor;
            break;
        case 4:
            my_ptr = new Coordinated::Interceptor;
            break;
        default:
            xo_message = "Invalid Method!";
            return false;
            break;
    }

    if (!my_ptr) {
        return false;
    }
    return (*my_ptr).Initialize(xi_init_file_name, xo_message);		
}

bool iInterceptor::reinitialize()
{
    if (!my_ptr) {
        return false;
    }
    (*my_ptr).Reinitialize();
	return true;
}

void iInterceptor::update_target_state(const objectstate& xi_target)
{
    if (my_ptr) {
        (*my_ptr).Update_Target_State(xi_target);
    }
}

bool iInterceptor::get_state(objectstate& xo_state)
{
    if (!my_ptr)
    {
        return false;
    }
	return (*my_ptr).Get_State(xo_state);
}

bool iInterceptor::is_started(double xi_current_time)
{
    if (!my_ptr)
    {
        return false;
    }
	return (*my_ptr).Is_Started();
}

bool iInterceptor::is_terminated(bool& xo_interceptor, bool& xo_target, int& xo_target_id)
{	
    if (!my_ptr)
    {
        return false;
    }
	return (*my_ptr).Is_Terminated(xo_interceptor, xo_target, xo_target_id);		
}

void iInterceptor::get_csv() 
{
    if (my_ptr) {
        (*my_ptr).Get_Record();
    }
}