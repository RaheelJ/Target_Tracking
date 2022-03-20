#include<Interceptor/Fractional_Interceptor.h>
#include<Interceptor/Hypersonic_Interceptor.h>
#include<Interceptor/Hypersonic_Interceptor.h>
#include<Interceptor/NonLinear_Proportional_Interceptor.h>
#include<iostream>
#include<Interceptor/Auxiliary.h>

int main() {
    double t_meas = 0;                                      /* Start time of Target flight */
    iInterceptor I1;                                        /* Initialize the Interceptor */
    objectstate Target_State;                               
    objectstate Interceptor_State;
    Interceptor_State.id = 4;
    bool tracking, init, interceptor_terminate, target_hit;
    int target_id;
    string message;

    init = I1.initialize("Input/Interceptor_Parameters.xml", message);
    if (init) {}
    else {
        cout << message;
        return 0;
    }

   /* Loop for tracking */
    for (t_meas = 1; t_meas <= 1300; t_meas = t_meas + 1) {
        Read_CSV(t_meas, "Input/Target_2.csv", Target_State);           /* Read the Target state */
        cout << Target_State.x_or_lat << '\t' << Target_State.y_or_lon << '\t' << Target_State.z_or_alt << '\t' << Target_State.time << "\n";

        I1.update_target_state(Target_State);                            /* Follow the Target and save the Trajectory */

        tracking = I1.is_started(t_meas);
        if (tracking)
        {
            cout << "In Pursuit" << "\n";
        }
        
        I1.get_state(Interceptor_State);
        cout << Interceptor_State.x_or_lat << '\t' << Interceptor_State.y_or_lon << '\t' << Interceptor_State.z_or_alt << '\t' << Interceptor_State.time << "\n";
        cout << Interceptor_State.x_vel << '\t' << Interceptor_State.y_vel << '\t' << Interceptor_State.z_vel << "\n \n";

        I1.is_terminated(interceptor_terminate, target_hit, target_id);
        if (target_hit) {
            cout << '\n' << "Target Hit !!" << '\n';
            break;
        }
        else if (interceptor_terminate) {
            cout << '\n' << "Target Missed !!" << '\n';
            break;
        }
    }
    I1.get_csv();

    return 0;
}


