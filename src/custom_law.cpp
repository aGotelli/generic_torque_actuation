/**
 * @file demo_sine_torque_1_motor.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include <signal.h>
#include <atomic>

#include "generic_torque_actuation/actuators_interface.hpp"


#include "utilities/Eigen/eigen_io.hpp"


std::shared_ptr<bool> m_start_recording = std::make_shared<bool>(true);

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool StopDemos(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 *
 * @param s
 */
void my_handler(int)
{
    StopDemos = true;
}

/**
 * @brief This is the main demo program.
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int, char **)
{
    // make sure we catch the ctrl+c signal to kill the application properly.
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    StopDemos = false;










    auto null_fun = [&](const double &t_time) -> double {

        return 0.0;
    };


    auto ramp_fun = [&](const double &t_time) -> double {

        // here is the control in current (Ampere)
        double desired_current = 0.0;
        const double default_current = 0.1;

        const double initial_time = 2;


        double rampup_duration = 1;
        double rampup_end_time = initial_time + rampup_duration;

        double top_current = 3;
        double rampup_gain = top_current/rampup_duration;


        double time;

        if(t_time < initial_time)
            return default_current;


        if(t_time <= rampup_end_time){
            //  Here we are in the beginning of the slope
            time = t_time - initial_time;

            desired_current = default_current + time*rampup_gain;

            return desired_current;
        }

        desired_current = default_current + top_current;

        return desired_current;
    };





    auto cos_fun = [&](const double &t_time)->double{

        // here is the control in current (Ampere)
        double desired_current = 0.0;
        const double default_current = 0.1;

        const double initial_time = 5;

        // sine torque params
        const double current_amplitude = 1.5;
        const double frequency = 0.5;
        const double omega = 2 * M_PI * frequency;

        if(t_time <= initial_time /*||
            t_time > total_time*/)
            return default_current;



        double theta = omega * (t_time - initial_time);

        desired_current = default_current +
                          current_amplitude * (1 - cos(theta))/2;


        return desired_current;
    };


    auto cos_fun2 = [&](const double &t_time)->double{

        // here is the control in current (Ampere)
        double desired_current = 0.0;
        const double default_current = 0.1;



        // sine torque params
        const double current_amplitude = 1.5;
        const double frequency = 0.5;
        const double omega = 2 * M_PI * frequency;


        const double initial_time = 5 + 0.5*1/frequency;
//        const double duration = 10;
//        const double total_time = initial_time + duration;

        if(t_time <= initial_time/* ||
            t_time > total_time*/)
            return default_current;



        double theta = omega * (t_time - initial_time);

        desired_current = default_current +
                          current_amplitude * (1 - cos(theta))/2;


        return desired_current;
    };


    ::generic_torque_actuation::ActuationLaws actuations_laws;
//    actuations_laws.push_back( cos_fun2 );
    actuations_laws.push_back( cos_fun );
    actuations_laws.push_back( cos_fun2 );






    std::vector<int> premultipliers {
        1,
        -1
    };



    ::generic_torque_actuation::ActuatorsInterface actuators_interface(actuations_laws,
                                                                       m_start_recording,
                                                                       premultipliers);

    actuators_interface.controller->start_loop();

    rt_printf("loops have started \n");

    // Wait until the application is killed.
    while (!StopDemos)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    actuators_interface.controller->stop_loop();


    YAML::Node actuator_node;
    Eigen::MatrixXd actuator_data;

    actuators_interface.getSamplesData(actuator_node, actuator_data);


    const std::string path = "data/simple_benchmark/dual_motor/robot_board/double_sine/";

    SaveFile(actuator_node, "configuration_file.yaml", path);

    writeToFile("actuator_data", actuator_data, path);






    return 0;
}
