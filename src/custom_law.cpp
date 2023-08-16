/**
 * @file demo_sine_torque_1_motor.cpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include <signal.h>
#include <atomic>

#include "generic_torque_actuation/actuators_interface.hpp"


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





    // sine torque params
    double current_amplitude = 1.5;
    double current_period = 1.5;
    double current_pulsation = 2 * 3.1415 / current_period;

    // here is the control in current (Ampere)
    double desired_current = 0.0;


    const double default_current = 0.1;


    ::generic_torque_actuation::ActuationLaws actuations_laws;
    actuations_laws.push_back(
        [&](const double &t_time)->double{

            double theta = current_pulsation * t_time;

            desired_current = default_current +
                current_amplitude * (1 - cos(theta))/2;

            return desired_current;
        }
        );



    ::generic_torque_actuation::ActuatorsInterface actuators_interface(actuations_laws,
                                                                       m_start_recording);

    actuators_interface.controller->start_loop();

    rt_printf("loops have started \n");

    // Wait until the application is killed.
    while (!StopDemos)
    {
        real_time_tools::Timer::sleep_sec(0.01);
    }

    actuators_interface.controller->stop_loop();

    return 0;
}
