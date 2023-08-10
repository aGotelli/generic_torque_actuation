/**
 * @file sine_torque_control.hpp
 * @copyright Copyright (c) 2018-2020, New York University and Max Planck
 * Gesellschaft, License BSD-3-Clause
 */

#include "blmc_drivers/devices/motor.hpp"
#include <memory>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

namespace generic_torque_actuation
{
/**
 * @brief This is a simple shortcut
 */
typedef std::shared_ptr<blmc_drivers::SafeMotor> SafeMotor_ptr;


typedef std::vector<std::function<double(const double &)>> ActuationLaws;

/**
 * @brief This is a basic PD controller to be used in the demos of this package.
 */
class GenericTorqueControl
{
public:
    GenericTorqueControl()=default;

    /**
     * @brief Construct a new SineTorqueControl object.
     *
     * @param motor_slider_pairs
     */
    GenericTorqueControl(std::vector<SafeMotor_ptr> motor_list,
                         ActuationLaws t_actuation_law);

    /**
     * @brief Destroy the SineTorqueControl object
     */
    ~GenericTorqueControl();

    /**
     * @brief This method is a helper to start the thread loop.
     */
    void start_loop();
    /**
     * @brief Stop the control and dump the data
     */
    void stop_loop();




    /**
     * @brief This is list of motors
     */
    std::vector<SafeMotor_ptr> motor_list_;
    /**
     * @brief This is the real time thread object.
     */
    real_time_tools::RealTimeThread rt_thread_;

    /**
     * @brief this function is just a wrapper around the actual loop function,
     * such that it can be spawned as a posix thread.
     */
    static THREAD_FUNCTION_RETURN_TYPE loop(void* instance_pointer);

    /**
     * @brief this is a simple control loop which runs at a kilohertz.
     *
     * it reads the measurement from the analog sensor, in this case the
     * slider. then it scales it and sends it as the current target to
     * the motor.
     */
    void loop();

    std::shared_ptr<bool> m_start_recording {
        std::make_shared<bool>(false) };

    /**
     * @brief managing the stopping of the loop
     */
    bool stop_loop_;

    /**
     * @brief memory_buffer_size_ is the max size of the memory buffer.
     */
    unsigned memory_buffer_size_;

    /**
     * @brief Encoder data
     */
    std::vector<std::deque<double> > encoders_;

    /**
     * @brief Velocity data
     */
    std::vector<std::deque<double> > velocities_;

    /**
     * @brief current data
     */
    std::vector<std::deque<double> > currents_;

    /**
     * @brief control_buffer_
     */
    std::vector<std::deque<double> > control_buffer_;


    /**
     * @brief time
     */
    std::vector<std::deque<double> > m_time;
    std::chrono::high_resolution_clock::time_point m_start;
    std::vector<std::deque<std::chrono::high_resolution_clock::time_point> > m_time_stamps;


    ActuationLaws m_actuation_law;

};

}  // namespace generic_torque_actuation
