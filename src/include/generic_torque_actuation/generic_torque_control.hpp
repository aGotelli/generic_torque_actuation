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
                         ActuationLaws t_actuation_law,
                         std::vector<int> t_premultipliers);

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
    virtual void loop();

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


    std::vector<int> m_premultipliers;


protected:

    virtual double getDesiredCurrent(const unsigned int t_motor_index,
                                     const double &t_local_time)const;

};




//struct PIDTorqueControl : public GenericTorqueControl {

//    PIDTorqueControl()=default;


//    PIDTorqueControl(std::vector<SafeMotor_ptr> motor_list,
//                     ActuationLaws t_actuation_law,
//                     std::shared_ptr<std::vector<double>> t_forces,
//                     const double t_Kp,
//                     const double t_Ki,
//                     const double t_Kd)
//        : GenericTorqueControl(motor_list,
//                               t_actuation_law),
//        m_forces(t_forces),
//        m_Kp(t_Kp),
//        m_Ki(t_Ki),
//        m_Kd(t_Kd)
//    {
//        std::cout << "Initialised PID control with \nKp: " << m_Kp << "\nKi: " << m_Ki << "\nKd: " << m_Kd << "\n\n";
//        std::cout.flush();
//    }


//    virtual double getDesiredCurrent(const unsigned int t_motor_index,
//                                     const double &t_local_time)const override;



//    std::shared_ptr<std::vector<double>> m_forces;

//    double m_dt { 0.001 };
//    mutable double m_integral { 0.0 };
//    mutable double m_previous_error { 0.0 };

//    double m_Kp { 0.0 };
//    double m_Ki { 0.0 };
//    double m_Kd { 0.0 };




//};

}  // namespace generic_torque_actuation
