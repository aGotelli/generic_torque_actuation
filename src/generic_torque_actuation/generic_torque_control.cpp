#include "generic_torque_actuation/generic_torque_control.hpp"

namespace generic_torque_actuation
{


GenericTorqueControl::GenericTorqueControl(std::vector<SafeMotor_ptr> motor_list,
                                           ActuationLaws t_actuation_law)
    : motor_list_(motor_list),
      m_actuation_law(t_actuation_law)
{
    encoders_.clear();
    velocities_.clear();
    currents_.clear();
    control_buffer_.clear();
    m_time.clear();

    for (unsigned int i = 0; i < motor_list.size(); ++i)
    {
        encoders_.push_back(std::deque<double>());
        currents_.push_back(std::deque<double>());
        velocities_.push_back(std::deque<double>());
        control_buffer_.push_back(std::deque<double>());
        m_time.push_back(std::deque<double>());
        m_time_stamps.push_back(std::deque<std::chrono::high_resolution_clock::time_point>() );

        encoders_.back().clear();
        velocities_.back().clear();
        currents_.back().clear();
        control_buffer_.back().clear();
        m_time.back().clear();
        m_time_stamps.back().clear();
    }
    stop_loop_ = false;
}


GenericTorqueControl::~GenericTorqueControl()
{
    stop_loop_ = true;
    rt_thread_.join();
}


void GenericTorqueControl::start_loop()
{
    rt_thread_.create_realtime_thread(&GenericTorqueControl::loop, this);
}



THREAD_FUNCTION_RETURN_TYPE GenericTorqueControl::loop(void* instance_pointer)
{
    ((GenericTorqueControl*)(instance_pointer))->loop();
    return THREAD_FUNCTION_RETURN_VALUE;
}


void GenericTorqueControl::loop()
{
    const int& blmc_position_index = ::blmc_drivers::MotorInterface::MeasurementIndex::position;
    const int& blmc_velocity_index = ::blmc_drivers::MotorInterface::MeasurementIndex::velocity;
    const int& blmc_current_index = ::blmc_drivers::MotorInterface::MeasurementIndex::current;
    // some data
    double actual_position = 0.0;
    double actual_velocity = 0.0;
    double actual_current = 0.0;
    double local_time = 0.0;

    double desired_current = 0.0;


    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);  // here we spin every 1ms
    real_time_tools::Timer time_logger;
    size_t count = 0;

    std::cout << "motor_list_.size() : " << motor_list_.size() << "\n\n";
    m_start = std::chrono::high_resolution_clock::now();
    while (!stop_loop_)
    {
        local_time = count * 0.001;

        // compute the control
        for (unsigned int i = 0; i < motor_list_.size(); ++i)
        {
            actual_position = motor_list_[i]
                                  ->get_measurement(blmc_position_index)
                                  ->newest_element();
            actual_velocity = motor_list_[i]
                                  ->get_measurement(blmc_velocity_index)
                                  ->newest_element();
            actual_current = motor_list_[i]
                                 ->get_measurement(blmc_current_index)
                                 ->newest_element();


            desired_current = getDesiredCurrent(i, local_time);

            motor_list_[i]->set_current_target(desired_current);
            motor_list_[i]->send_if_input_changed();

            if(*m_start_recording){
                encoders_[i].push_back(actual_position);
                velocities_[i].push_back(actual_velocity);
                currents_[i].push_back(actual_current);
                control_buffer_[i].push_back(desired_current);
                m_time[i].push_back(local_time);
                m_time_stamps[i].push_back( std::chrono::high_resolution_clock::now() );
            }

            // we sleep here 1ms.
            spinner.spin();


            // Printings
            if ((count % 100) == 0)
            {
                rt_printf("sending current: %f\n", desired_current);
                // time_logger.print_statistics();
            }
            ++count;
        }  // endfor
    }      // endwhile
    time_logger.dump_measurements("/home/andrea/Desktop/blmc_drivers/demos/andrea/simple_torque_control_tendon_sinu/test");
}


double GenericTorqueControl::getDesiredCurrent(const unsigned int t_motor_index,
                                               const double &t_local_time)const
{
    double desired_current = m_actuation_law[t_motor_index](t_local_time);

    return desired_current;
}



void GenericTorqueControl::stop_loop()
{
    stop_loop_ = true;
//    // dumping stuff
//    std::string file_name = "/home/andrea/Desktop/blmc_drivers/demos/simple_torque_control_tendon_sinu/sine_torque_xp.csv";
//    try
//    {
//        std::ofstream log_file(file_name, std::ios::binary | std::ios::out);
//        log_file.precision(10);

//        assert(encoders_[0].size() == velocities_[0].size() &&
//               velocities_[0].size() == control_buffer_[0].size() &&
//               control_buffer_[0].size() == currents_[0].size());
//        log_file << "time" << ", " /*<< "encoders" << " " << "velocities" << " "
//                 << "control_buffer" << " "*/ << "currents"
//                 << " ";
//        log_file << std::endl;
//        for (unsigned int j = 0; j < encoders_[0].size(); ++j)
//        {
//            for (unsigned int i = 0; i < encoders_.size(); ++i)
//            {
//                log_file << m_time[i][j] << ", " /*<< encoders_[i][j] << " " << velocities_[i][j] << " "
//                         << control_buffer_[i][j] << " "*/ << currents_[i][j]
//                         << " ";
//            }
//            log_file << std::endl;
//        }

//        log_file.flush();
//        log_file.close();
//    }
//    catch (...)
//    {
//        rt_printf(
//            "fstream Error in dump_tic_tac_measurements(): "
//            "no time measurment saved\n");
//    }

//    rt_printf("dumped the trajectory");
}



double PIDTorqueControl::getDesiredCurrent(const unsigned int t_motor_index,
                                           const double &t_local_time)const
{
    //  Here we compute the current using the real value and thus the PID
    const double desired_tension = m_actuation_law[t_motor_index](t_local_time);

    const double current_tension = m_forces->at(t_motor_index);

    const double error = current_tension - desired_tension;

    //  Proportional term
    const double P = m_Kp*error;

    //  Integral term
    m_integral += error*m_dt;
    const double I = m_Ki*m_integral;

    // Derivative term
    const double D = (error - m_previous_error) / m_dt;


    const double actuation = P+I+D;

    const double tension_to_current = 1.0;



    const double desired_current = actuation * tension_to_current;


    return desired_current;
}


}  // namespace generic_torque_actuation
