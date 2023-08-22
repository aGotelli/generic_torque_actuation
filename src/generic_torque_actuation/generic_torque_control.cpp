#include "generic_torque_actuation/generic_torque_control.hpp"



#include <iomanip>
#include <sstream>


namespace generic_torque_actuation
{


GenericTorqueControl::GenericTorqueControl(std::vector<SafeMotor_ptr> motor_list,
                                           ActuationLaws t_actuation_law,
                                           std::vector<int> t_premultipliers)
    : motor_list_(motor_list),
      m_actuation_law(t_actuation_law),
      m_premultipliers(t_premultipliers)
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
    const double dt = 0.0005;
    spinner.set_period( dt );  // here we spin every 1ms
    real_time_tools::Timer time_logger;
    size_t count = 0;

    std::cout << "motor_list_.size() : " << motor_list_.size() << "\n\n";

    const std::string sent_prefix = "Sent currents : ";
    const std::string read_prefix = "Actual currents : ";
    std::stringstream message;
    std::stringstream consol_log;


    auto current = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::duration elapsed_time;
    m_start = std::chrono::high_resolution_clock::now();
    while (!stop_loop_)
    {


        message.clear();
        consol_log.clear();


        // compute the control
        for (unsigned int i = 0; i < motor_list_.size(); i++)
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

            current = std::chrono::high_resolution_clock::now();
            elapsed_time = current - m_start;
            local_time = elapsed_time.count() / 1e9;
            desired_current = m_premultipliers[i] * getDesiredCurrent(i, local_time);


            motor_list_[i]->set_current_target(desired_current);
            motor_list_[i]->send_if_input_changed();

            // we sleep here 1ms.
            spinner.spin();

            if(*m_start_recording){
                encoders_[i].push_back(actual_position);
                velocities_[i].push_back(actual_velocity);
                currents_[i].push_back(actual_current);
                control_buffer_[i].push_back(desired_current);
                m_time[i].push_back(local_time);
                m_time_stamps[i].push_back( std::chrono::high_resolution_clock::now() );
            }



//            message << std::fixed << std::setprecision(4) << desired_current << " " << actual_current << "   ";

        }  // endfor





        if ((count++ % 100) == 0)
        {
            const unsigned int index = count -1;
//            consol_log << std::fixed << std::setprecision(4) << "Time : " << local_time /*<< message.str()*/ << "\n";
            rt_printf("local time : %f  Motor 1 : %f  %f    Motor 2 : %f  %f \n",
                      local_time,
                      control_buffer_[0][index],
                      currents_[0][index],
                      control_buffer_[1][index],
                      currents_[0][index]);

        }


    }      // endwhile
    //time_logger.dump_measurements("/home/andrea/Desktop/blmc_drivers/demos/andrea/simple_torque_control_tendon_sinu/test");
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



//double PIDTorqueControl::getDesiredCurrent(const unsigned int t_motor_index,
//                                           const double &t_local_time)const
//{

//    //  Here we compute the current using the real value and thus the PID
//    const double desired_tension = m_actuation_law[t_motor_index](t_local_time);
////    rt_printf("desired_tension : %f \n", desired_tension);

//    const double current_tension = m_forces->at(t_motor_index);
////    rt_printf("current_tension : %f \n", current_tension);

//    if(current_tension < 0.001)
//        return 0;

//    const double static_offset = 0.5;

//    const double corrected_tension = current_tension - static_offset;
////    rt_printf("corrected_tension : %f \n", corrected_tension);

//    const double error = corrected_tension - desired_tension;
////    rt_printf("error : %f \n", error);

//    //  Proportional term
//    const double P = m_Kp*error;
////    rt_printf("P : %f \n", P);

//    //  Integral term
//    m_integral += error*m_dt;
//    const double I = m_Ki*m_integral;
////    rt_printf("I : %f \n", I);

//    // Derivative term
//    const double D = m_Kd*(error - m_previous_error) / m_dt;
//    m_previous_error = error;
////    rt_printf("D : %f \n", D);


//    const double actuation = P+I+D;

//    const double tension_to_current = -1.0;



//    const double desired_current = actuation * tension_to_current;
////    rt_printf("Desired current : %f \n\n", desired_current);


//    return desired_current;
//}


}  // namespace generic_torque_actuation
