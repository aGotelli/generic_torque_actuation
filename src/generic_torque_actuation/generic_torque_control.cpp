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
//    std::cout << "Controller Contructor" << std::endl;

    encoders_.clear();
    velocities_.clear();
    currents_.clear();
    control_buffer_.clear();
    m_time.clear();

//    std::cout << "Controller clear vectors" << std::endl;

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

//    std::cout << "Controller finished allocation loop" << std::endl;
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


    unsigned int count = 0;
    unsigned int printing_once_every = 100;


    real_time_tools::Spinner spinner;
    const double dt = 0.001;
    spinner.set_period( dt );  // here we spin every 1ms

    std::cout << "motor_list_.size() : " << motor_list_.size() << "\n\n";



    auto current = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::duration elapsed_time;
    m_start = std::chrono::high_resolution_clock::now();
    while (!stop_loop_)
    {

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



            if(*m_start_recording){
                encoders_[i].push_back(actual_position);
                velocities_[i].push_back(actual_velocity);
                currents_[i].push_back(actual_current);
                control_buffer_[i].push_back(desired_current);
                m_time[i].push_back(local_time);
                m_time_stamps[i].push_back( std::chrono::high_resolution_clock::now() );
            }


        }  // endfor

        // we sleep here 1ms.
        spinner.spin();


        if(count >= printing_once_every){

            std::string log = "Time : " + std::to_string(local_time) + "  ";

            for(unsigned int i=0; i<motor_list_.size(); i++)
                log += "Motor " + std::to_string(i+1) + " currents : " + std::to_string(control_buffer_[i].at(control_buffer_[i].size() - 1)) +
                       ", " + std::to_string(currents_[i].at(currents_[i].size() - 1)) + "  ";

            rt_printf("%s \n", log.c_str());
            count = 0;
        } else count ++;


    }      // endwhile
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

}




}  // namespace generic_torque_actuation
