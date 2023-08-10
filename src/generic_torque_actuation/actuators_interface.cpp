#include "generic_torque_actuation/actuators_interface.hpp"


namespace generic_torque_actuation
{

ActuatorsInterface::ActuatorsInterface(ActuationLaws t_actuations_laws,
                                       std::shared_ptr<bool> t_start_recording)
    : m_actuations_laws( t_actuations_laws )
{
    controller.m_start_recording = t_start_recording;
    rt_printf("controllers are set up \n");

}



void ActuatorsInterface::getSamplesData(YAML::Node &t_actuator_node, Eigen::MatrixXd &t_actuator_data)
{

    controller.stop_loop_ = true;
    t_actuator_data = Eigen::MatrixXd(5, controller.currents_[0].size());

    for(unsigned int i=0; i<controller.currents_[0].size(); i++){

        const auto relative_time = std::chrono::duration<double, std::milli>(controller.m_time_stamps[0][i] - controller.m_start).count();
        t_actuator_data(0, i) = relative_time;

        t_actuator_data(1, i) = controller.currents_[0][i];

        t_actuator_data(2, i) = controller.control_buffer_[0][i];

        t_actuator_data(3, i) = controller.encoders_[0][i];

        t_actuator_data(4, i) = controller.velocities_[0][i];
    }
}















}   //  namespace generic_torque_actuation

