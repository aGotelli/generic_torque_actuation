#include "generic_torque_actuation/actuators_interface.hpp"


namespace generic_torque_actuation
{

ActuatorsInterface::ActuatorsInterface(ActuationLaws t_actuations_laws,
                                       std::shared_ptr<bool> t_start_recording,
                                       std::vector<int> t_premultipliers)
    : m_actuations_laws( t_actuations_laws )
{

//    std::cout << "Entering constructor brackets" << std::endl;
    if(t_premultipliers.empty())
        t_premultipliers = std::vector<int>(t_actuations_laws.size(), 1);


//    std::cout << "Motor list size : " << m_motor_list.size() << std::endl;

//    for(const auto &motor : m_motor_list)
//        motor->print();
//    std::cout.flush();

//    std::cout << "Creating controller" << std::endl;
    controller =
        std::make_shared<generic_torque_actuation::GenericTorqueControl>(m_motor_list,
                                                                         m_actuations_laws,
                                                                        t_premultipliers);

//    std::cout << "Controller created" << std::endl;

    controller->m_start_recording = t_start_recording;
    rt_printf("controllers are set up \n");

}


//ActuatorsInterface::ActuatorsInterface(ActuationLaws t_actuations_laws,
//                                       std::shared_ptr<bool> t_start_recording,
//                                       std::shared_ptr<std::vector<double>> t_forces,
//                                       const double t_Kp,
//                                       const double t_Ki,
//                                       const double t_Kd)
//    : m_actuations_laws( t_actuations_laws )
//{
//    controller =
//        std::make_shared<generic_torque_actuation::PIDTorqueControl>(m_motor_list,
//                                                                     m_actuations_laws,
//                                                                     t_forces,
//                                                                     t_Kp,
//                                                                     t_Ki,
//                                                                     t_Kd);



//    controller->m_start_recording = t_start_recording;
//    rt_printf("controllers are set up \n");

//}






void ActuatorsInterface::getSamplesData(YAML::Node &t_actuator_node, Eigen::MatrixXd &t_actuator_data)
{



    controller->stop_loop_ = true;
    const unsigned int rows_per_actuator = 5;
    const auto number_of_rows = m_number_of_motors * rows_per_actuator;
    t_actuator_data = Eigen::MatrixXd(number_of_rows, controller->currents_[0].size());


    unsigned int start_row = 0;
    for(unsigned int motor=0; motor<m_number_of_motors; motor++){
        start_row = motor*rows_per_actuator;

        for(unsigned int i=0; i<controller->currents_[0].size(); i++){

            const auto relative_time = std::chrono::high_resolution_clock::duration(controller->m_time_stamps[motor][i] - controller->m_start).count()/1e9;

            t_actuator_data.block(start_row, i, rows_per_actuator, 1) <<    relative_time,
                                                                            controller->currents_[motor][i],
                                                                            controller->control_buffer_[motor][i],
                                                                            controller->encoders_[motor][i],
                                                                            controller->velocities_[motor][i];
        }
    }

    t_actuator_node["data_storage"]["number_of_samples"] = controller->currents_[0].size();
    t_actuator_node["data_storage"]["rows_per_actuator"] = rows_per_actuator;
    t_actuator_node["data_storage"]["number_of_actuators"] = m_number_of_motors;

    t_actuator_node["data_storage"]["data_order"].push_back("relative_time");
    t_actuator_node["data_storage"]["data_order"].push_back("actual_current");
    t_actuator_node["data_storage"]["data_order"].push_back("desired_current");
    t_actuator_node["data_storage"]["data_order"].push_back("encoder_angle");
    t_actuator_node["data_storage"]["data_order"].push_back("encoder_velocity");


}















}   //  namespace generic_torque_actuation

