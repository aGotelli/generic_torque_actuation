#ifndef ACTUATORS_INTERFACE_HPP
#define ACTUATORS_INTERFACE_HPP


#include "generic_torque_control.hpp"

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace generic_torque_actuation
{




class ActuatorsInterface
{
public:
    ActuatorsInterface(ActuationLaws t_actuations_laws,
                       std::shared_ptr<bool> t_start_recording,
                       std::vector<int> t_premultipliers=std::vector<int>());


//    ActuatorsInterface(ActuationLaws t_actuations_laws,
//                       std::shared_ptr<bool> t_start_recording,
//                       std::shared_ptr<std::vector<double>> t_forces,
//                       const double t_Kp,
//                       const double t_Ki,
//                       const double t_Kd);




    void getSamplesData(YAML::Node &t_actuator_node, Eigen::MatrixXd &t_actuator_data);

    ActuationLaws m_actuations_laws;


    unsigned int m_number_of_motors { static_cast<unsigned int>(m_actuations_laws.size()) };


    // First of all one need to initialize the communication with the can bus.
    std::vector<std::shared_ptr<blmc_drivers::CanBus>> m_can_busses { [this]()->std::vector<std::shared_ptr<blmc_drivers::CanBus>>{

        const unsigned int number_of_can_busses = ceil(static_cast<double>(m_number_of_motors) / 2.0f);

        std::vector<std::shared_ptr<blmc_drivers::CanBus>> can_busses(number_of_can_busses);

        for(unsigned int index=0;auto& can_bus : can_busses){
            std::string name = "can" + std::to_string(index++);
            can_bus = std::make_shared<blmc_drivers::CanBus>(name);
        }

        return can_busses;
    }() };

    // Then we create a motor board object that will use the can bus in order
    // communicate between this application and the actual motor board.
    // Important: the blmc motors are alinged during this stage.
    std::vector<std::shared_ptr<blmc_drivers::CanBusMotorBoard>> m_boards { [this]()->std::vector<std::shared_ptr<blmc_drivers::CanBusMotorBoard>>{
        //
        std::vector<std::shared_ptr<blmc_drivers::CanBusMotorBoard>> boards(m_can_busses.size());

        for(unsigned int index=0; auto& board : boards)
            board = std::make_shared<blmc_drivers::CanBusMotorBoard>(m_can_busses[index++]);


        return boards;

    }() };





    std::vector<generic_torque_actuation::SafeMotor_ptr> m_motor_list {
        [this](){
            std::vector<generic_torque_actuation::SafeMotor_ptr> motor_list;

            for(unsigned int i=0; i<m_number_of_motors; i++) {
                unsigned int index = floor(static_cast<double>(i)/2.0f);
                auto motor = std::make_shared<blmc_drivers::SafeMotor>(m_boards[index], i);

                motor_list.push_back(motor);
            }

            return motor_list;
        }()
    };


    std::shared_ptr<generic_torque_actuation::GenericTorqueControl> controller;
};

}   //  namespace generic_torque_actuation

#endif // ACTUATORS_INTERFACE_HPP
