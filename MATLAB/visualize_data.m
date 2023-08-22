close all
clear all
clc


time_stamp_linewidth = 2.0;
font_size = 16;
legend_font_size = 16;
axis_ticks_font_size = 16;

save_figure_format = ".png";

experiment_tag = "load_and_sine";


Hz = 100;
path = "../data/simple_benchmark/dual_motor/robot_board/double_sine/";


actuators_data = load(path + "actuator_data.csv");

config_file = yaml.loadFile(path + "configuration_file.yaml");

number_of_actuator = config_file.data_storage.number_of_actuators;
rows_per_actuator = config_file.data_storage.rows_per_actuator;


%%  Select Actuator dataset

elements = [0 rows_per_actuator];

elements = elements + 1;
actuators_Linux_time_stamps = actuators_data(elements, :);
elements = elements + 1;
actuators_current = actuators_data(elements, :);
elements = elements + 1;
actuators_desired_current = actuators_data(elements, :);
elements = elements + 1;

actuators_angles = actuators_data(4, :);
actuators_velocities = actuators_data(5, :);


actuators_Linux_time_stamps = actuators_Linux_time_stamps - actuators_Linux_time_stamps(1);

actuators_Linux_time_stamps = actuators_Linux_time_stamps / 1000.0;



%%  Plot results

name = "Current";
fig = figure("Name", name);

plot(actuators_Linux_time_stamps, actuators_current(1,:), '-r')
hold on
plot(actuators_Linux_time_stamps, actuators_desired_current(1,:), '-k')
plot(actuators_Linux_time_stamps, actuators_current(2,:), '-b')
plot(actuators_Linux_time_stamps, actuators_desired_current(2,:), '-m')
legend('Current 1', 'Desired Current 1', 'Current 2', 'Desired Current 2')
%legend('Location','best')
title(name)
ylabel("Current [A]")
xlabel("Time [s]")
filename = "Current";
saveas(fig, path + filename + save_figure_format)
saveas(fig, path + filename + ".fig")



