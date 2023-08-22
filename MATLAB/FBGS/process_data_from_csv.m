close all
clear all
clc


time_stamp_linewidth = 2.0;
font_size = 16;
legend_font_size = 16;
axis_ticks_font_size = 16;

save_figure_format = ".png";


Hz = 100;
path = "../../data/" + num2str(Hz) + "Hz/two_sensors/";


samples_data = load("../../data/100Hz/prova_new_interface/FBGS_data.csv");


samples_numbers = samples_data(1,:);
time_stamps = samples_data(2,:);

relative_samples_numbers = samples_numbers - samples_numbers(1);
relative_time_stamps = time_stamps - time_stamps(1);


sample_number_differences = relative_samples_numbers(2:end) - relative_samples_numbers(1:end-1);



name = "Samples number";
fig = figure("Name", name);
plot(relative_samples_numbers)
hold on
title(name)
ylabel("Index")
xlabel("Sample number")
filename = "Samples_number";
saveas(fig, path + filename + save_figure_format)
saveas(fig, path + filename + ".fig")


name = "Samples number differences";
fig = figure("Name", name);
plot(sample_number_differences)
hold on
title(name)
ylabel("Index")
xlabel("Sample number_differences")
filename = "Samples_number";
saveas(fig, path + filename + save_figure_format)
saveas(fig, path + filename + ".fig")





number_of_sensors = samples_data(3,1);


sensors_num_shape_points = samples_data(4:(4+number_of_sensors-1), 1);




for sensor=1:number_of_sensors

    num_shape_points = sensors_num_shape_points(sensor);


    first = 3 + number_of_sensors + sensor + (sensor-1)*num_shape_points*4;
    last = first + num_shape_points - 1;

    arch_length = samples_data(first:last, 1);
    
    first = last+1;
    last  = first + 3*num_shape_points - 1;
    shapes_vectors = samples_data(first:last, :);






    shapes_vectors_difference = shapes_vectors(:, 2:end) - shapes_vectors(:, 1:end-1);
    
    shapes_vectors_velocities    = shapes_vectors_difference*Hz;
    shapes_vectors_accelerations = shapes_vectors_difference*Hz*Hz;
    
    
    
    
    name = "Rod " + int2str(sensor) + " tip positions";
    fig = figure("Name", name);
    plot(shapes_vectors(num_shape_points, :), '-r', 'DisplayName', 'x')
    hold on
    plot(shapes_vectors(2*num_shape_points, :), '-b', 'DisplayName', 'y')
    plot(shapes_vectors(3*num_shape_points, :), '-g', 'DisplayName', 'z')
    legend('Location','best')
    title(name)
    ylabel("Position [m]")
    xlabel("Time [s]")
    filename = "Rod_" + int2str(sensor) + "_tip_positions";
    saveas(fig, path + filename + save_figure_format)
    saveas(fig, path + filename + ".fig")
    
    
    name = "Rod " + int2str(sensor) + " tip velocities";
    fig = figure("Name", name);
    plot(shapes_vectors_velocities(num_shape_points, :), '-r', 'DisplayName', 'x')
    hold on
    plot(shapes_vectors_velocities(2*num_shape_points, :), '-b', 'DisplayName', 'y')
    plot(shapes_vectors_velocities(3*num_shape_points, :), '-g', 'DisplayName', 'z')
    legend('Location','best')
    title(name)
    ylabel("Velocities [m/s]")
    xlabel("Time [s]")
    filename = "Rod_" + int2str(sensor) + "_tip_velocities";
    saveas(fig, path + filename + save_figure_format)
    saveas(fig, path + filename + ".fig")
    
    name = "Rod " + int2str(sensor) + " tip accelerations";
    fig = figure("Name", name);
    plot(shapes_vectors_accelerations(num_shape_points, :), '-r', 'DisplayName', 'x')
    hold on
    plot(shapes_vectors_accelerations(2*num_shape_points, :), '-b', 'DisplayName', 'y')
    plot(shapes_vectors_accelerations(3*num_shape_points, :), '-g', 'DisplayName', 'z')
    legend('Location','best')
    title(name)
    ylabel("Accelerations [m/s^2]")
    xlabel("Time [s]")
    filename = "Rod_" + int2str(sensor) + "_tip_accelerations";
    saveas(fig, path + filename + save_figure_format)
    saveas(fig, path + filename + ".fig")



end





