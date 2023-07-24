close all
% clear all
% clc

% Hz = 200;
% path = "../../data/fbgs/" + num2str(Hz) + "Hz/9s/";
% shapes_file = yaml.loadFile("../../data/fbgs/200Hz/9s/simulation_results_9s.yaml");

Hz = 100;
path = "../../data/fbgs/" + num2str(Hz) + "Hz/two_sensors/";
shapes_file = yaml.loadFile("../../data/fbgs/" + num2str(Hz) + "Hz/two_sensors/simulation_results_20s.yaml");


make_video_rod = false;
make_video_tip = false;

time_stamp_linewidth = 2.0;
font_size = 16;
legend_font_size = 16;
axis_ticks_font_size = 16;

save_figure_format = ".png";



% number_of_snapshots = shapes_file.number_of_snapshots;
number_of_snapshots = shapes_file.measurements.number_of_snapshots;
number_of_sensors = shapes_file.measurements.number_of_sensors;


%   get matrices dimensons
rod_shapes_stack = [];

rods_shapes_stack = [];


%   get matrices dimensons
rod_shapes_stack = [];

rods_shapes_stack = [];


sensors = shapes_file.sensors;

for it_t=1:number_of_snapshots
    
    sensor_t = sensors{it_t};
    for sensor_number=1:number_of_sensors
        sensor = sensor_t(sensor_number);
        sensor = sensor{1};

        shape = sensor.shape;

        number_of_points = shape.matrix.rows;
    
        rod_shape_vector = cell2mat( shape.matrix.data );
        
        rod_shape = zeros(number_of_points, 3);

        begin = sensor_number*3 + 1;
        last = begin + 2;
        for i=1:number_of_points
            rod_shape(i, 1) = rod_shape_vector(i + 0 * number_of_points);
            rod_shape(i, 2) = rod_shape_vector(i + 1 * number_of_points);
            rod_shape(i, 3) = rod_shape_vector(i + 2 * number_of_points);
            
            rod_shapes_stack(it_t, :, begin:last) = rod_shape;
        end
    end
end



if make_video_rod

    for it_t=1:number_of_snapshots
    
    
        % get the rod shape
        rod_shape = rod_shapes_stack(it_t, :, :);
    


        x = rod_shape(1, :, 1);
        y = rod_shape(1, :, 3);
    
        figure(1)
        plot(x, y, '-b', 'LineWidth', 2)
        hold on


        xlabel('m') 
        ylabel('m') 

    
    
        grid on
        hold off
        axis equal
        xlim([-0.1 0.3])
        ylim([-0.2, 0.2])

        time = it_t/Hz;
        title(['time = ' num2str( time ) 's'])

        drawnow
    
    end

end



for sensor_number=1:number_of_sensors
    begin = sensor_number*3 + 1;
    last = begin + 2;

    tip_pos = rod_shapes_stack(2:end, :, begin:last);
    tip_pos_pre = rod_shapes_stack(1:end-1, :, begin:last);

    tip_pos_diff = tip_pos - tip_pos_pre;

    x_diff = tip_pos_diff(1);
    y_diff = tip_pos_diff(2);
    z_diff = tip_pos_diff(3);
    
    dt = 1/Hz;
    
    rod_velocitites = tip_pos_diff/dt;
    rod_acelerations = tip_pos_diff/dt^2;

    
    t_end = dt*number_of_snapshots;
    time_steps = linspace(0, t_end, number_of_snapshots);
    
    rows = size(rod_shapes_stack, 1);
    cols = 3;
    tip_positions = reshape(rod_shapes_stack(:, end, begin:last), [rows, cols]);
    name = "Rod " + int2str(sensor_number) + " tip positions";
    fig = figure("Name", name);
    plot(time_steps, tip_positions(:, 1), '-r', 'DisplayName', 'x')
    hold on
    plot(time_steps, tip_positions(:, 2), '-b', 'DisplayName', 'y')
    plot(time_steps, tip_positions(:, 3), '-g', 'DisplayName', 'z')
    legend('Location','best')
    title(name)
    ylabel("Position [m]")
    xlabel("Time [s]")
    filename = "Rod_" + int2str(sensor_number) + "_tip_positions";
    saveas(fig, path + filename + save_figure_format)
    saveas(fig, path + filename + ".fig")
    
    
    name = "Rod " + int2str(sensor_number) + " tip velocities";
    fig = figure("Name", name);
    plot(time_steps(2:end), rod_velocitites(:, end, 1), '-r', 'DisplayName', 'x')
    hold on
    plot(time_steps(2:end), rod_velocitites(:, end, 2), '-b', 'DisplayName', 'y')
    plot(time_steps(2:end), rod_velocitites(:, end, 3), '-g', 'DisplayName', 'z')
    legend('Location','best')
    title(name)
    ylabel("Velocities [m/s]")
    xlabel("Time [s]")
    filename = "Rod_" + int2str(sensor_number) + "_tip_velocities";
    saveas(fig, path + filename + save_figure_format)
    saveas(fig, path + filename + ".fig")
    
    name = "Rod " + int2str(sensor_number) + " tip accelerations";
    fig = figure("Name", "Rod tip accelerations");
    plot(time_steps(2:end), rod_acelerations(:, end, 1), '-r', 'DisplayName', 'x')
    hold on
    plot(time_steps(2:end), rod_acelerations(:, end, 2), '-b', 'DisplayName', 'y')
    plot(time_steps(2:end), rod_acelerations(:, end, 3), '-g', 'DisplayName', 'z')
    legend('Location','best')
    title(name)
    ylabel("Accelerations [m/s^2]")
    xlabel("Time [s]")
    filename = "Rod_" + int2str(sensor_number) + "_tip_accelerations";
    saveas(fig, path + filename + save_figure_format)
    saveas(fig, path + filename + ".fig")

end

if make_video_tip

    for it_t=1:number_of_snapshots
    
    
        % get the rod shape
        rod_tip = rod_shapes_stack(it_t, end, :);
    


        
    
        figure(1)
        plot3(rod_tip(1), rod_tip(2), rod_tip(3), '*r', 'MarkerSize', 3)
        hold on


        xlabel('m') 
        ylabel('m') 

    
    
        grid on
        hold off
        axis equal
        xlim([-0.1 0.3])
        ylim([-0.2, 0.2])

        time = it_t/Hz;
        title(['time = ' num2str( time ) 's'])

        drawnow
    
    end

end