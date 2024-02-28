function WalkingManSimulation(K, k, control_Interval)
    % Initialize variables
    theta = 0;
    omega = 0;
    del_t = 0.001;
    w = 10;
    horizon_len = 10;
    xcom_prev = 0;
    x_com_vel = 0;
    ycom_prev = 1;
    y_com_vel = 0;
    cop_idx = 1;
    xcop_prev = 0;
    xcop = 0;
    left = 1;

    % Set up the figure and axes for visualization
    figure;
    axis(gca, 'equal');
    axis([-2 10 -2 2]);
    grid on;

    % Simulation loop
    for h = 1:(horizon_len * 1000)
        if mod(h, floor(control_Interval / del_t)) == 1
            xcop_prev = xcop;
            xcop = K(cop_idx).value * [xcom_prev; x_com_vel] + k(cop_idx).value;
            cop_idx = cop_idx + 1;
            left = mod(left + 1, 2); 
        end

        theta = asin((-xcop + xcom_prev));
        ang_acc = w * sin(theta);
        x_com_acc = ang_acc * cos(theta);
        x_com_vel = x_com_vel + x_com_acc * del_t;
        x_com_curr = xcom_prev + x_com_vel * del_t + 0.5 * x_com_acc * (del_t ^ 2);
        step_size = x_com_curr - xcop;
        y_com_curr = abs(sqrt(1 - step_size * step_size));

        % Update visualization
        if left
            leg1 = line([xcop x_com_curr], [0 y_com_curr], 'Color', 'red');
            leg2 = line([xcop_prev x_com_curr], [0 y_com_curr], 'Color', 'blue');
        else
            leg1 = line([xcop x_com_curr], [0 y_com_curr], 'Color', 'blue');
            leg2 = line([xcop_prev x_com_curr], [0 y_com_curr], 'Color', 'red');
        end

        xcom_prev = x_com_curr;
        ycom_prev = y_com_curr;

        % Pause to control simulation speed
        pause(del_t);

        % Delete previous lines to update animation
        delete(leg1);
        delete(leg2);
    end
end
