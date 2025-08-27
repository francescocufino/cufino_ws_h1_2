close all, clc, clear all

% Define base path
base_path = './';
timestep = 0.01;  % seconds


% File names and plot titles
files = {
    'joint_positions_cmd.csv',      'Joint Positions Command';
    'joint_positions_actual.csv',   'Joint Positions Actual';
    'left_ee_cmd.csv',              'Left End Effector Command';
    'right_ee_cmd.csv',             'Right End Effector Command';
    'left_ee_actual.csv',           'Left End Effector Actual';
    'right_ee_actual.csv',          'Right End Effector Actual';
    'left_twist_ee_cmd.csv',        'Left Twist EE Command';
    'right_twist_ee_cmd.csv',       'Right Twist EE Command'
    'force_ee.csv',                 'End-effector forces'
        'est_torques.csv',           'est_torques'
    'gravity_torques.csv',           'gravity_torques'

};

% Loop over each file
for i = 1:size(files, 1)
    file_path = fullfile(base_path, files{i,1});
    plot_title = files{i,2};

    % Load CSV data
    data = readmatrix(file_path);

    % Create time vector using timestep
    t = (0:size(data, 1)-1)' * timestep;

    % Number of dimensions (columns)
    num_dims = size(data, 2);

    % Create figure
    figure('Name', plot_title, 'NumberTitle', 'off');
    
    for j = 1:num_dims
        subplot(num_dims, 1, j);
        plot(t, data(:, j), 'LineWidth', 1.2);
        ylabel(sprintf('Dim %d', j));
        grid on;

        if j == 1
            title(plot_title, 'Interpreter', 'none');
        end
        if j == num_dims
            xlabel('Time (s)');
        end
    end
end
