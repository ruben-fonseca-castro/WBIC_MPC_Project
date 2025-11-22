function setup_paths(person_select)
    % This function sets up the Java and LCM paths based on the user.

    if strcmp(person_select,'David')
        % --- Ubuntu / Linux Paths ---
        disp('Linux/Unix system detected.');
        lcm_lib_path = '/home/davidho/miniconda3/envs/arcpy/lib/python3.9/site-packages/share/java/lcm.jar';
        project_dir = fullfile(getenv('HOME'), 'arc-bridge');

    elseif strcmp(person_select,'Ruben')
        % --- Windows Paths ---
        disp('Windows system detected.');
        lcm_lib_path = 'C:\Users\Ruben\miniconda3\envs\arcpy\lib\site-packages\share\java\lcm.jar';
        project_dir = 'C:\Users\Ruben\Documents_COPY\LeggedRobots\Final_Project\arc-bridge';
    
    elseif strcmp(person_select,'Ruben_Linux')
        % --- Ubuntu / Linux Paths ---
        disp('Linux/Unix system detected.');
        lcm_lib_path = '/home/rubenafc/miniconda3/envs/arcpy/lib/python3.9/site-packages/share/java/lcm.jar';
        project_dir = '/home/rubenafc/Documents/LRC_Final_Project/arc-bridge';

    elseif strcmp(person_select,'Pranav')
        % --- Windows Paths ---
        disp('Windows system detected.');
        lcm_lib_path = 'C:\path\to\your\lcm.jar';
        project_dir = fullfile(getenv('USERPROFILE'), 'Documents', 'arc-bridge');
    
    else
        error('Invalid person_select. Check setup_paths.m');
    end

    % --- Add the paths to the Java Classpath ---
    
    a1_types_path = fullfile(project_dir, 'arc_bridge', 'lcm_msgs', 'lcm_msgs.jar');
    
    % current_path = javaclasspath('-dynamic');
    current_path = javaclasspath;

    % Check and add main LCM library
    if ~any(endsWith(current_path, lcm_lib_path))
        javaaddpath(lcm_lib_path);
    end

    % Check and add A1 message types
    if ~any(endsWith(current_path, a1_types_path))
        javaaddpath(a1_types_path);
    end
end