%use this matlab script to generate coefficents for the thruster control
format compact

global EPSILON;
EPSILON = 1000000;

FIGURE_RPM_VS_FORCE = 1;
FIGURE_DSHOT_VS_RPM = 2;

keep_prompting = 1;
prompt_num = 0;
legend_names = {};

figure(FIGURE_RPM_VS_FORCE);
xlabel("force");
ylabel("rpm");
title("rpm vs force");
hold on

figure(FIGURE_DSHOT_VS_RPM);
xlabel("rpm");
ylabel("dshot");
title("dshot vs rpm");
hold on

while(keep_prompting)
    %fill in points array with points in form of [[rpm, dshot]]
    prompt_num = prompt_num + 1;
    filename = uigetfile(".csv");
    legend_names = [legend_names, filename, filename + "-fitted"];
    
    dshot_table = sortrows(readtable(filename));
    
    %remove duplicate x-values from array
    dshot_duplicates = diff(dshot_table{:, 1}) == 0;
    rpm_duplicates = diff(dshot_table{:, 2}) == 0;
    dshot_table(dshot_duplicates, :) = [];
    dshot_table(rpm_duplicates, :) = [];
    
    %sort all rows based on increasing dshot
    sortrows(dshot_table, 1);
    
    dshot_values = dshot_table{:, 1};
    rpms = dshot_table{:, 2};
    forces = dshot_table{:, 3};
    
    % Find index where sign changes
    [switch_index,~] = find(dshot_values > 0,1);

    % Seperating dshot data
    neg_dshot = dshot_values(1:switch_index-1);
    pos_dshot = dshot_values(switch_index:end);
    % Seperating rpm data
    neg_rpm = rpms(1:switch_index-1);
    pos_rpm = rpms(switch_index:end);
    % Seperating force data
    neg_force = forces(1:switch_index-1);
    pos_force = forces(switch_index:end);

    if mean(pos_force) < 0
        pos_force = pos_force * -1;
    end

    if mean(neg_force) > 0
        neg_force = neg_force * -1;
    end

    % Base Functions
    % Create X matrix for force curve fit as a function of form C + CX + Ctanh(X) + C(X)^(1/4)
    Xp_force = [ones(length(pos_force),1) pos_force tanh(pos_force) pos_force.^(1/4)];
    Xn_force = [ones(length(neg_force),1) neg_force tanh(neg_force) (-neg_force).^(1/4)];
    % Create X matrix for rpm curve fit as a function of form C + CX + CX^2
    Xp_rpm = [ones(length(pos_rpm),1) pos_rpm pos_rpm.^2];
    Xn_rpm = [ones(length(neg_rpm),1) neg_rpm neg_rpm.^2];

    % Curve Fitting
    % NOTE: B is a matrix of coeffcients

    % Force to RPM curve fit for positive and negative
    [Bp_force, R, Radj] = leastm_lab(Xp_force, pos_rpm);
    [Bn_force, R, Radj] = leastm_lab(Xn_force, neg_rpm);
    % RPM to dshot curve fit for positive and negative
    [Bp_rpm, R, Radj] = leastm_lab(Xp_rpm, pos_dshot);
    [Bn_rpm, R, Radj] = leastm_lab(Xn_rpm, neg_dshot);

    % Plot results
    plot_force = [neg_force; pos_force];
    plot_rpm = [neg_rpm; pos_rpm];
    plot_XBforce = [Xn_force * Bn_force; Xp_force * Bp_force];
    plot_dshot = [neg_dshot; pos_dshot];
    plot_XBrpm = [Xn_rpm * Bn_rpm; Xp_rpm * Bp_rpm];
    
    % Plot Force to RPM curve
    % Plotting data
    figure(FIGURE_RPM_VS_FORCE)
    
%     plot(pos_force,pos_rpm,'.')
    plot(plot_force, plot_rpm, '.')
    % Plotting curve fit
%     plot(pos_force,Xp_force*Bp_force)
    plot(plot_force, plot_XBforce)
    

    % RPM to dshot curve
    figure(FIGURE_DSHOT_VS_RPM)
    % Plotting data
%     plot(pos_rpm,pos_dshot,".");
    plot(plot_rpm, plot_dshot,".");
    % Plotting curve fit
%     plot(pos_rpm,Xp_rpm*Bp_rpm)
    plot(plot_rpm, plot_XBrpm)
    
    %print B coefficients
    printArr("Positive force B", Bp_force);
    printArr("Negative force B", Bn_force);
    printArr("Positive RPM B", Bp_rpm);
    printArr("Negative RPM B", Bn_rpm);

    keep_prompting = input("Add another csv file? Type 1 for yes and 0 for no: ");
end

figure(FIGURE_RPM_VS_FORCE);
legend(legend_names);

figure(FIGURE_DSHOT_VS_RPM);
legend(legend_names);


function printArr(msg, arr)
    global EPSILON
    fprintf("%s: %d", msg, round(real(arr(1)) * EPSILON));
    for i = 2:length(arr)
        fprintf(", %d", round(real(arr(i)) * EPSILON));
    end
    fprintf("\n");
end
