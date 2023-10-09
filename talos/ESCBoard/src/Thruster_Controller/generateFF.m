%use this matlab script to generate coefficents for the thruster control
format compact

EPSILON = 1000000;

FIGURE_RPM_VS_DSHOT = 1;
FIGURE_FORCE_VS_DSHOT = 2;
FIGURE_FORCE_VS_RPM = 3;
FIGURE_DSHOT_VS_RPM = 4;

keepPrompting = 1;
promptNum = 0;
legendNames = {};

figure(FIGURE_RPM_VS_DSHOT);
xlabel("dshot");
ylabel("rpm");
title("rpm vs dshot");
hold on

figure(FIGURE_FORCE_VS_DSHOT);
xlabel("dshot");
ylabel("force");
title("force vs dshot");
hold on

figure(FIGURE_FORCE_VS_RPM);
xlabel("rpm");
ylabel("force");
title("force vs rpm");
hold on

figure(FIGURE_DSHOT_VS_RPM);
xlabel("rpm");
ylabel("dshot");
title("dshot vs rpm");
hold on

while(keepPrompting)
    %fill in points array with points in form of [[rpm, dshot]]
    promptNum = promptNum + 1;
    filename = uigetfile(".csv");
    legendNames = [legendNames, filename];
    
    dshotTable = readtable(filename);
    dshotValues = dshotTable{:, 1};
    rpms = dshotTable{:, 2};
    forces = dshotTable{:, 3};
    
    analyzeDshotArray(FIGURE_RPM_VS_DSHOT, [dshotValues, rpms]);
    analyzeDshotArray(FIGURE_FORCE_VS_DSHOT, [dshotValues, forces]);
    analyzeDshotArray(FIGURE_FORCE_VS_RPM, [rpms, forces]);
    
    coeffs = analyzeDshotArray(FIGURE_DSHOT_VS_RPM, [rpms, dshotValues]);
    fprintf("Spline Coefficients: {");
    for i = 1:height(coeffs)
        fprintf("{%d,%d,%d,%d},", ...
            round(coeffs(i, 1) * EPSILON), ...
            round(coeffs(i, 2) * EPSILON), ...
            round(coeffs(i, 3) * EPSILON), ...
            round(coeffs(i, 4) * EPSILON));
    end
    fprintf("}\n");
    
    fprintf("RPMs: {");
    for i = 2:height(rpms) - 1
        fprintf("%d,", round(rpms(i)));
    end
    fprintf("}\n\n");
    
    keepPrompting = input("Add another csv file? Type 1 for yes and 0 for no: ");
end

figure(FIGURE_RPM_VS_DSHOT);
legend(legendNames);

figure(FIGURE_FORCE_VS_DSHOT);
legend(legendNames);

figure(FIGURE_FORCE_VS_RPM);
legend(legendNames);

figure(FIGURE_DSHOT_VS_RPM);
legend(legendNames);

function coefficents = analyzeDshotArray(fig, dshotArray)
    %remove duplicate x-values from array
    zeroLocs = diff(dshotArray(:, 1)) == 0;
    dshotArray(zeroLocs, :) = [];

    arraySize = size(dshotArray);
    A = zeros((arraySize(1) - 1) * 4, (arraySize(1) - 1) * 4);
    B = zeros((arraySize(1) - 1) * 4, 1);

    for i = 0:(length(dshotArray) - 2)
        %x val
        xVal1 = dshotArray(i + 1, 1);
        xVal2 = dshotArray(i + 2, 1);

        %apply 1st point continuity eq
       A(i * 4 + 1, i * 4 + 1) = xVal1^3;
       A(i * 4 + 1, i * 4 + 2) = xVal1^2;
       A(i * 4 + 1, i * 4 + 3) = xVal1^1;
       A(i * 4 + 1, i * 4 + 4) = 1;

       B(i * 4 + 1) = dshotArray(i + 1, 2);

       %apply 2st point continuity eq
       A(i * 4 + 2, i * 4 + 1) = xVal2^3;
       A(i * 4 + 2, i * 4 + 2) = xVal2^2;
       A(i * 4 + 2, i * 4 + 3) = xVal2^1;
       A(i * 4 + 2, i * 4 + 4) = 1;

       B(i * 4 + 2) = dshotArray(i + 2, 2);

       %if not the first link
       if(i > 0)

           xVal0 = dshotArray(i, 1);

            %apply 1st derivative continuity eq

            %current derivative
            A(i * 4 + 3, i * 4 + 1) = 3 * xVal1^2;
            A(i * 4 + 3, i * 4 + 2) = 2 * xVal1;
            A(i * 4 + 3, i * 4 + 3) = 1;

            A(i * 4 + 3, (i - 1) * 4 + 1) = -3 * xVal1^2;
            A(i * 4 + 3, (i - 1) * 4 + 2) = -2 * xVal1;
            A(i * 4 + 3, (i - 1) * 4 + 3) = -1;

            % apply second derivate continuty eq
            A(i * 4 + 4, i * 4 + 1) = 6 * xVal1;
            A(i * 4 + 4, i * 4 + 2) = 2;

            A(i * 4 + 4, (i - 1) * 4 + 1) = -6 * xVal1;
            A(i * 4 + 4, (i - 1) * 4 + 2) = -2;
       end

    end   

    %apply final two constraints
    % 1st derivative = 0 at xmin

    A(3, 1) = 3 * dshotArray(1,1)^2;
    A(3, 2) = 2 * dshotArray(1,1);
    A(3, 3) = 1;

    %1str derivative = 0 at xmax
    A(4, (length(dshotArray) - 2) * 4 + 1) = 3 * dshotArray(1,1)^2;
    A(4, (length(dshotArray) - 2) * 4 + 2) = 2 * dshotArray(1,1);
    A(4, (length(dshotArray) - 2) * 4 + 3) = 1;
    
    coefficents = transpose(reshape(inv(A) * B, 4, length(dshotArray) - 1));

    %make plot
    points = 10000;
    xVals = linspace(dshotArray(1,1), dshotArray(end,1), points);
    yVals = zeros(1,points);
    for i=1:points
        xValue = xVals(i);

        for j = 1:length(dshotArray) - 1
            if(xValue >= dshotArray(j,1) && xValue <= dshotArray(j + 1,1))
                %evaluate on this j coefficents
                yVals(i) = coefficents(j, 1) * xValue^3 + coefficents(j, 2) * xValue^2 + coefficents(j, 3) * xValue + coefficents(j, 4);
            end
        end
    end
    
    figure(fig);
    plot(xVals, yVals)
end
