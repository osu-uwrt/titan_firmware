%use this matlab script to generate coefficents for the thruster control

%fill in points array with points in form of [[rpm, dshot]]
dshotArray = [-1, -1; -.5, -.5; 0, 0; 1, 1; 2, 2; 3, 3; 4, 4; 5, 5; 6, 6; 9, 9; 10, 10];

arraySize = size(dshotArray);
A = zeros((arraySize(1) - 1) * 4, (arraySize(1) - 1) * 4);
B = zeros((arraySize(1) - 1) * 4, 1);

for i=0:(length(dshotArray) - 2)
    %x val
    xVal1 = dshotArray(i + 1,1);
    xVal2 = dshotArray(i + 2,1);
   
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


    
end