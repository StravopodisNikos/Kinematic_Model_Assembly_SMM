dxl_bolt_step = 0.005;

x_4_7_10 = -0.05:dxl_bolt_step:0.05;    % quantized assembly params
x_5_8_11 = -0.025:dxl_bolt_step:0.025;
x_6_9_12 = [-1.5708 0 1.5708];

int_x_4_7_10 = 1:size(x_4_7_10,2);      % integer vectors for matching assembly params
int_x_5_8_11 = 1:size(x_5_8_11,2);
int_x_6_9_12 = 1:size(x_6_9_12,2);

% EXAMPLE FOR STRING BOUNDS DEFINITION
%      x1   x2   x3     
LB1 = [1    2    1  ];    
UB1 = [3    3    3  ]; 
%               x4                          x5                                  x6
LB2 = [          1                          1                                   1       ];
UB2 = [int_x_4_7_10(size(x_4_7_10,2))    int_x_5_8_11(size(int_x_5_8_11,2))      int_x_6_9_12(size(int_x_6_9_12,2))];
%               x7                          x8                                  x9
LB3 = LB2;
UB3 = UB2;
%               x10                        x11                                  x12
LB4 = LB2;
UB4 = UB2;
%      x13     x14
LB5 = [0        0];
UB5 = [1        1];
% final:
LB  = horzcat(LB1,LB2,LB3,LB4,LB5);
UB  = horzcat(UB1,UB2,UB3,UB4,UB5);

% * Ston kwdika pou soy eixa dwsei eixa ftiaksei ta strings gia to assembly
% se integers! ** epishs kai tis 2 wwnies pitch gia th synndesh tvn dxl!
% *** Twra allaksa mono tis 3 times gia tis syndeseis tvn pseudo!