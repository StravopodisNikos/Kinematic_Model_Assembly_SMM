% Theory implementation file for par.7.3 Melchiorri

% 5th order polynomial trajectory implementation
clear;
clc;
close all;

way_pts = [1.5708 -0.84 0.15 -0.7854 1.5708  ;...     %q1
1.5708 1.35 -0.58  0.7854 -1.5708  ;...    %q2
1.5708 2.14 -2.45  1.5708 -1.2456 ];...    %q3
% time steps
t_start = 0; t_finish = 10;  % [sec]
time_pts = linspace(t_start,t_finish,size(way_pts,2))';

dt = 0.001;
time_samples = time_pts(1):dt:time_pts(length(time_pts));
[q_d, dq_d, ddq_d, ~] = quinticpolytraj(way_pts, time_pts, time_samples);

% Fast Fourier Transform(Discrete)
Fddq_d = fft(ddq_d,2);

% Compute the power spectral density
Pyy = abs(Fddq_d(1,:))/size(time_samples,2);
Fs = 1/dt;
f = Fs/size(time_samples,2)*(0:(size(time_samples,2)-1));
figure; plot(f,Pyy(1:(size(time_samples,2)))); hold on;
title('Power spectral density1')
xlabel('Frequency (Hz)')

Pyy = Fddq_d(2,:).*conj(Fddq_d(2,:))/size(time_samples,2);
Fs = 1/dt;
f = Fs/size(time_samples,2)*(0:(size(time_samples,2)-1));
figure; plot(f,Pyy(1:(size(time_samples,2)))); hold on;
title('Power spectral density2')
xlabel('Frequency (Hz)')

Pyy = Fddq_d(3,:).*conj(Fddq_d(3,:))/size(time_samples,2);
Fs = 1/dt;
f = Fs/size(time_samples,2)*(0:(size(time_samples,2)-1));
figure; plot(f,Pyy(1:(size(time_samples,2)))); hold on;
title('Power spectral density3')
xlabel('Frequency (Hz)')