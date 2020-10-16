function [gst0] = build_tool_frame(g_s_m_i)
% adds default tf frame from dynamixel moving frame to tcp  

g_last_active_TOOL =    [   1.0000    0.0000    0.0000         0; ...
                            0.0000    1.0000    0.0000    0.0500; ...
                            0.0000    0.0000    1.0000   -0.0000; ...
                                 0         0         0    1.0000];

gst0 = g_s_m_i * g_last_active_TOOL;
end