function [totle_time,t_acc,t_uni,t_dec] = get_trapzoid_info(s, v0, vt, a)
%GET_TRAPZOID_INFO 此处显示有关此函数的摘要
%   此处显示详细说明
    totle_time = 0;
    t_acc = 0;
    t_uni = 0;
    t_dec = 0;
    
    s_acc = (vt^2 - v0^2) / (2 * a);
    s_uni = s - 2 * s_acc;
    s_dec = s_acc;
    
    if (s_uni > 0)
        t_acc = (vt - v0) / a;
        t_uni = s_uni / vt;
        t_dec = t_acc;
    else
        s_acc = s / 2;
        s_uni = 0;
        s_dec = s_acc;
        
        new_vt = sqrt(2 * a * s_acc + v0^2);
        t_acc = (new_vt - v0) / a;
        t_uni = 0;
        t_dec = t_acc;
    end
    
    totle_time = t_acc + t_uni + t_dec;
end

