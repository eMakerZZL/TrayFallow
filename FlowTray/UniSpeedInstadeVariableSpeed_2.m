close all;
clear;
clc;

p0 = [0 0 0];
p1 = [1000 2000 500];

delta_x_f = p1(1) - p0(1);
delta_y_f = p1(2) - p0(2);
delta_z_f = p1(3) - p0(3);

v0_f = 20;
vt_f = 350;
vc_f = 200;
 a_f = 1500;

s_stable = sqrt(delta_x_f^2 + delta_y_f^2 + delta_z_f^2);

px_stable = delta_x_f / s_stable;
py_stable = delta_y_f / s_stable;
pz_stable = delta_z_f / s_stable;

vx0_stable = v0_f * px_stable;
vy0_stable = v0_f * py_stable;
vz0_stable = v0_f * pz_stable;

vxt_stable = vt_f * px_stable;
vyt_stable = vt_f * py_stable;
vzt_stable = vt_f * pz_stable;

ax_stable = a_f * px_stable;
ay_stable = a_f * py_stable;
az_stable = a_f * pz_stable;

if     (delta_x_f > 0) 
    [t,t_acc,t_uni,t_dec] = get_trapzoid_info(delta_x_f, vx0_stable, vxt_stable, ax_stable);
elseif (delta_y_f > 0)
    [t,t_acc,t_uni,t_dec] = get_trapzoid_info(delta_y_f, vy0_stable, vyt_stable, ay_stable);
elseif (delta_z_f > 0)
    [t,t_acc,t_uni,t_dec] = get_trapzoid_info(delta_z_f, vz0_stable, vyt_stable, az_stable);
else
    error('s must a positive number!');
end

delta_t = 0.001;
time = 0 : delta_t : t;
time_acc = time(time < t_acc);
time_uni = time(time >= t_acc & time <= (t_acc + t_uni));
time_dec = time(time > (t_acc + t_uni));

c_time_acc = time_acc;
c_time_uni = time_uni - time_acc(end);
c_time_dec = time_dec - time_uni(end);

T_acc = c_time_acc(end);
T_uni = c_time_uni(end);
T_dec = c_time_dec(end);

%未叠加传送带速度的X轴速度
vx_stable_acc_i = vx0_stable + ax_stable * c_time_acc;
vx_stable_uni_i = vx0_stable + ax_stable * T_acc * ones(size(c_time_uni));
vx_stable_dec_i = vx0_stable + ax_stable * T_acc - ax_stable * c_time_acc;
vx_stable = [vx_stable_acc_i, vx_stable_uni_i, vx_stable_dec_i];

%未叠加传送带速度的Y轴速度
vy_stable_acc_i = vy0_stable + ay_stable * c_time_acc;
vy_stable_uni_i = vy0_stable + ay_stable * T_acc * ones(size(c_time_uni));
vy_stable_dec_i = vy0_stable + ay_stable * T_acc - ay_stable * c_time_acc;
vy_stable = [vy_stable_acc_i, vy_stable_uni_i, vy_stable_dec_i];

%未叠加传送带速度的Z轴速度
vz_stable_acc_i = vz0_stable + az_stable * c_time_acc;
vz_stable_uni_i = vz0_stable + az_stable * T_acc * ones(size(c_time_uni));
vz_stable_dec_i = vz0_stable + az_stable * T_acc - az_stable * c_time_acc;
vz_stable = [vz_stable_acc_i, vz_stable_uni_i, vz_stable_dec_i];

%未叠加传送带速度的X轴位移
sx_stable_acc_i = vx0_stable * c_time_acc + 1/2 * ax_stable * c_time_acc.^2;
sx_stable_uni_i = vx0_stable * c_time_uni + ax_stable * T_acc * c_time_uni + sx_stable_acc_i(end);
sx_stable_dec_i = vx0_stable * c_time_dec + ax_stable * T_acc * c_time_dec - 1/2 * ax_stable * c_time_dec.^2 + sx_stable_uni_i(end);
sx_stable = [sx_stable_acc_i, sx_stable_uni_i, sx_stable_dec_i];

%未叠加传送带速度的Y轴位移
sy_stable_acc_i = vy0_stable * c_time_acc + 1/2 * ay_stable * c_time_acc.^2;
sy_stable_uni_i = vy0_stable * c_time_uni + ay_stable * T_acc * c_time_uni + sy_stable_acc_i(end);
sy_stable_dec_i = vy0_stable * c_time_dec + ay_stable * T_acc * c_time_dec - 1/2 * ay_stable * c_time_dec.^2 + sy_stable_uni_i(end);
sy_stable = [sy_stable_acc_i, sy_stable_uni_i, sy_stable_dec_i];

%未叠加传送带速度的Z轴位移
sz_stable_acc_i = vz0_stable * c_time_acc + 1/2 * az_stable * c_time_acc.^2;
sz_stable_uni_i = vz0_stable * c_time_uni + az_stable * T_acc * c_time_uni + sz_stable_acc_i(end);
sz_stable_dec_i = vz0_stable * c_time_dec + az_stable * T_acc * c_time_dec - 1/2 * az_stable * c_time_dec.^2 + sz_stable_uni_i(end);
sz_stable = [sz_stable_acc_i, sz_stable_uni_i, sz_stable_dec_i];

%Y,Z轴速度由X轴速度推出
vx_move = vx_stable + vc_f;

sx_move = sx_stable + vc_f * time;
sy_move = sy_stable;
sz_move = sz_stable;

s_move = sqrt(sx_move.^2 + sy_move.^2 + sz_move.^2);

px_move = (sx_move(2:end) - sx_move(1:end-1)) ./ (s_move(2:end) - s_move(1:end-1));
py_move = (sy_move(2:end) - sy_move(1:end-1)) ./ (s_move(2:end) - s_move(1:end-1));
pz_move = (sz_move(2:end) - sz_move(1:end-1)) ./ (s_move(2:end) - s_move(1:end-1));

v_move = vx_move(1:end - 1) ./ px_move;
vy_move = v_move .* py_move;
vz_move = v_move .* pz_move;

c_s_move  =  zeros(size(v_move));
c_sx_move =  zeros(size(v_move));
c_sy_move =  zeros(size(v_move));
c_sz_move =  zeros(size(v_move));

c_sx_move(1) = vx_move(1) * delta_t; 
c_sy_move(1) = vy_move(1) * delta_t; 
c_sz_move(1) = vz_move(1) * delta_t; 
for i = 2 : length(v_move)
    c_sx_move(i) = vx_move(i) * delta_t + c_sx_move(i-1); 
    c_sy_move(i) = vy_move(i) * delta_t + c_sy_move(i-1); 
    c_sz_move(i) = vz_move(i) * delta_t + c_sz_move(i-1); 
end

subplot(3,1,1);plot(vx_stable,'DisplayName','vx_s_t_a_b_l_e','Color','r');hold on;plot(vx_move,'DisplayName','vx_m_o_v_e');hold off;title('Axis X');grid on;
subplot(3,1,2);plot(vy_stable,'DisplayName','vy_s_t_a_b_l_e','Color','r');hold on;plot(vy_move,'DisplayName','vy_m_o_v_e');hold off;title('Axis Y');grid on;
subplot(3,1,3);plot(vz_stable,'DisplayName','vz_s_t_a_b_l_e','Color','r');hold on;plot(vz_move,'DisplayName','vz_m_o_v_e');hold off;title('Axis Z');grid on;

figure(2);
plot3(sx_stable,sy_stable,sz_stable,sx_move,sy_move,sz_move,c_sx_move,c_sy_move,c_sz_move);title('位移路径对比');grid on;
hold off;

figure(3);
diff_sx = c_sx_move - sx_move(1:end-1);
diff_sy = c_sy_move - sy_move(1:end-1);
diff_sz = c_sz_move - sz_move(1:end-1);
plot(diff_sx);hold on;
plot(diff_sy);
plot(diff_sz);
fprintf('max error sx = %.8f\r\n',max(diff_sx));
fprintf('max error sy = %.8f\r\n',max(diff_sz));
fprintf('max error sz = %.8f\r\n',max(diff_sy));
hold off;
