#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define XYZ_COORD 3
#define TOTLE 0
#define ACC 1
#define UNI 2
#define DEC 3
#define SQUARE(n) ((n) * (n))

typedef struct _AxisStableParam_t {
    float t_totle;
    float t_acc;
    float t_uni;
    float t_dec;
    float s_acc;
    float s_uni;
    float s_dec;
} AxisStableParam_t;

unsigned int calc_simple_trapzoid_speed_ctrl(float delta[XYZ_COORD], float v0_stable[XYZ_COORD], float vt_stable[XYZ_COORD], float a_stable[XYZ_COORD], AxisStableParam_t* p)
{
    unsigned int axis;

    float s, v0, vt, a;

    float s_acc, s_uni, s_dec;
    float t_acc, t_uni, t_dec;
    float t_totle;

    for (axis = X_AXIS; axis <= Z_AXIS; axis++) {
        s = delta[axis];
        if (s >= 0) {
            v0 = v0_stable[axis];
            vt = vt_stable[axis];
            a = a_stable[axis];
            break;
        }
    }

    if (axis > Z_AXIS)
        return 1;

    s_acc = (SQUARE(vt) - SQUARE(v0)) / (2 * a);
    s_dec = s_acc;
    s_uni = s - s_acc - s_dec;

    if (s_uni > 0) {
        t_acc = (vt - v0) / a;
        t_uni = s_uni / vt;
        t_dec = t_acc;
    } else {
        s_acc = s / 2;
        s_dec = s_acc;
        s_uni = s - s_acc - s_dec;

        vt = sqrt(2 * a * s_acc + SQUARE(v0));

        t_acc = (vt - v0) / a;
        t_uni = 0;
        t_dec = t_acc;
    }

    t_totle = t_acc + t_uni + t_dec;

    p->t_totle = t_totle;
    p->t_acc = t_acc;
    p->t_uni = t_uni;
    p->t_dec = t_dec;
    p->s_acc = s_acc;
    p->s_uni = s_uni;
    p->s_dec = s_dec;

    return 0;
}

unsigned int calc_AxisMoveWithTraySpeed(float p0[XYZ_COORD], float p1[XYZ_COORD], float speed)
{
    unsigned int axis;
    unsigned int res;
    FILE * fp;

    fp = fopen ("file.txt", "w+");

    float v0_f = 20.0f;    //用户设定的最小插补速度
    float vt_f = 350.0f;   //用户设定的最大插补速度
    float  a_f = 1500.0f;
    float vc_f = 200.0f;
    /** float a_f = (vt_f - v0_f) / ... * MULT_1000; //用户设定的加速度 */

    float delta[XYZ_COORD];

    float s_stable;
    float p_stable[XYZ_COORD];
    float a_stable[XYZ_COORD];
    float v0_stable[XYZ_COORD];
    float vt_stable[XYZ_COORD];
    float vi_stable[XYZ_COORD];
    float si_stable[XYZ_COORD];

    float s_move_prev;
    float s_move_after;
    float v_move_totle;
    float p_move[XYZ_COORD];
    float vi_move[XYZ_COORD];
    float si_move_prev[XYZ_COORD];
    float si_move_after[XYZ_COORD];

    float cur_coord[XYZ_COORD];
    float next_coord[XYZ_COORD];

    AxisStableParam_t trapzoid;

    memset(si_move_prev, 0, sizeof(float) * XYZ_COORD);
    memset(cur_coord, 0, sizeof(float) * XYZ_COORD);
    s_move_prev = 0.0f;

    for (axis = X_AXIS; axis <= Z_AXIS; axis++)
        delta[axis] = p1[axis] - p0[axis];

    s_stable = sqrt(SQUARE(delta[X_AXIS]) + SQUARE(delta[Y_AXIS]) + SQUARE(delta[Z_AXIS]));

    for (axis = X_AXIS; axis <= Z_AXIS; axis++) {
        p_stable[axis] = delta[axis] / s_stable;
        a_stable[axis]  =  a_f * p_stable[axis];
        v0_stable[axis] = v0_f * p_stable[axis];
        vt_stable[axis] = vt_f * p_stable[axis];
    }

    res = calc_simple_trapzoid_speed_ctrl(delta, v0_stable, vt_stable, a_stable, &trapzoid);
    if (res != 0)
        return 1;

    const float delta_t = 0.001f;
    float t = 0.0f;
    float T = 0.0f;
    float T_acc = trapzoid.t_acc;
    float run_s[XYZ_COORD];
    float t_offset = 0.0f;

    while (t < trapzoid.t_totle) {
        //加速阶段
        T = t;
        if (t < trapzoid.t_acc) {
            for (axis = X_AXIS; axis <= Z_AXIS; axis++) {
                vi_stable[axis] = v0_stable[axis] + a_stable[axis] * T;
                si_stable[axis] = v0_stable[axis] * T + 0.5f * a_stable[axis] * SQUARE(T);
            }

            if ((t + delta_t) >= trapzoid.t_acc) {
                for (axis = X_AXIS; axis <= Z_AXIS; axis++) {
                    run_s[axis] = si_stable[axis];
                }
            }
        }
        //匀速阶段
        else if (t > trapzoid.t_acc && t < (trapzoid.t_acc + trapzoid.t_uni)) {
            t_offset = trapzoid.t_acc;
            T = t - t_offset;
            for (axis = X_AXIS; axis <= Z_AXIS; axis++) {
                vi_stable[axis] = v0_stable[axis] + a_stable[axis] * T_acc;
                si_stable[axis] = v0_stable[axis] * T + a_stable[axis] * T_acc * T + run_s[axis];
            }

            if ((t + delta_t) >= (trapzoid.t_acc + trapzoid.t_uni)) {
                for (axis = X_AXIS; axis <= Z_AXIS; axis++) {
                    run_s[axis] = si_stable[axis];
                }
            }
        }
        //减速阶段
        else {
            t_offset = trapzoid.t_uni + trapzoid.t_acc;
            T = t - t_offset;
            for (axis = X_AXIS; axis <= Z_AXIS; axis++) {
                vi_stable[axis] = v0_stable[axis] + a_stable[axis] * T_acc - a_stable[axis] * T;
                si_stable[axis] = v0_stable[axis] * T + a_stable[axis] * T_acc * T - 0.5f * a_stable[axis] * SQUARE(T) + run_s[axis];
            }
        }

        vi_move[X_AXIS] = vi_stable[X_AXIS] + vc_f;

        si_move_after[X_AXIS] = si_stable[X_AXIS] + vc_f * t;
        si_move_after[Y_AXIS] = si_stable[Y_AXIS];
        si_move_after[Z_AXIS] = si_stable[Z_AXIS];

        s_move_after = sqrt(SQUARE(si_move_after[X_AXIS]) + SQUARE(si_move_after[Y_AXIS]) + SQUARE(si_move_after[Z_AXIS]));

        for (axis = X_AXIS; axis <= Z_AXIS; axis++) {
            p_move[axis] = (si_move_after[axis] - si_move_prev[axis]) / (s_move_after - s_move_prev);
            if (isnan(p_move[axis])) //此处会出现数值太小,float值分辨率达不到 = 0的情况
                p_move[axis] = 1e-6;
        }

        v_move_totle = vi_move[X_AXIS] / p_move[X_AXIS];
        vi_move[Y_AXIS] = v_move_totle * p_move[Y_AXIS];
        vi_move[Z_AXIS] = v_move_totle * p_move[Z_AXIS];

        for (axis = X_AXIS; axis <= Z_AXIS; axis++)
            next_coord[axis] = vi_move[axis] * delta_t;

        for (axis = X_AXIS; axis <= Z_AXIS; axis++) {
            fprintf(fp,"%f,",cur_coord[axis] + next_coord[axis]);
        }
        fprintf(fp,"\r\n");

        for (axis = X_AXIS; axis <= Z_AXIS; axis++)
            cur_coord[axis] += next_coord[axis];

        memcpy(si_move_prev, si_move_after, sizeof(float) * XYZ_COORD);
        s_move_prev = s_move_after;

        t += delta_t;
    }

   fclose(fp);
   return 0;
}

int main(int argc, char *argv[])
{
    float p0[] = {0.0f ,0.0f, 0.0f};
    float p1[] = {1000.0f, 2000.0f, 500.0f};
    calc_AxisMoveWithTraySpeed(p0,p1,0);
    
    return 0;
}
