#ifndef _GRAVITY_H
#define _GRAVITY_H

typedef struct {
    float k;             // ����ϵ������������ϵ����
    float current_v;     // ��ǰ���٣�m/s��
    float s_bias;        // ǹ��ˮƽƫ��������m��
    float z_bias;        // yaw�ᵽǹ��ˮƽ��Ĵ�ֱ������m��
} BallisticParams;

#define GRAVITY         9.80665f    // ʹ�ø���ȷ���������ٶ�
#define AIR_DENSITY     1.225f      // �����ܶȣ�kg/m3��
#define PROJECTILE_MASS 0.0032f      // ����������kg��
#define Initial_velocity 16.2

extern BallisticParams params;
void init_BallisticParams(BallisticParams* params);
float calculateCompensationAngle(float shoot_angle, float distance, float initial_velocity, BallisticParams params);
#endif
