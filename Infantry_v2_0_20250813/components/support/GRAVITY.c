//#include "GRAVITY.h"
//#include "arm_math.h"

//BallisticParams params;

//void init_BallisticParams(BallisticParams* params){
//    params->k = K; // ����ϵ��
//    params->current_v = Initial_velocity; // ����
//    params->s_bias = S_bias; // ǹ��ǰ�Ƶľ���
//    params->z_bias = Z_bias; // yaw������ǹ��ˮƽ��Ĵ�ֱ����
//}

//float monoDirectionalAirResistanceModel(float s, float v, float angle, BallisticParams params) {
//    float z;
//    // �������ʱ�� t
//    float t = (float)((exp(params.k * s) - 1) / (params.k * v * cos(angle)));
//    if (t < 0) {
//        // Ŀ��㳬��������
//        return 0;
//    }
//    // ����߶� z
//    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
//    return z;
//}

//float pitchTrajectoryCompensation(float s, float z, float v, BallisticParams params) {
//    float z_temp, z_actual, dz;
//    float angle_pitch;
//    int i = 0;
//    z_temp = z;

//    // ��������
//    for (i = 0; i < 20; i++) {
//        angle_pitch = atan2(z_temp, s); // ���㵱ǰ�Ƕ�
//        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch, params);
//        if (!z_actual) {
//            angle_pitch = 0;
//            break;
//        }
//        dz = 0.3 * (z - z_actual); // ����߶����
//        z_temp = z_temp + dz; // ����Ŀ��߶�
//        if (fabsf(dz) < 0.001) {
//            break; // ����㹻С���˳�����
//        }
//    }
//    return angle_pitch;
//}

//float calculateCompensationAngle(float shoot_angle, float distance, float initial_velocity, BallisticParams params) {

//    // ����ˮƽ����
//    float horizontal_distance = distance * cos(shoot_angle);
//    // ���㴹ֱ����
//    float vertical_distance = distance * sin(shoot_angle);
//    // ���㲹����
//    float compensation_angle = pitchTrajectoryCompensation(horizontal_distance, vertical_distance, params.current_v, params);

//    return compensation_angle;
//}

#include "GRAVITY.h"
#include "arm_math.h"
#include <math.h>
#include "float.h"
#define MAX_ITERATIONS 10       // �����������Ż�
#define CONVERGENCE_THRESHOLD 0.0001f  // ������ֵ����
#define MIN_DISTANCE 0.1f       // ��С��Ч����

BallisticParams params;

void init_BallisticParams(BallisticParams* params){
		
//    params->k = 0.5f * AIR_DENSITY * PROJECTILE_MASS; // ����ȷ�ĵ���ϵ������
		params->k = 0.092; //��Դ����ʦ��Vanguardս�ӣ�����ΪRM_VISION�������䣬deep seek����0.0402�����ϵĹ�ʽ
//		params->k = 0.0402;
    params->s_bias = 0.15f;
    params->z_bias = 0.21f;
}

// �Ľ��ĵ���ģ�ͣ����Ƕ��ο���������
float advancedAirResistanceModel(float s, float v, float angle, BallisticParams params) {
    const float cos_theta = arm_cos_f32(angle);
    const float sin_theta = arm_sin_f32(angle);
    
    // ʹ������-����������΢�ַ�����ֵ��
    float t = 0.0f;
    float dt = 0.001f;
    float x = 0.0f, z = 0.0f;
    float vx = v * cos_theta;
    float vz = v * sin_theta;
    
    for(int i=0; i<10000 && x<s; i++){
        const float velocity = sqrtf(vx*vx + vz*vz);
        const float drag = params.k * velocity;
        
        // x�����˶�
        vx -= drag * vx * dt;
        x += vx * dt;
        
        // z�����˶�
        vz -= (GRAVITY + drag * vz) * dt;
        z += vz * dt;
        
        t += dt;
        
        // ��ǰ�˳�����
        if(x > s || z < -10.0f) break;
    }
    return z;
}

// �Ż������㷨��ţ��-����ѷ����
float optimizedPitchCompensation(float s, float z_target, float v, BallisticParams params) {
    float angle = atan2f(z_target, s);  // ��ʼ����
    float delta;
    float best_angle = angle;
    float min_error = FLT_MAX;
    
    for(int i=0; i<MAX_ITERATIONS; i++){
        const float z_actual = advancedAirResistanceModel(s, v, angle, params);
        const float error = z_target - z_actual;
        
        // ������ֵ����
        const float delta_angle = 0.001f;
        const float z_deriv = advancedAirResistanceModel(s, v, angle + delta_angle, params);
        const float derivative = (z_deriv - z_actual) / delta_angle;
        
        // ţ�ٵ�����
        if(fabsf(derivative) > 1e-6f) {
            delta = error / derivative;
            angle += delta;
        }
        
        // ��¼��ѽ�
        if(fabsf(error) < min_error){
            min_error = fabsf(error);
            best_angle = angle;
        }
        
        // �������
        if(fabsf(delta) < CONVERGENCE_THRESHOLD) break;
    }
    return best_angle;
}

float calculateCompensationAngle(float shoot_angle, float distance, float initial_velocity, BallisticParams params) {
    // ����У��
    if(distance < MIN_DISTANCE) return shoot_angle;
    
    // ����ת��������ǹ��ƫ�ƣ�
    const float s = distance * cosf(shoot_angle) + params.s_bias;
    const float z = distance * sinf(shoot_angle) + params.z_bias;
    
    // ʹ���Ż���ĵ���ģ��
    return optimizedPitchCompensation(s, z, initial_velocity, params);
}

