#include "idle.h"
#include "main.h"
#include "cmsis_os.h"

volatile unsigned long idleTime = 0;

void idle_task(void const * argument){
	uint32_t lastWakeTime = osKernelSysTick();
    while (1) {
        // ��¼������������ʱ��
        uint32_t enterIdleTime = osKernelSysTick();
				
        // ִ�п����������ز���
        // ��������������Ҫ�ڿ���ʱ��ִ�еĴ���
			
        // �������ʱ�䣨�Եδ�Ϊ��λ��
        uint32_t currentTime = osKernelSysTick();
        idleTime += (currentTime - enterIdleTime);

    }
}
