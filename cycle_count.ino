/*
 * cycle_count.ino
 *
 *  Created on: 30.11.2021
 *      Author: PC
 */





struct cycleCount
{
    uint32_t complete;
    uint32_t load;
    uint32_t rest;
    uint32_t count;
} cycle = {0, 0, 0, 0};


static uint32_t lastCount = 0;
static uint32_t lastCountPre = 0;

inline void CyclePrint(void)
{
    Serial.printf("cc: %d, %d, %d, %d -> %0.3f\n", cycle.load, cycle.rest, cycle.complete, cycle.count, (100.0f * ((float)cycle.load)) / ((float)cycle.complete));
    cycle.load = 0;
    cycle.rest = 0;
    cycle.complete = 0;
    cycle.count = 0;

#if 0
    {
        // Stats are enabled in sdkconfig freertos
        static char __stats_buffer[1024];
        vTaskGetRunTimeStats(__stats_buffer);
        printf("%s\n", __stats_buffer);
        vTaskList(__stats_buffer);
        printf( "%s\n", __stats_buffer);

        // main             686679    20%
        // IDLE             2517502    76%
        // IDLE             3199903    97%
        // esp_timer        23         <1%
    }
#endif
}

inline void calcCycleCountPre(void)
{
#ifdef ESP8266
    uint32_t newCnt = esp_get_cycle_count();
#endif
#ifdef ESP32
    uint32_t newCnt = ESP.getCycleCount();
#endif
#ifdef TEENSYDUINO
    uint32_t newCnt = ARM_DWT_CYCCNT;
#endif
    cycle.load += newCnt - lastCount;
    lastCountPre = newCnt;
}

inline void calcCycleCount(void)
{
#ifdef ESP8266
    uint32_t newCnt = esp_get_cycle_count();
#endif

#ifdef ESP32
    uint32_t newCnt = ESP.getCycleCount();
#endif
#ifdef TEENSYDUINO
    uint32_t newCnt = ARM_DWT_CYCCNT;
#endif


    cycle.complete += newCnt - lastCount;
    cycle.rest += newCnt - lastCountPre;
    lastCount = newCnt;
    cycle.count += 1;
}
