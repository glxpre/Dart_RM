#include "buzzer.h"
#include "bsp_pwm.h"
#include "bsp_dwt.h"
#include "string.h"

static PWMInstance *buzzer_pwm;
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0};

// 辅助函数：获取当前系统时间（毫秒）
static uint32_t Buzzer_GetTick(void) {
    // 可根据实际情况替换为DWT计时或其他高精度计时方式
    return HAL_GetTick();
}

// 辅助函数：设置PWM频率（根据定时器配置计算ARR）
static void Buzzer_SetPWMFreq(uint16_t freq) {
    if (freq == 0) {
        PWMSetDutyRatio(buzzer_pwm, 0); // 休止符：关闭输出
        return;
    }
    // 动态设置PWM周期（假设PWMSetPeriod参数为秒）
    PWMSetPeriod(buzzer_pwm, 1.0f / freq);
}

/**
 * @brief 蜂鸣器初始化
 */
void BuzzerInit(void) {
    PWM_Init_Config_s buzzer_config = {
        .htim = &htim12,
        .channel = TIM_CHANNEL_2,
        .dutyratio = 0,
        .period = 0.001f, // 初始周期1ms，后续动态调整
    };
    buzzer_pwm = PWMRegister(&buzzer_config);
}

/**
 * @brief 注册蜂鸣器实例
 */
BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config) {
    if (config->alarm_level >= BUZZER_DEVICE_CNT) while (1); // 错误处理

    BuzzzerInstance *buzzer_temp = (BuzzzerInstance *)malloc(sizeof(BuzzzerInstance));
    if (buzzer_temp == NULL) return NULL;
    memset(buzzer_temp, 0, sizeof(BuzzzerInstance));

    // 初始化默认参数
    buzzer_temp->alarm_level = config->alarm_level;
    buzzer_temp->loudness = config->loudness;
    buzzer_temp->octave = config->octave;
    buzzer_temp->alarm_state = ALARM_OFF;
    buzzer_temp->is_playing = 0;
    buzzer_temp->current_priority = ALARM_LEVEL_LOW;

    buzzer_list[config->alarm_level] = buzzer_temp;
    return buzzer_temp;
}

/**
 * @brief 设置报警状态
 */
void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state) {
    if (buzzer == NULL) return;
    buzzer->alarm_state = state;
}

/**
 * @brief 设置蜂鸣器响度
 */
void BuzzerSetLoudness(BuzzzerInstance *buzzer, float loudness) {
    if (buzzer == NULL) return;
    // 限制响度范围0.0~1.0
    loudness = (loudness < 0.0f) ? 0.0f : (loudness > 1.0f ? 1.0f : loudness);
    buzzer->loudness = loudness;
    // 正在播放时立即更新
    if (buzzer->is_playing && buzzer->note_sequence[buzzer->current_note_idx].freq != 0) {
        PWMSetDutyRatio(buzzer_pwm, loudness);
    }
}

/**
 * @brief 播放音符序列（非阻塞）
 * @param priority 优先级：数值越小优先级越高，可打断低优先级播放
 */
void BuzzerPlaySequence(BuzzzerInstance *buzzer, const Note_t *sequence, uint16_t length, AlarmLevel_e priority) {
    if (buzzer == NULL || sequence == NULL || length == 0) return;

    // 仅当未播放 或 新优先级更高时才触发
    if (!buzzer->is_playing || priority < buzzer->current_priority) {
        buzzer->note_sequence = sequence;
        buzzer->sequence_length = length;
        buzzer->current_note_idx = 0;
        buzzer->play_start_time = Buzzer_GetTick();
        buzzer->is_playing = 1;
        buzzer->current_priority = priority;

        // 播放第一个音符
        uint16_t first_freq = sequence[0].freq;
        Buzzer_SetPWMFreq(first_freq);
        PWMSetDutyRatio(buzzer_pwm, (first_freq != 0) ? buzzer->loudness : 0);
    }
}

/**
 * @brief 停止当前播放
 */
void BuzzerStop(BuzzzerInstance *buzzer) {
    if (buzzer == NULL) return;
    buzzer->is_playing = 0;
    PWMSetDutyRatio(buzzer_pwm, 0);
}

/**
 * @brief 蜂鸣器循环任务（定期调用，周期10ms）
 */
void BuzzerTask(void) {
    BuzzzerInstance *buzz;
    uint32_t current_time;
    uint32_t elapsed_time;

    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i) {
        buzz = buzzer_list[i];
        if (buzz == NULL || !buzz->is_playing) continue;

        current_time = Buzzer_GetTick();
        elapsed_time = current_time - buzz->play_start_time;
        const Note_t *current_note = &buzz->note_sequence[buzz->current_note_idx];

        // 检查当前音符是否播放完毕
        if (elapsed_time >= current_note->duration_ms) {
            buzz->current_note_idx++;
            if (buzz->current_note_idx >= buzz->sequence_length) {
                // 播放结束
                buzz->is_playing = 0;
                PWMSetDutyRatio(buzzer_pwm, 0);
                continue;
            }
            // 更新下一个音符的开始时间
            buzz->play_start_time = current_time;
        }

        // 更新当前音符的频率和响度
        uint16_t freq = buzz->note_sequence[buzz->current_note_idx].freq;
        Buzzer_SetPWMFreq(freq);
        PWMSetDutyRatio(buzzer_pwm, (freq != 0) ? buzz->loudness : 0);

        // 单PWM限制：只处理一个最高优先级实例
        break;
    }
}