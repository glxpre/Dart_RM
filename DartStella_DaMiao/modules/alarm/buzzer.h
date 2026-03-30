#ifndef BUZZER_H
#define BUZZER_H
#include "bsp_pwm.h"
#include <stdint.h>

#define BUZZER_DEVICE_CNT 5

// 音符频率定义（十二平均律，A4=440Hz，取整适配uint16_t）
#define NOTE_C1  33
// 小字组（Octave 3，低八度）
#define NOTE_C3  131
#define NOTE_D3  147
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_G3  196
#define NOTE_A3  220
#define NOTE_bB3 233 // 降B3
#define NOTE_B3  247

// 小字一组（Octave 4，中音区）
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_bB4 466 // 降B4（简谱中音1，1=♭B）
#define NOTE_B4  494

// 小字二组（Octave 5，高音区，简谱中音2-7、高八度1）
#define NOTE_C5  523  // 简谱中音2
#define NOTE_D5  587  // 简谱中音3
#define NOTE_bE5 622  // 降E5（简谱中音4）
#define NOTE_E5  659
#define NOTE_F5  698  // 简谱中音5
#define NOTE_G5  784  // 简谱中音6
#define NOTE_A5  880  // 简谱中音7
#define NOTE_bB5 932  // 降B5（简谱高八度1，数字上带1个点）
#define NOTE_B5  988

// 小字三组（Octave 6，倍高音区，简谱高八度2-7）
#define NOTE_C6  1047 // 简谱高八度2
#define NOTE_D6  1175 // 简谱高八度3
#define NOTE_bE6 1245 // 降E6（简谱高八度4）
#define NOTE_E6  1319
#define NOTE_F6  1397 // 简谱高八度5
#define NOTE_G6  1568 // 简谱高八度6
#define NOTE_A6  1760 // 简谱高八度7
#define NOTE_bB6 1865 // 降B6（简谱倍高八度1）
#define NOTE_B6  1976

#define NOTE_REST 0    // 休止符

typedef enum {
    OCTAVE_1 = 0,
    OCTAVE_2,
    OCTAVE_3,
    OCTAVE_4,
    OCTAVE_5,
    OCTAVE_6,
    OCTAVE_7,
    OCTAVE_8,
} octave_e;

typedef enum {
    ALARM_LEVEL_HIGH = 0,      // 最高优先级
    ALARM_LEVEL_ABOVE_MEDIUM,
    ALARM_LEVEL_MEDIUM,
    ALARM_LEVEL_BELOW_MEDIUM,
    ALARM_LEVEL_LOW,            // 最低优先级
} AlarmLevel_e;

typedef enum {
    ALARM_OFF = 0,
    ALARM_ON,
} AlarmState_e;

// 音符结构体：包含频率和持续时间
typedef struct {
    uint16_t freq;       // 音符频率 (Hz)，0为休止符
    uint16_t duration_ms; // 持续时间 (毫秒)
} Note_t;

typedef struct {
    AlarmLevel_e alarm_level; // 实例的默认优先级
    octave_e octave;           // 原有八度设置（可选）
    float loudness;            // 响度 (占空比，范围0.0~1.0)
} Buzzer_config_s;

typedef struct {
    float loudness;            // 响度
    octave_e octave;           // 八度
    AlarmLevel_e alarm_level;  // 实例默认优先级
    AlarmState_e alarm_state;  // 原有报警状态（保留）
    // 新增播放控制成员
    const Note_t *note_sequence; // 音符序列指针（指向外部数组）
    uint16_t sequence_length;    // 音符序列长度
    uint16_t current_note_idx;   // 当前播放的音符索引
    uint32_t play_start_time;    // 当前音符开始播放的时间（毫秒）
    uint8_t is_playing;          // 是否正在播放 (0: 未播放, 1: 正在播放)
    AlarmLevel_e current_priority; // 当前播放的优先级
} BuzzzerInstance;

// 初始化函数
void BuzzerInit(void);
// 注册蜂鸣器实例
BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config);
// 设置报警状态（保留原有接口）
void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state);
// 播放音符序列（非阻塞，priority为优先级，数值越小优先级越高）
void BuzzerPlaySequence(BuzzzerInstance *buzzer, const Note_t *sequence, uint16_t length, AlarmLevel_e priority);
// 停止当前播放
void BuzzerStop(BuzzzerInstance *buzzer);
// 设置响度
void BuzzerSetLoudness(BuzzzerInstance *buzzer, float loudness);
// 循环任务（需在主循环或RTOS任务中定期调用）
void BuzzerTask(void);

#endif // !BUZZER_H