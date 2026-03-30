# buzzer

用于拉响蜂鸣器警报

## 使用范例

# 蜂鸣器库使用说明

## 一、库功能概述
本蜂鸣器库支持**非阻塞式**播放自定义音符序列，具备以下特性：
- 多优先级控制（高优先级可打断低优先级）
- 同优先级未播放完时，重复调用不触发
- 实时响度调节


## 二、核心结构体说明

### 1. 音符结构体 `Note_t`
定义单个音符的频率和持续时间，用于组成播放序列。
```c
typedef struct {
    uint16_t freq;       // 音符频率 (Hz)，0为休止符（NOTE_REST）
    uint16_t duration_ms; // 持续时间 (毫秒)
} Note_t;
```

### 2. 蜂鸣器配置结构体 `Buzzer_config_s`
用于注册蜂鸣器实例时的初始化配置。
```c
typedef struct {
    AlarmLevel_e alarm_level; // 实例默认优先级（数值越小优先级越高）
    octave_e octave;           // 保留八度设置（兼容旧代码，新功能可忽略）
    float loudness;            // 响度 (占空比，范围 0.0~1.0，建议 0.3~0.7)
} Buzzer_config_s;
```

### 3. 蜂鸣器实例结构体 `BuzzzerInstance`
库内部维护的实例状态，用户无需直接操作成员，通过API函数控制即可。
```c
typedef struct {
    float loudness;            // 当前响度
    octave_e octave;           // 八度设置
    AlarmLevel_e alarm_level;  // 实例默认优先级
    AlarmState_e alarm_state;  // 原有报警状态（兼容旧代码）
    // 以下为内部播放控制成员，用户无需修改
    const Note_t *note_sequence; // 音符序列指针
    uint16_t sequence_length;    // 音符序列长度
    uint16_t current_note_idx;   // 当前播放索引
    uint32_t play_start_time;    // 当前音符开始时间
    uint8_t is_playing;          // 播放状态标志
    AlarmLevel_e current_priority; // 当前播放优先级
} BuzzzerInstance;
```


## 三、API 函数说明

### 1. 初始化函数 `BuzzerInit`
**功能**：初始化蜂鸣器硬件PWM，需在系统启动时调用一次。
```c
void BuzzerInit(void);
```

### 2. 注册函数 `BuzzerRegister`
**功能**：创建并注册一个蜂鸣器实例，返回实例指针供后续调用。
```c
BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config);
```
- **参数**：
    - `config`：指向 `Buzzer_config_s` 配置结构体的指针。
- **返回值**：
    - 成功：返回 `BuzzzerInstance` 实例指针；
    - 失败：返回 `NULL`（内存不足时）。

### 3. 播放序列函数 `BuzzerPlaySequence`
**功能**：非阻塞式播放自定义音符序列，支持优先级控制。
```c
void BuzzerPlaySequence(BuzzzerInstance *buzzer, const Note_t *sequence, 
                         uint16_t length, AlarmLevel_e priority);
```
- **参数**：
    - `buzzer`：已注册的蜂鸣器实例指针；
    - `sequence`：指向 `Note_t` 音符数组的指针；
    - `length`：音符数组的长度（元素个数）；
    - `priority`：本次播放的优先级（数值越小优先级越高，如 `ALARM_LEVEL_HIGH`）。
- **特性**：
    - 若实例未播放，立即触发；
    - 若实例正在播放，仅当新优先级 **更高**（数值更小）时，打断当前播放并触发新序列；
    - 同优先级重复调用，未播放完时不触发。

### 4. 停止播放函数 `BuzzerStop`
**功能**：立即停止当前播放。
```c
void BuzzerStop(BuzzzerInstance *buzzer);
```
- **参数**：
    - `buzzer`：已注册的蜂鸣器实例指针。

### 5. 设置响度函数 `BuzzerSetLoudness`
**功能**：实时调整蜂鸣器响度（占空比）。
```c
void BuzzerSetLoudness(BuzzzerInstance *buzzer, float loudness);
```
- **参数**：
    - `buzzer`：已注册的蜂鸣器实例指针；
    - `loudness`：响度值（范围 `0.0~1.0`，超出范围自动限制）。

### 6. 循环任务函数 `BuzzerTask`
**功能**：非阻塞播放的核心调度函数，需在主循环或RTOS任务中**定期调用**（建议周期1~10ms）。
```c
void BuzzerTask(void);
```


## 四、使用步骤示例

### 步骤1：定义音符序列
```c
// 示例：简单的"滴-滴"提示音
const Note_t simple_beep[] = {
    {NOTE_C5, 200},  // 响200ms
    {NOTE_REST, 200}, // 停200ms
    {NOTE_C5, 200},  // 再响200ms
    {NOTE_REST, 1400} // 停1.4s（总周期2s）
};
```

### 步骤2：初始化与注册
```c
// 1. 系统启动时初始化硬件
BuzzerInit();

// 2. 定义配置并注册实例
Buzzer_config_s config = {
    .alarm_level = ALARM_LEVEL_MEDIUM, // 默认优先级
    .loudness = 0.5f,                   // 初始响度50%
    .octave = OCTAVE_4,                  // 兼容旧代码，可忽略
};
BuzzzerInstance *my_buzzer = BuzzerRegister(&config);
```

### 步骤3：调用播放
```c
// 在需要触发提示音的地方调用
BuzzerPlaySequence(my_buzzer, simple_beep, 
                   sizeof(simple_beep)/sizeof(simple_beep[0]), 
                   ALARM_LEVEL_MEDIUM);
```




## 四、优先级枚举参考
```c
typedef enum {
    ALARM_LEVEL_HIGH = 0,      // 最高优先级
    ALARM_LEVEL_ABOVE_MEDIUM,
    ALARM_LEVEL_MEDIUM,
    ALARM_LEVEL_BELOW_MEDIUM,
    ALARM_LEVEL_LOW,            // 最低优先级
} AlarmLevel_e;
```