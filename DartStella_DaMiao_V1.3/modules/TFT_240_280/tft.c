//
// Created by Chen on 25-10-13.
//

#include "lcd.h"
#include "message_center.h"
#include <math.h>
#include <stdint.h>

#include "buzzer.h"
#include "robot_def.h"


#define SCREEN_W  280    // 你的屏幕宽度
#define SCREEN_H  240    // 你的屏幕高度
#define CUBE_SIZE 30     // 正方体边长（控制大小）
#define CENTER_X  SCREEN_W/2  // 屏幕中心点X
#define CENTER_Y  SCREEN_H/2  // 屏幕中心点Y

static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息
static BuzzzerInstance *robocmd_alarm;

// 三维顶点结构体
typedef struct {
    float x, y, z;
} Point3D;

// 二维点结构体（用于投影后存储）
typedef struct {
    int x, y;
} Point2D;

// 三维旋转（x/y/z轴）-> 二维投影 -> 绘制正方体
void drawRotatingCube(float angleX, float angleY, float angleZ,int centerX, int centerY,int16_t color) {
    // 1. 定义正方体8个原始顶点（中心在原点）
    Point3D cube[8] = {
        {-CUBE_SIZE/2, -CUBE_SIZE/2, -CUBE_SIZE/2},
        { CUBE_SIZE/2, -CUBE_SIZE/2, -CUBE_SIZE/2},
        { CUBE_SIZE/2,  CUBE_SIZE/2, -CUBE_SIZE/2},
        {-CUBE_SIZE/2,  CUBE_SIZE/2, -CUBE_SIZE/2},
        {-CUBE_SIZE/2, -CUBE_SIZE/2,  CUBE_SIZE/2},
        { CUBE_SIZE/2, -CUBE_SIZE/2,  CUBE_SIZE/2},
        { CUBE_SIZE/2,  CUBE_SIZE/2,  CUBE_SIZE/2},
        {-CUBE_SIZE/2,  CUBE_SIZE/2,  CUBE_SIZE/2}
    };

    // 2. 角度转换（度→弧度，C语言三角函数用弧度）
    float rx = angleX * M_PI / 180.0f;
    float ry = angleY * M_PI / 180.0f;
    float rz = angleZ * M_PI / 180.0f;

    // 3. 旋转矩阵系数（x→y→z轴旋转顺序）
    float cosX = cosf(rx), sinX = sinf(rx);
    float cosY = cosf(ry), sinY = sinf(ry);
    float cosZ = cosf(rz), sinZ = sinf(rz);

    Point3D rotated[8];
    // 4. 对每个顶点执行三维旋转
    for (int i = 0; i < 8; i++) {
        Point3D p = cube[i];
        float x, y, z;

        // X轴旋转
        y = p.y * cosX - p.z * sinX;
        z = p.y * sinX + p.z * cosX;
        p.y = y; p.z = z;

        // Y轴旋转
        x = p.x * cosY + p.z * sinY;
        z = -p.x * sinY + p.z * cosY;
        p.x = x; p.z = z;

        // Z轴旋转
        x = p.x * cosZ - p.y * sinZ;
        y = p.x * sinZ + p.y * cosZ;
        p.x = x; p.y = y;

        rotated[i] = p;
    }

    // 5. 正交投影（z轴影响缩放，增强立体感）
    Point2D proj[8];  // 修复原代码的语法错误
    float perspectiveFactor = 200.0f;  // 透视强度（值越小，近大远小越明显）
    for (int i = 0; i < 8; i++) {
        float scale = perspectiveFactor / (perspectiveFactor + rotated[i].z);
        // 核心修改：使用传入的centerX/centerY作为显示中心
        proj[i].x = centerX + (int)(rotated[i].x * scale);
        proj[i].y = centerY + (int)(rotated[i].y * scale);
    }

    // 6. 绘制正方体12条边（顶点连接顺序）
    int edges[12][2] = {
        {0,1}, {1,2}, {2,3}, {3,0},  // 前面
        {4,5}, {5,6}, {6,7}, {7,4},  // 后面
        {0,4}, {1,5}, {2,6}, {3,7}   // 侧面连接
    };
    for (int i = 0; i < 12; i++) {
        int p1 = edges[i][0], p2 = edges[i][1];
        LCD_DrawLine(proj[p1].x, proj[p1].y, proj[p2].x, proj[p2].y,color);
    }
}


void Test_print_UI_Init(void) {
    //LCD_Init();//LCD初始化
    //LCD_Fill(0,0,LCD_W, LCD_H,WHITE);
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    //LCD_Fill(0,0,LCD_W, LCD_H,BLACK);
    LCD_ShowString(0, 12,(uint8_t *)"         PASS         ", GREEN, BLACK, 24, 0);

    Buzzer_config_s buzzer_config ={
        .alarm_level = ALARM_LEVEL_HIGH, //设置警报等级 同一状态下 高等级的响应
        .loudness=  0.4, //设置响度
        .octave=  OCTAVE_1, // 设置音阶
    };
    robocmd_alarm = BuzzerRegister(&buzzer_config);


}
void tft_ui_task(void) {
    AlarmSetStatus(robocmd_alarm, ALARM_ON);
    drawRotatingCube(-gimbal_fetch_data.gimbal_imu_data.Roll,-gimbal_fetch_data.gimbal_imu_data.Pitch,-gimbal_fetch_data.gimbal_imu_data.Yaw,210,30,BLACK);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    //LCD_Fill(0,0,LCD_W, LCD_H,BLACK);
    //LCD_ShowString(20, 12,(uint8_t *)"        PASS         ", GREEN, BLACK, 24, 0);
    drawRotatingCube(-gimbal_fetch_data.gimbal_imu_data.Roll,-gimbal_fetch_data.gimbal_imu_data.Pitch,-gimbal_fetch_data.gimbal_imu_data.Yaw,210,30,WHITE);
    // LCD_ShowChinese(84, 100, (uint8_t *)"达妙科技", WHITE, BLACK, 32, 0);

}
