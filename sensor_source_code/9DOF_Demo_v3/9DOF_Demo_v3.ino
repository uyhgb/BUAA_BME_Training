#include "IMU.h"

// ==================== 全局变量区域 ====================
// IMU 数据结构体
EulerAngles stAngles;                      // 欧拉角 (roll, pitch, yaw)
IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;    // 陀螺仪原始数据 (°/s)
IMU_ST_SENSOR_DATA_FLOAT stAccelRawData;   // 加速度计原始数据 (g)
IMU_ST_SENSOR_DATA stMagnRawData;          // 磁力计原始数据 (short int)

// 计时器变量
unsigned long previousMillis = 0;
const long interval = 10;     // 10ms 间隔 = 100Hz 采样率

// 串口波特率
const long BAUD_RATE = 115200;

// ==================== 初始化函数 ====================
void setup() {
  // 1. 初始化串口通信
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ; // 等待串口连接 (仅 USB CDC 需要)
  }
  
  // 2. 初始化 IMU 传感器
  imuInit();
  
  // 3. 等待传感器稳定
  delay(100);
  
  // 4. 发送 CSV 表头 (方便 SVM 数据处理)
  Serial.println("Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ");
  
  // 5. 重置计时器起点
  previousMillis = millis();
}

// ==================== 主循环 ====================
void loop() {
  unsigned long currentMillis = millis();

  // 【核心逻辑】非阻塞计时器
  // 只有当时间过去了 10ms，才执行一次读取和发送
  if (currentMillis - previousMillis >= interval) {
    // 更新计时器 (处理 millis() 溢出的正确方法)
    previousMillis = currentMillis;

    // 1. 获取 IMU 数据
    imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);

    // 2. 发送 CSV 格式数据 (Timestamp,Roll,Pitch,Yaw,Ax,Ay,Az,Gx,Gy,Gz)
    // 【优化】使用预分配缓冲区减少多次 Serial.print() 的延迟
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
             "%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
             currentMillis,                // 时间戳 (ms) - SVM 时间同步用
             stAngles.roll, stAngles.pitch, stAngles.yaw,
             stAccelRawData.X, stAccelRawData.Y, stAccelRawData.Z,
             stGyroRawData.X, stGyroRawData.Y, stGyroRawData.Z
    );
    Serial.println(buffer);  // 一次性发送，减少阻塞
  }
  
  // 这里没有 delay()！
  // MCU 可以利用空闲时间处理:
  // - USB CDC 底层握手
  // - 中断服务 (如果有)
  // - 其他非阻塞任务
}
