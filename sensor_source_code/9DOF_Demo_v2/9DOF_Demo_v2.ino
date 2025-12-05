#include"IMU.h"
#include <ArduinoJson.h>  // 需要安装ArduinoJson库

/*
 * 数据通信说明:
 * ESP32 (本设备) <--串口线--> 电脑 (ROS2节点)
 * 
 * 为什么不能直接调用imuDataGet()?  
 * - ESP32和电脑是两个独立的物理设备
 * - imuDataGet()只能在ESP32上调用,无法跨设备
 * - 需要通过串口将数据编码后发送给电脑
 * - 电脑端的ROS2节点接收串口数据并解析
 * 
 * JSON格式的优势:
 * - 人类可读,方便调试
 * - 跨平台,易于解析
 * - 结构清晰,字段可扩展
 */

// IMU数据结构体
EulerAngles stAngles;
IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;
IMU_ST_SENSOR_DATA_FLOAT stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;  // 不使用但需保留(imuDataGet参数需要)

void setup() {
  Serial.begin(115200);  // 设置波特率为115200
  imuInit();
}

void loop() {
  // 调用IMU库函数获取传感器数据(运行在ESP32上)
  imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  
  // 创建JSON文档(用于编码数据并通过串口发送)
  StaticJsonDocument<256> doc;
  
  // 填充姿态数据 (Roll, Pitch, Yaw) - 单位:度
  JsonObject orientation = doc.createNestedObject("orientation");
  orientation["roll"] = stAngles.roll;
  orientation["pitch"] = stAngles.pitch;
  orientation["yaw"] = stAngles.yaw;
  
  // 填充加速度数据 - 单位:m/s^2 (包含重力)
  JsonObject accel = doc.createNestedObject("acceleration");
  accel["x"] = stAccelRawData.X;
  accel["y"] = stAccelRawData.Y;
  accel["z"] = stAccelRawData.Z;
  
  // 填充角速度数据 - 单位:rad/s
  JsonObject gyro = doc.createNestedObject("gyroscope");
  gyro["x"] = stGyroRawData.X;
  gyro["y"] = stGyroRawData.Y;
  gyro["z"] = stGyroRawData.Z;
  
  // 【磁力计数据 - 已禁用】
  // 如需使用磁力计,取消下面的注释
  // JsonObject mag = doc.createNestedObject("magnetic");
  // mag["x"] = stMagnRawData.s16X;
  // mag["y"] = stMagnRawData.s16Y;
  // mag["z"] = stMagnRawData.s16Z;
  
  // 添加时间戳 - 单位:毫秒 (ESP32启动后的运行时间)
  doc["timestamp"] = millis();
  
  // 序列化并发送
  serializeJson(doc, Serial);
  Serial.println();  // 添加换行符作为消息分隔符
  
  delay(50);  // 20Hz发布频率 

  }

