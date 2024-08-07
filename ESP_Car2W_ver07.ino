/* author : Calvin Kuo
// ESP_Car2W_ver07 (Aug.12,2023)
# 以較低頻率PWM模擬全壓啟動8msec。
// ESP_Car2W_ver06 (Aug.09,2023)
# 按 ESP_MotorStartup 所得數據重做馬達全壓起動
// ESP_MotorStartup (Aug.08,2023)
# 測量TT馬達(以減速比1:48款為例)之最小起動電壓(以PWM計)？及全壓起動之最小持續時間？
# 當電池電壓7.27V時，測得最小起動電壓 = PWM128。全壓起動之最小持續時間 = 8ms。
# 以上述測得全壓起動後，接續最小可運轉電壓(以PWM計)？
# 測得全壓起動後之最小可運轉電壓 = 48。
# 以上述測得之全壓起動後之最小可運轉電壓，重測量可能的最小全壓起動時間長度?
# 最小全壓起動時間長度 = 7ms。
// ESP_Car2W_ver05 (Aug.02,2023)
# 試作低速馬達啟動，全電壓啟動後再降轉速度。
// ESP_Car2W_ver04 (Jul.04,2023)
# 取消中斷，確認可行性。
# 整理程式，相同的DMP數據不重複執行PID。
// ESP_Car2W_ver03 (Jul.03,2023)
# Kp、Ki、Kd調校。經由藍芽修改PID參數。
# 測得較佳數據為：Kp=20、Ki=140、Kd=0.8
// ESP_Car2W_ver02 (Jul.03,2023)
# 測量整車在平衡狀態的Pitch角度。
# 暫時關閉TT Motor以靜止狀態嘗試以外力調整使整車平衡，以藍芽回傳Pitch角度數據。
# 測得約-7度、4度(含平衡重物)。
// ESP_Car2W_ver01 (Jul.03,2023)
# 參考函式庫的範例程式MPU6050_DMP6，試做兩輪平衡。
#不使用中斷還是可以讀取到最後一筆DMP數據，故可以考慮取消中斷。
// ESP_Car2W_Test (Jun.xx,2023)
#	試做兩輪平衡。由MPU6050 DMP取得Pitch角，經PID運算後控制TT Motor轉速。
// ESP_MPU6050_DMP6	 (Jun.xx,2023)
# 試做讀取 MPU6050 DMP
// Wiring
  ESP32 GPIO22 -> MPU6050 SCL
  ESP32 GPIO21 -> MPU6050 SDA
  ESP32 GPIO4  -> MPU6050 INT
  ESP32 GPIO13 -> HC-SR04 Trig
  ESP32 GPIO39 -> HC-SR04 Echo
  ESP32 GPIO33 -> L298N In1
  ESP32 GPIO25 -> L298N In2
  ESP32 GPIO26 -> L298N In3
  ESP32 GPIO27 -> L298N In4
  ESP32 GPIO14 -> L298N EnableA (PWM)
  ESP32 GPIO12 -> L298N EnableB (PWM)
  L298N Out1 -> Right Motor +
  L298N Out2 -> Right Motor -
  L298N Out3 -> Left Motor +
  L298N Out4 -> Left Motor -
  Battery+ -> L298N 12V
  L298N 5V -> ESP32 Ext5V
  ESP32 +5V -> MPU6050 VCC、HC-SR04 VCC
*/
#include <BluetoothSerial.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

// SSD1306 OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_Address 0x3C
// MPU6050
#define MPU6050_Address 0x68
// PWM 的上下限
#define minPWM 48     // 最小值
#define maxPWM 255    // 最大值
// PID 取樣時間
#define PID_SampleTime 10    // PID最小取樣時間10usec
//
//#define HCSR04Trig_pin  13
//#define HCSR04Echo_pin  39
#define L298NIn1_pin 33
#define L298NIn2_pin 25
#define L298NIn3_pin 26
#define L298NIn4_pin 27
#define L298NEnableA_pin 14
#define L298NEnableB_pin 12
#define rightChannel 0    // 配置使用PWM通道0
#define leftChannel 1     // 配置使用PWM通道1
// MPU6050 校正值 (以IMU_Zero 測量所得)
#define XGyroOffset 128
#define YGyroOffset 72
#define ZGyroOffset -18
#define ZAccelOffset 1286

BluetoothSerial SerialBT; // 宣告藍芽物件
MPU6050 mpu;
// MPU 變數
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer
// 方向與運動 變數
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// 姿態角變數
double currentYaw;   // Yaw (偏向角)
double currentPitch; // Pitch (俯仰角)
double targetYaw = 0.0;   // 目標Yaw值，PID運算之目標值
//double targetPitch = 0.0; // 目標Pitch值，PID運算之目標值// 自適應控制的起始值
double targetPitch = -7.1; // 目標Pitch值，PID運算之目標值// 整車於平衡狀態之Pitch角
// PID變數
double pitchKp = 20.0;  // 比例增益
double pitchKi = 130.0; // 積分增益
double pitchKd = 1.0;   // 微分增益
// PID參數組3 Kp：20、Ki：130、Kp：1.0 (2023 Aug 14)
// PID參數組2 Kp：30、Ki：400、Kp：1.0 (2023 Jul 14)
// PID參數組1 Kp：30、Ki：140、Kp：0.8
double rawPWM = 0.0;    // Pitch的PID運算結果。
double oldPWM = 0.0;    // 前次rawPWM
double yawKp = 4.0;     // 比例增益
double yawKi = 0.0;     // 積分增益
double yawKd = 0.8;     // 微分增益
double diffPWM = 0.0;   // yaw的PID運算結果。
PID pitchPID(&currentPitch, &rawPWM, &targetPitch, pitchKp, pitchKi, pitchKd, DIRECT);  // 宣告PID物件
PID yawPID(&currentYaw, &diffPWM, &targetYaw, yawKp, yawKi, yawKd, DIRECT);             // 宣告PID物件
// PWM 變數
int16_t targetSpeed=0;  // 設定行駛速度
int16_t rightPWM, leftPWM;    // 經過PID運算修正後的左右馬達PWM數值
//
//
  int16_t timeLength=0;   // 測量全壓起動時間用；只用於ESP_MotorStartup中，其他版本程式可刪除此變數
//

void setup() {
  Serial.begin(115200);
  Wire.begin();
  SerialBT.begin("ESP32_BT_Car");   // 初始化藍芽序列埠
  InitializeGPIOpin();  // Initialize GPIO pin
  InitializeMPU6050();  // Initialize MPU6050
  InitialPID();         // 配置PID參數
}  
void loop() {
  if (!dmpReady) return;
//  sonar();
//  avoidance();
//  adaptive();   // 自適應控制
  updatePID();
  controlCar();
  if (Serial.available()) {
    char myCommand = Serial.read();
    executeCommand(myCommand);
  }
  if (SerialBT.available()) {
    char myCommand = SerialBT.read();
    executeCommand(myCommand);
    pitchPID_tuning(myCommand);   // 調校PID用
  }
}
void sonar() {}
void avoidance() {}
/*void adaptive() {
    //自適應控制
    static double lastRawPWM = 0;
    static int times =0;
    double error = currentPitch - targetPitch;
    if ((rawPWM < 0 && lastRawPWM < 0) || (rawPWM > 0 && lastRawPWM > 0))  {
      times ++;
      if(times > 100) {
        targetPitch += 0.01 * error;
      }
    } else {
      times=0;
    }
    lastRawPWM = rawPWM;
}*/
void updatePID() {
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    currentYaw = ypr[0] * 180/M_PI;
    currentPitch = ypr[1] * 180/M_PI;
    pitchPID.Compute();   // 進行pitch的PID運算
    if( rawPWM < minPWM && rawPWM > -minPWM) {  // 校正 minPWM 範圍數值，以符合馬達最小可運轉電壓。
      rawPWM = 0;
    } 
    yawPID.Compute();     // 進行yaw的PID運算
//    Serial.print("taget:");
//    Serial.print(targetPitch);
//    Serial.print("\t");
//    Serial.print("current:");
//    Serial.println(currentPitch);
    //
    /*Serial.print("ypr\t");
    Serial.print(currentYaw);
    Serial.print("\t");
    Serial.print(currentPitch);
    Serial.print("\t");
    Serial.print(ypr[2] * 180/M_PI);
    Serial.print("\t");
    Serial.print(rawPWM);
    Serial.println();*/
/*    SerialBT.print("ypr\t");
    SerialBT.print(currentYaw);
    SerialBT.print("\t");
    SerialBT.print(currentPitch);
    SerialBT.print("\t");
    SerialBT.print(ypr[2] * 180/M_PI);
    SerialBT.println();*/
  }
}
void controlCar() {
/*  if(currentPitch > -20 && currentPitch < 20) {  // 設置合理的傾倒角度+-20度角 (暫定)，超出範圍者認定已完全傾倒。
    if(oldPWM != rawPWM) {
      if(rawPWM != 0) {
        if(oldPWM != 0 ) {
          if((oldPWM > 0 && rawPWM > 0) || (oldPWM < 0 && oldPWM < 0)) {
            rightPWM = rawPWM;
            leftPWM = rawPWM;
            oldPWM = rawPWM;
          } else {
            if(rawPWM > 0) {
              rightPWM = maxPWM;
              leftPWM = maxPWM;
              oldPWM = maxPWM;
            } else {
              rightPWM = -maxPWM;
              leftPWM = -maxPWM;
              oldPWM = -maxPWM;
            }
          }
        } else {
          if(rawPWM > 0) {
            rightPWM = maxPWM;
            leftPWM = maxPWM;
            oldPWM = maxPWM;
          } else {
            rightPWM = -maxPWM;
            leftPWM = -maxPWM;
            oldPWM = -maxPWM;
          }
        }
      } else {
        rightPWM = 0;
        leftPWM = 0;
        oldPWM =0;
      }
    }
    if(rightPWM > 0) // 試作，僅以RightPWM代表
      forward();
    else if(rawPWM < 0) // 向後傾倒
      backward();
    else
      stop(); // 正常直立中
  } else
    stop();   // 超出傾倒範圍，使其停止
*/

  rightPWM = rawPWM;
  leftPWM = rawPWM;
  if(currentPitch > -20 && currentPitch < 20) {  // 設置合理的傾倒角度+-20度角 (暫定)
    if(rawPWM > 0)    // 向前傾倒
      forward();
    else if(rawPWM < 0) // 向後傾倒
      backward();
    else
      stop(); // 正常直立中
  } else      // 超出傾倒範圍，使其停止
    stop();
}
void executeCommand(char myCommand) {
  switch (myCommand) {
    case 'f':   // forward
    case 'F':
      break;
    case 'b':   // backward
    case 'B':
      break;
    case 'r':   // turn right
    case 'R':
      break;
    case 'l':   // turn left
    case 'L':
      break;
    case 's':   // stop
    case 'S':
      break;
    case 'u':   // speed up
    case 'U':
      break;
    case 'd':   // speed down
    case 'D':
      break;
    default : 
      Serial.println("Unknown command");
      break;
  }
}
void pitchPID_tuning(char myCommand) {
  static char point='p';
  static float rate=0.1;
  switch (myCommand) {
    case 'p':
      point = 'p';
      SerialBT.println("point to Kp");
      break;
    case 'i':
      point = 'i';
      SerialBT.println("point to Ki");
      break;
    case 'd':
      point = 'd';
      SerialBT.println("point to kd");
      break;
    case 's': // 顯示 Kp、Ki、Kd
      SerialBT.print("Kp, Ki, Kd = ");
      SerialBT.print(pitchKp);
      SerialBT.print(" , ");
      SerialBT.print(pitchKi);
      SerialBT.print(" , ");
      SerialBT.println(pitchKd);
      break;
    case 'u': // 增加一級
      if(point == 'p') {
        pitchKp += rate;
        SerialBT.print("Kp = ");
        SerialBT.println(pitchKp);
      } else if(point == 'i') {
        pitchKi += rate;
        SerialBT.print("Ki = ");
        SerialBT.println(pitchKi);
      } else {
        pitchKd += rate;
        SerialBT.print("Kd = ");
        SerialBT.println(pitchKd);
      } 
      pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
      break;
    case 'j': // 減少一級
      if(point == 'p') {
        pitchKp -= rate;
        SerialBT.print("Kp = ");
        SerialBT.println(pitchKp);
      } else if(point == 'i') {
        pitchKi -= rate;
        SerialBT.print("Ki = ");
        SerialBT.println(pitchKi);
      } else {
        pitchKd -= rate;
        SerialBT.print("Kd = ");
        SerialBT.println(pitchKd);
      } 
      pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
      break;
    case '0': // reset
      if(point == 'p') {
        pitchKp = 0;
        SerialBT.println("reset Kp");
      } else if(point == 'i') {
        pitchKi = 0;
        SerialBT.println("reset Ki");
      } else {
        pitchKd = 0;
        SerialBT.println("reset Kd");
      } 
      pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
      break;
    case '1': // 0.1x
      rate = 0.1;
      SerialBT.println("rate = 0.1x");
      break;
    case '2': // 1.0x
      rate = 1.0;
      SerialBT.println("rate = 1.0x");
      break;
    case '3': // 10.0x
      rate = 10.0;
      SerialBT.println("rate = 10.0x");
      break;
    default : 
      Serial.println("Unknown command");
      break;
  }
}
void InitializeMPU6050() {
  mpu.initialize();     // Initialize MPU6050
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(XGyroOffset);
    mpu.setYGyroOffset(YGyroOffset);
    mpu.setZGyroOffset(ZGyroOffset);
    mpu.setZAccelOffset(ZAccelOffset);
    // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateAccel(6);
    //mpu.CalibrateGyro(6);
    //mpu.PrintActiveOffsets();
    
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    dmpReady = true;
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}
void InitialPID() {   // Initial pitch and yaw PID parameters
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(PID_SampleTime);
  pitchPID.SetOutputLimits(-255.0, 255.0);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetSampleTime(PID_SampleTime);
  yawPID.SetOutputLimits(-255.0, 255.0);
}
void InitializeGPIOpin() {  // Initialize GPIO pin
  pinMode(L298NIn1_pin, OUTPUT);
  pinMode(L298NIn2_pin, OUTPUT);
  pinMode(L298NIn3_pin, OUTPUT);
  pinMode(L298NIn4_pin, OUTPUT);
  pinMode(L298NEnableA_pin, OUTPUT);
  pinMode(L298NEnableB_pin, OUTPUT);
  digitalWrite(L298NIn1_pin,LOW);
  digitalWrite(L298NIn2_pin,LOW);
  digitalWrite(L298NIn3_pin,LOW);
  digitalWrite(L298NIn4_pin,LOW);
  digitalWrite(L298NEnableA_pin,LOW);
  digitalWrite(L298NEnableB_pin,LOW);
  // Configure LED PWM functionalitites for Motor Speed
  ledcSetup(rightChannel, 200, 8);
  ledcSetup(leftChannel, 200, 8);
//  ledcSetup(rightChannel, 1000, 8);
//  ledcSetup(leftChannel, 1000, 8);
  // Attach the channel to the GPIO to be controlled
  ledcAttachPin(L298NEnableA_pin, rightChannel);
  ledcAttachPin(L298NEnableB_pin, leftChannel);
}
void forward() {
  ledcWrite(rightChannel, rightPWM);    // 更新PWM輸出
  ledcWrite(leftChannel, leftPWM);
  digitalWrite(L298NIn1_pin, LOW);     digitalWrite(L298NIn2_pin, HIGH); // 右馬達
  digitalWrite(L298NIn3_pin, HIGH);    digitalWrite(L298NIn4_pin, LOW);  // 左馬達
//  if(rawPWM == maxPWM || rawPWM == -maxPWM)
//    delayMicroseconds(8000);
}
void backward() {
  ledcWrite(rightChannel, -rightPWM);    // 更新PWM輸出
  ledcWrite(leftChannel, -leftPWM);
  digitalWrite(L298NIn1_pin, HIGH);   digitalWrite(L298NIn2_pin, LOW); // 右馬達
  digitalWrite(L298NIn3_pin, LOW);    digitalWrite(L298NIn4_pin, HIGH);  // 左馬達
//  if(rawPWM == maxPWM || rawPWM == -maxPWM)
//    delayMicroseconds(8000);
}
void stop() {
  ledcWrite(rightChannel, 0);    // 更新PWM輸出
  ledcWrite(leftChannel, 0);
  digitalWrite(L298NIn1_pin, LOW);    digitalWrite(L298NIn2_pin, LOW);   // 右馬達停止
  digitalWrite(L298NIn3_pin, LOW);    digitalWrite(L298NIn4_pin, LOW);   // 左馬達停止
}