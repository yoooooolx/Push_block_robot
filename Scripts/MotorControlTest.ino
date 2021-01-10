/*
 * @Author: 尹云可
 * @Date: 2020-11-26 11:35:37
 * @LastEditors: 尹云可
 * @LastEditTime: 2021-01-10 14:57:18
 * @FilePath: \Scripts\MotorControlTest.ino
 * @Description:推方块机器人的主代码
 */

//引用的库
#include <TimerOne.h>
#include <AutoPID.h>

//计数器的值
volatile long counter_valL0 = 0;
volatile long counter_valL1 = 0;
volatile long counter_valR0 = 0;
volatile long counter_valR1 = 0; //该变量用于存储编码器的值，所以用类型修饰符volatile；
//volatile int timeIsrCounter = 0; //用于计量定时中断发生的次数

/**********电机控制IO口**********/
#define ENA 4
#define INA1 5
#define INA2 6
#define ENB 7
#define INB1 8
#define INB2 9
//左轮AB相为3、2口，右轮AB相为19、18口
//20、21为总线口（SCL、SDA）

//电机的两个EN口的状态
bool EN1 = true;
bool EN2 = true;

/***************PID相关************/

double inputSpeedLeft, speedLeft, outputSpeedLeft, inputSpeedRight, speedRight, outputSpeedRight; //轮子的输入速度、设定速度、输出速度

//PID参数

//输出值的上下限。PWM的输入值为0~255，正负号用于区分方向
#define OUTPUT_MIN_L (-6500)
#define OUTPUT_MAX_L 6500
#define OUTPUT_MIN_R (-6500)
#define OUTPUT_MAX_R 6500

//PID的K值，待调
#define KP_L 0.4    //0.3~0.4
#define KI_L 0.4    //1太大
#define KD_L 30
#define KP_R 0.4
#define KI_R 0.4
#define KD_R 30

//左右轮PID对象
AutoPID leftWheelPID(&inputSpeedLeft, &speedLeft, &outputSpeedLeft, OUTPUT_MIN_L, OUTPUT_MAX_L, KP_L, KI_L, KD_L);     //左轮PID对象
AutoPID rightWheelPID(&inputSpeedRight, &speedRight, &outputSpeedRight, OUTPUT_MIN_R, OUTPUT_MAX_R, KP_R, KI_R, KD_R); //右轮PID对象

/**********运动控制相关*********/
//运动状态枚举
enum motionStat
{
     forward,
     back,
     left,
     right,
     stop,
     idle
} motionStatus;

//运动状态定义
#define FORWARD 0
#define BACK 1
#define LEFT 2
#define RIGHT 3
#define STOP 4
#define IDLE 5

//rpm乘以此系数，即为应当输出的PWM值（可用性未知）
const double rpmToPWMFactor = 0.031875;

//左右电机正反转，1为正转，-1为反转，0为未知
volatile int motorDirectionL = 0;
volatile int motorDirectionR = 0;

//Setup函数
void setup()
{
     //delay(2000);
     pinMode(3, INPUT);
     pinMode(2, INPUT); //设置为输入模式，并且3,2号引脚是中断口2,3；用于左轮
     pinMode(19, INPUT);
     pinMode(18, INPUT); //设置为输入模式，并且19,18号引脚是中断口4,5；用于右轮

     pinMode(ENA, OUTPUT);
     pinMode(INA1, OUTPUT);
     pinMode(INA2, OUTPUT);
     pinMode(ENB, OUTPUT);
     pinMode(INB1, OUTPUT);
     pinMode(INB2, OUTPUT); //设置控制两个电机的两个EN口和四个IN口的输出模式。A是左轮，B是右轮

     Serial.begin(115200);                  //初始化波特率为115200
     attachInterrupt(2, counterL0, RISING); //设置编码器A相位上升沿中断
     attachInterrupt(3, counterL1, RISING); //设置编码器B相位上升沿中断
     attachInterrupt(4, counterR0, RISING); //设置编码器A相位上升沿中断
     attachInterrupt(5, counterR1, RISING); //设置编码器B相位上升沿中断

     Timer1.initialize(100000);        // 设置定时器中断时间，单位微秒 ，这里是50毫秒
     Timer1.attachInterrupt(timerIsr); // 打开定时器中断

     interrupts(); //打开外部中断

     //初始化枚举
     motionStatus = idle;

     //PID每500ms计算一次
     leftWheelPID.setTimeStep(500);
     rightWheelPID.setTimeStep(500);
}

//Loop函数
void loop()
{
     motionControl(FORWARD, 6500);
     delay(5000);
     motionControl(BACK, 4500);
     delay(5000);
     motionControl(STOP, 0);
     delay(2000);
     motionControl(LEFT, 3000);
     delay(5000);
     motionControl(RIGHT, 5500);
     delay(5000);
     motionControl(IDLE, 0);
     delay(5000);
}

//外部中断处理函数

/******************************************************************
当B相在上升沿时，若A相为1，则为顺时针旋转；若A相为0，则为逆时针旋转
******************************************************************/

void counterL0() //左轮电机A相上升沿输入
{
     counter_valL0 += 1; //每一个中断加一
}
void counterL1() //左轮电机B相上升沿输入
{
     //此时若A相为1，则为顺时针转动，即正转
     if (digitalRead(3) == 1)
     {
          motorDirectionL = -1;
     }
     else
     {
          motorDirectionL = 1;
     }
     counter_valL1 += motorDirectionL; //每一个中断加一
}

void counterR0() //右轮电机A相上升沿输入
{
     counter_valR0 += 1; //每一个中断加一
}
void counterR1() //右轮电机B相上升沿输入
{
     //此时若A相为1，则为顺时针转动，即反转
     if (digitalRead(19) == 1)
     {
          motorDirectionR = -1;
     }
     else
     {
          motorDirectionR = 1;
     }
     counter_valR1 += motorDirectionR; //每一个中断加一
}

//定时器中断处理函数
void timerIsr()
{
     inputSpeedLeft = 60.0 * (double)counter_valL1 / (11.0 * 0.1); //这里的单位是转每分钟：rpm

     /*Serial.print("左轮当前的速度是：");
     Serial.print(inputSpeedLeft);
     Serial.println("rpm");
     Serial.print("左轮设定的速度是：");
     Serial.print(speedLeft);
     Serial.println("rpm");
     Serial.print("左轮输出的速度是：");
     Serial.print(outputSpeedLeft);
     Serial.println("rpm");*/

     Serial.println(inputSpeedLeft);

     counter_valL0 = 0;
     counter_valL1 = 0; //清空该时间段内的脉冲数

     inputSpeedRight = 60.0 * (double)counter_valR1 / (11.0 * 0.1); //这里的单位是转每分钟：rpm

     /*Serial.print("右轮当前的速度是：");
     Serial.print(inputSpeedRight);
     Serial.println("rpm");
     Serial.print("右轮设定的速度是：");
     Serial.print(speedRight);
     Serial.println("rpm");
     Serial.print("右轮输出的速度是：");
     Serial.print(outputSpeedRight);
     Serial.println("rpm");*/

     //Serial.println(inputSpeedRight);

     counter_valR0 = 0;
     counter_valR1 = 0; //清空该时间段内的脉冲数

     //counter_valL0和counter_valR0检测的是轮子的速率，暂时用不到

     //尝试进行一次PID计算
     motorWrite();

     /*//每执行5次本函数
     timeIsrCounter += 1;
     if (timeIsrCounter >= 5)
     {
          //清空计数
          timeIsrCounter = 0;

          //输出数据
          Serial.print("左轮当前的速度是：");
          Serial.print(inputSpeedLeft);
          Serial.println("rpm");
          Serial.print("左轮设定的速度是：");
          Serial.print(speedLeft);
          Serial.println("rpm");
          Serial.print("左轮输出的速度是：");
          Serial.print(outputSpeedLeft);
          Serial.println("rpm");

          Serial.print("右轮当前的速度是：");
          Serial.print(inputSpeedRight);
          Serial.println("rpm");
          Serial.print("右轮设定的速度是：");
          Serial.print(speedRight);
          Serial.println("rpm");
          Serial.print("右轮输出的速度是：");
          Serial.print(outputSpeedRight);
          Serial.println("rpm");
     }*/
}

//控制电机
void motorWrite()
{
     int _ina1, _ina2, _inb1, _inb2, _ena, _enb;
     double _outputSpeedLeft = outputSpeedLeft * rpmToPWMFactor;
     double _outputSpeedRight = outputSpeedRight * rpmToPWMFactor;
     //计算PID
     leftWheelPID.run();
     rightWheelPID.run();
     //设定电机的IN值
     if (_outputSpeedLeft >= 0.001)
     {
          if (_outputSpeedLeft > 254.999)
          {
               _ina2 = 255;
          }
          else
          {
               _ina2 = (int)_outputSpeedLeft;
          }
          _ina1 = 0;
     }
     else if (_outputSpeedLeft <= -0.001)
     {
          if (_outputSpeedLeft < -254.999)
          {
               _ina1 = 255;
          }
          else
          {
               _ina1 = -(int)_outputSpeedLeft;
          }
          _ina2 = 0;
     }
     else
     {
          _ina1 = 0;
          _ina2 = 0;
     }

     if (_outputSpeedRight >= 0.001)
     {
          if (_outputSpeedRight > 254.999)
          {
               _inb2 = 255;
          }
          else
          {
               _inb2 = (int)_outputSpeedRight;
          }
          _inb1 = 0;
     }
     else if (_outputSpeedRight <= -0.001)
     {
          if (_outputSpeedRight < -254.999)
          {
               _inb1 = 255;
          }
          else
          {
               _inb1 = -(int)_outputSpeedRight;
          }
          _inb2 = 0;
     }
     else
     {
          _inb1 = 0;
          _inb2 = 0;
     }

     //设定电机的EN值
     if (EN1)
     {
          _ena = HIGH;
     }
     else
     {
          _ena = LOW;
     }
     if (EN2)
     {
          _enb = HIGH;
     }
     else
     {
          _enb = LOW;
     }

     //写入值
     digitalWrite(ENA, _ena);
     digitalWrite(ENB, _enb);
     analogWrite(INA1, _ina1);
     analogWrite(INA2, _ina2);
     analogWrite(INB1, _inb1);
     analogWrite(INB2, _inb2);
}

//控制运动
void motionControl(int _motionStatus, double _velocity) //_velocity为想要的实际速度，单位m/s
{
     switch (_motionStatus) //speedLeft和speedRight单位是rpm
     {
     case FORWARD: //以给定速度前进
          motionStatus = forward;
          EN1 = true;
          EN2 = true;
          speedLeft = _velocity;
          speedRight = _velocity;
          break;

     case BACK: //以给定速度后退
          motionStatus = back;
          EN1 = true;
          EN2 = true;
          speedLeft = -_velocity;
          speedRight = -_velocity;
          break;

     case LEFT: //以给定速度左转（运动轨迹不确定）
          motionStatus = left;
          EN1 = true;
          EN2 = true;
          speedLeft = -_velocity;
          speedRight = _velocity;
          break;

     case RIGHT: //以给定速度右转（运动轨迹不确定）
          motionStatus = right;
          EN1 = true;
          EN2 = true;
          speedLeft = _velocity;
          speedRight = -_velocity;
          break;

     case STOP: //原地刹车。准备改成直接刹车，不通过PID
          motionStatus = stop;
          EN1 = true;
          EN2 = true;
          speedLeft = 0.0;
          speedRight = 0.0;
          break;

     case IDLE: //原地自由转动。准备改成直接自由转动，不通过PID
          motionStatus = idle;
          EN1 = false;
          EN2 = false;
          speedLeft = 0.0;
          speedRight = 0.0;
          break;
     }

     //数据写入电机
     motorWrite();
}