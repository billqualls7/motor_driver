/*
 * @Author: error: git config user.name && git config user.email & please set dead value or install git
 * @Date: 2023-01-22 16:50:22
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-02-04 16:29:53
 * @FilePath: \motor_driver\can_demp.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

typedef struct Driver_Message_TypeDef
{
    /* data */
    int ID ;
    float pos ;
    float vel ;
    float torque ;
   
};


/*
这几个参数没搞懂
*/
////////////////////////////////////////////////////////////////
#define P_MIN 0
#define P_MAX 65535

#define V_MIN 0
#define V_MAX 4096

#define C_MAX     1

#define KP_MAX 0
#define KD_MAX 0
////////////////////////////////////////////////////////////////
#define MOTOR_START                  0xFC
#define MOTOR_STOP                   0xFD
#define MOTOR_SET_ZERO_POSITION      0xFE
#define MOTOR_TORMODE                0xF9
#define MOTOR_VELMODE                0xFA
#define MOTOR_POSMODE                0xFB

// struct Driver_Message_TypeDef Driver_msgs ;


uint16_t float_to_uint(float v,float v_min,float v_max,uint32_t width)
{
    float temp;
    int32_t utemp;
    temp = ((v-v_min)/(v_max-v_min))*((float)width);
    utemp = (int32_t)temp;
    if(utemp < 0)
    utemp = 0;
    if(utemp > width)
    utemp = width;
    return utemp;
}



/**
 * @description: CAN答应返回六位，该函数进行解算
 * @param {int} *data  六位 
 * @param {int} Pmax Can 控制的最大圈数   
 * 无末端编码器时 Pmax = 1 时 Can 控制的最大圈数为+-1 圈  
 *  有末端编码器时 Pmax = 1 Can 控制的最大圈数为+- 0.5 圈,并且 Pmax 需固定配置为 1
 * @return {Driver_msgs_Rec}
 * 
 * 使用：
 * struct Driver_Message Driver_msgs_Rec=Can_Recevice_Data_Trans(data,1)
 * 
 */
struct Driver_Message Can_Recevice_Data_Trans(int *data,int Pmax)
{
    struct Driver_Message_TypeDef Driver_msgs_Rec
    Driver_msgs_Rec.ID = data[0] ;
    Driver_msgs_Rec.pos = (((data[1]<<8) | data[2])-32768)/32768*12.5*0.5*Pmax ; //单位 rad
    Driver_msgs_Rec.vel = (((data[3]<<8) | (data[4]&0xF0))-2048)/2048*65 ;       //单位 rad/s 
    Driver_msgs_Rec.torque = ((((data[4]&0x0F)<<4) | data[5])-2048)/2048*4 ;     // 单位安培  

    return Driver_msgs_Rec
} 


/**
 * @description: 对发送数据进行解算
 * @param {Driver_Message_TypeDef *} Driver_msgs_Set
 * @param {int} Pmax
 * @return {*}
 */
void Can_Set_Data(Driver_Message_TypeDef * Driver_msgs_Set,int Pmax)
{
    int pos_16,vel_16,tor_16;

    int Kp_int,Kd_int;

    int f_kp,f_kd ;

    int data[8]/////////////////////////////////////////////

    pos_16 = float_to_uint((Driver_msgs_Set.pos/Pmax/0.5/12.5*32768+32768),P_MIN, P_MAX, 65535) ;
    vel_16 = float_to_uint((Driver_msgs_Set.vel/65*2048-2048),V_MIN, V_MAX, 4096) ;
    tor_16 = float_to_uint((Driver_msgs_Set.torque/4*2048-2048),-C_MAX, C_MAX, 4096) ;

    Kp_int = float_to_uint(f_kp, 0 , KP_MAX, 4096);
    Kd_int = float_to_uint(f_kd, 0 , KD_MAX, 4096);
/////////////////////////////////////////////////////////////////////////////////////
    data[0] = pos_16>>8 ;              //指定位置，16 位数，范围 0-65536, 代表电机旋转到-12.5rad 到 12.5rad （-1*PMax 圈到+1*PMax 圈）范围内的指定位置，并保持
    data[1] = pos_16&0xFF ;            
    data[2] = vel_16>>4;       //指定速度，12 位数，范围 0-4096, 指定电机转速
    data[3] = ((vel_16&0xF)<<4) | (Kp_int>>8) ;         //Byte3(高四位) 指定速度 v 低 4 位  Byte3(低四位) 设定 Kp 高 4 位
    data[4] = Kp_int&0xFF ;         //设定位置环 PD 控制 P 值，12 位数，范围 0-4096   Byte4 设定 Kp 低 8 位
    data[5] = Kd_int>>4 ;         //设定位置环 PD 控制 D 值，12 位数，范围 0-4096
    data[6] = ((Kd_int&0xF)<<4) | (tor_16>>8) ;         //指定扭矩，12 位数，范围 0-4096, 指定电机恒定扭矩  Byte6(高四位) 设定 Kd 低 4 位   Byte6 (低四位) 指定扭矩 t 高 4 位
    data[7] = tor_16&0xFF ;

    // cansend(); 

}

/**
 * @description: 伪代码
 * @return {*}
 * Motor_Mode(MOTOR_START); //启动电机
 */
void Motor_Mode(uint8_t mode)
{

        data[0] = 0xFF;
        data[1] = 0xFF;
        data[2] = 0xFF;
        data[3] = 0xFF;
        data[4] = 0xFF;
        data[5] = 0xFF;
        data[6] = 0xFF;
        data[7] = mode;
       // cansend();    

}
