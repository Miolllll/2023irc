#include "mbed.h"
#include "GP2A.h"
#include <iterator>
#include <string>
#include "Thread.h"
#include "rtos.h"
#include "MPU6050.h"
#include <math.h>

//----------------------------제어주기-----------------------

#define MCU_CONTROL_RATE 10
//#define serial_UPDATE_RATE 10
#define SENSER_UPDATE_RATE 10
#define IMU_UPDATE_RATE 10
#define PRINT_RATE 100

//----------------------------스레드 우선순위------------------------

//Thread serial_thread(osPriorityRealtime3);
Thread SENSER_thread(osPriorityRealtime);
Thread IMU_thread(osPriorityHigh);
Thread PRINT_thread(osPriorityAboveNormal);

//--------------------------------IMU 설정------------------------------

#define pi 3.141592654

float dt = 0;
float ARoll, APitch, AYaw = 0;//가속도 센서 롤, 피치, 요
float GRoll, GPitch, GYaw = 0;//자이로 센서 롤, 피치, 요
float gyroX, gyroY, gyroZ = 0;
float accelX, accelY, accelZ = 0;
float lgyroX, lgyroY, lgyroZ = 0;
float FPitch = 0;//필터 적용된 피치값
float FRoll = 0;//필터 적용된 롤값

float Pmin = -7;
float Pmax = 7;
float Rmin = -7;
float Rmax = 7;

MPU6050 mpu6050;

//---------------------------------------------------------------------------

int mode = 0;
int state = 0;

RawSerial raspi(PA_9, PA_10, 115200);//시리얼 통신 세팅 값
RawSerial pc(USBTX, USBRX, 115200);

volatile bool All_move = false;
volatile bool gotPacket = false;
volatile bool imu_bool = false;
volatile bool pi_stop = false;

int i = 0;
int degree;
int dist;
double N;
double D;


// [센서 정의 + 객체 선언]// ir 센서---------------------------------------------------------------------------
AnalogIn ir0(D13);
AnalogIn ir1(A0);
AnalogIn ir2(D12);
AnalogIn ir3(D11);
AnalogIn ir4(A1);
AnalogIn ir5(A2);
AnalogIn ir6(A3);
AnalogIn ir7(A4);
AnalogIn ir8(A5);

uint16_t ir_val0;
uint16_t ir_val1;
uint16_t ir_val2;
uint16_t ir_val3;
uint16_t ir_val4;
uint16_t ir_val5;
uint16_t ir_val6;
uint16_t ir_val7;
uint16_t ir_val8;

int val0 = 20000;
int val1 = 20000;
int val2 = 20000;
int val3 = 20000;
int val4 = 20000;
int val5 = 20000;
int val6 = 20000;
int val7 = 20000;
int val8 = 20000;

double pval1 = 80;
double pval2 = 20;
double pval3 = 80;
double pval4 = 22;

GP2A psd1(PC_5, 30, 150, 60, 0); // 핀 번호 선언 - AnalogIn
GP2A psd2(PC_4, 0.07, 0.8, 0.23625, -0.297); // 핀 번호 선언 - AnalogIn
GP2A psd3(PC_2, 30, 150, 60, 0); // 핀 번호 선언 - AnalogIn
GP2A psd4(PC_3, 0.07, 0.8, 0.23625, -0.297); // 핀 번호 선언 - AnalogIn

bool ir_Wh[8];
// 0 : ml + fl 앞 왼 바퀴
// 1 : fr + mr 앞 오 바퀴
// 2 : ml + bl 뒤 왼 바퀴
// 3 : br + mr 뒤 오 바퀴
// 4 : 앞 ir 전체
// 5 : 뒤 ir 전체
// 6 : 중간 앞 ir 전체
// 7 : 중간 뒤 ir 전체

DigitalOut DirL(D7);
DigitalOut DirR(D4);
PwmOut pwmL(D6);
PwmOut pwmR(D5);


#define SIZE 5//이동평균필터 샘플링 데이터 갯수->클수록 필터링이 많이 되며 딜레이 커짐
double buffer1[SIZE];
double buffer2[SIZE];
double buffer3[SIZE];
double buffer4[SIZE];
double sum[4];
double sensorValue[4];
double EMA_a = 0.1;      //LOW PASS FILTER 필터링 상수 ->작을수록 필터링이 많이 되며 딜레이 커짐
double EMA_S_pre[4];
double EMA_S[4];

double psd_val1;
double psd_val2;
double psd_val3;
double psd_val4;

double psdf_val1;
double psdf_val2;
double psdf_val3;
double psdf_val4;

// [함수 정의]
void psd_sensor_read(){ // 센서값 읽기
    // psd_val1 = psd1.getDistance(); // psd값 읽기 - AnalogIn
    // psd_val2 = psd2.getDistance()*100;
    // psd_val3 = psd3.getDistance();
    // psd_val4 = psd4.getDistance()*100;

    psdf_val1 = psd1.getDistance(); // psd값 읽기 - AnalogIn
    psdf_val2 = psd2.getDistance()*100;
    psdf_val3 = psd3.getDistance();
    psdf_val4 = psd4.getDistance()*100;
}

void setup() {
    EMA_S_pre[0] = psdf_val1;  //초기값 설정
    EMA_S_pre[1] = psdf_val2;
    EMA_S_pre[2] = psdf_val3;
    EMA_S_pre[3] = psdf_val4;
}

void psd_sensor_run() {

    sensorValue[0] = psdf_val1;
    sensorValue[1] = psdf_val2;
    sensorValue[2] = psdf_val3;
    sensorValue[3] = psdf_val4;

    EMA_S_pre[0] = (EMA_a*sensorValue[0]) + ((1-EMA_a)*EMA_S_pre[0]);
    EMA_S_pre[1] = (EMA_a*sensorValue[1]) + ((1-EMA_a)*EMA_S_pre[1]);
    EMA_S_pre[2] = (EMA_a*sensorValue[2]) + ((1-EMA_a)*EMA_S_pre[2]);
    EMA_S_pre[3] = (EMA_a*sensorValue[3]) + ((1-EMA_a)*EMA_S_pre[3]);

    //이동평균 필터 적용////////////////////////////

    // sum[0]-=buffer1[0];
    // sum[1]-=buffer2[0];
    // sum[2]-=buffer3[0];
    // sum[3]-=buffer4[0];

    // for(int i =0; i<SIZE-1 ; i++)
    // { 
    //     buffer1[i]=buffer1[i+1];
    //     buffer2[i]=buffer2[i+1];
    //     buffer3[i]=buffer3[i+1];
    //     buffer4[i]=buffer4[i+1];
    // }

    // buffer1[SIZE-1]=EMA_S_pre[0];
    // buffer2[SIZE-1]=EMA_S_pre[1];
    // buffer3[SIZE-1]=EMA_S_pre[2];
    // buffer4[SIZE-1]=EMA_S_pre[3];

    // sum[0]+=buffer1[SIZE-1];
    // sum[1]+=buffer2[SIZE-1];
    // sum[2]+=buffer3[SIZE-1];
    // sum[3]+=buffer4[SIZE-1];

    // EMA_S[0]=sum[0]/(SIZE); //이동평균필터를 통해 최종 필터링한 센서값 도출(EMA_S)
    // EMA_S[1]=sum[1]/(SIZE);
    // EMA_S[2]=sum[2]/(SIZE);
    // EMA_S[3]=sum[3]/(SIZE);
    
    // psd_val1 = EMA_S[0];
    // psd_val2 = EMA_S[1];
    // psd_val3 = EMA_S[2];
    // psd_val4 = EMA_S[3];

    psd_val1 = EMA_S_pre[0];
    psd_val2 = EMA_S_pre[1];
    psd_val3 = EMA_S_pre[2];
    psd_val4 = EMA_S_pre[3];
}

void ir_senser_read() {
    ir_val0 = ir0.read_u16(); // IR값 읽기 - AnalogIn
    ir_val1 = ir1.read_u16(); // IR값 읽기 - AnalogIn
    ir_val2 = ir2.read_u16(); // IR값 읽기 - AnalogIn
    ir_val3 = ir3.read_u16(); // IR값 읽기 - AnalogIn
    ir_val4 = ir4.read_u16(); // IR값 읽기 - AnalogIn
    ir_val5 = ir5.read_u16(); // IR값 읽기 - AnalogIn
    ir_val6 = ir6.read_u16(); // IR값 읽기 - AnalogIn
    ir_val7 = ir7.read_u16(); // IR값 읽기 - AnalogIn
    ir_val8 = ir8.read_u16(); // IR값 읽기 - AnalogIn
}


void ir_senser_cal() {
    if (ir_val0 < val0 && ir_val8 < val8) { // 앞 왼
            ir_Wh[0] = true;
        }
    else ir_Wh[0] = false;

    if (ir_val2 < val2 && ir_val3 < val3) { // 앞 오
        ir_Wh[1] = true;
    }
    else ir_Wh[1] = false;

    if (ir_val6 < val6 && ir_val7 < val7) { // 뒤 왼
        ir_Wh[2] = true;
    }
    else ir_Wh[2] = false;

    if (ir_val4 < val4 && ir_val5 < val5) { // 뒤 오
        ir_Wh[3] = true;
    }
    else ir_Wh[3] = false;

    if (ir_val0 < val0 && ir_val2 < val2) { // 앞
        ir_Wh[4] = true;
    }
    else ir_Wh[4] = false;

    if (ir_val5 < val5 && ir_val6 < val6) { // 뒤
        ir_Wh[5] = true;
    }
    else ir_Wh[5] = false;

    if (ir_val3 < val3 && ir_val8 < val8) { // 중간 앞
        ir_Wh[6] = true;
    }
    else ir_Wh[6] = false;

    if (ir_val4 < val4 && ir_val7 < val7) { // 중간 뒤
        ir_Wh[7] = true;
    }
    else ir_Wh[7] = false;
}

void ir_senser_run() {
    ir_senser_read();
    ir_senser_cal();
}

//타이머 설정---------------------------------------------------------------------------

float t;
float t_bool;
float t_imu;

Timer timer1;
Timer timer2;
Timer timer3;
Timer timer4;

void timerST () {
    timer1.start();  // 타이머 시작
    t = timer1.read_ms();  // 경과 시간 계산
}

void booltimerST () {
    timer2.start();  // 타이머 시작
    t_bool = timer2.read_ms();  // 경과 시간 계산
}

void imu_timerST () {
    timer3.start();  // 타이머 시작
    t_imu = timer3.read_ms();  // 경과 시간 계산
}

void timerEND () {
    timer1.stop();  // 타이머 정지
    timer1.reset(); // 타이머 초기화
    t = 0;
}

void booltimerEND () {
    timer2.stop();  // 타이머 정지
    timer2.reset(); // 타이머 초기화
    t_bool = 0;
}

void imu_timerEND () {
    timer3.stop();  // 타이머 정지
    timer3.reset(); // 타이머 초기화
    t_imu = 0;
}


void moter (int dirleft , int dirright , double pwmleft , double pwmright) {
    DirL = dirleft;
    DirR = dirright;
    pwmL = pwmleft;
    pwmR = pwmright;

    if (pwmleft <= -1) {
        dirleft = 1;
        pwmleft = 1;
    }
    else if (pwmleft < 0 && pwmleft > -1) {
        dirleft = 1;
        abs(pwmleft);
    }
    else if (pwmleft >= 1) {
        dirleft = 0;
        pwmleft = 1;
    }
    else {
        dirleft = 0;
    }

    if (pwmright <= -1) {
        dirright = 1;
        pwmright = 1;
    }
    else if (pwmright < 0 && pwmright > -1) {
        dirright = 1;
        abs(pwmright);
    }
    else if (pwmright >= 1) {
        dirright = 0;
        pwmright = 1;
    }
    else {
        dirright = 0;
    }
}

// void Timer1() {
//     timer1.start();  // 타이머 시작
//     while(1) {
//         t = timer1.read_ms();  // 경과 시간 계산
//         //printf("Elapsed time: %f milliseconds\n", t);
//     }
// }


//쓰레드 설정-----------------------------------------------------------------
void IMU_thread_loop(){
    
    uint64_t Now_IMU,Work_IMU;

    //Set up I2C
    i2c.frequency(400000);  // use fast (400 kHz) I2C

    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    pc.printf("I AM 0x%x\n\r", whoami);
    pc.printf("I SHOULD BE 0x68\n\r");

    if (whoami == 0x68){
        pc.printf("MPU6050 is online...");
        wait(1);

        mpu6050.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values

        if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
            mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
            mpu6050.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
            mpu6050.initMPU6050();
            pc.printf("MPU6050 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

            pc.printf("\nMPU6050 passed self test... Initializing");
            wait(2);
        }
        else pc.printf("\nDevice did not the pass self-test!\n\r");
    }
    else {
        pc.printf("Could not connect to MPU6050: \n\r");
        pc.printf("%#x \n",  whoami);
        while(1) ; // Loop forever if communication doesn't happen
    }

    while(1){
        Now_IMU=rtos::Kernel::get_ms_count();

        //pc.printf("IMU   \n");

        //If data ready bit set, all data registers have new data
        if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
            mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values
            mpu6050.readAccelData(accelCount);
            mpu6050.getGres();
            mpu6050.getAres();

            gyroY = (float)gyroCount[0]*gRes;
            gyroX = (float)gyroCount[1]*gRes;
            gyroZ = (float)gyroCount[2]*gRes; // - gyroBias[2];
            
            accelX = (float)accelCount[0]*aRes;
            accelY = (float)accelCount[1]*aRes;
            accelZ = (float)accelCount[2]*aRes;
        }

        ARoll = (180/pi)*(atan(accelX/(sqrt((accelY*accelY)+(accelZ*accelZ))))) - 0.4;
        APitch = (180/pi)*(atan(accelY/(sqrt((accelX*accelX)+(accelZ*accelZ))))) + 1.7;
        AYaw = (180/pi)*(atan((sqrt((accelX*accelX)+(accelY*accelY)))/accelZ)) - 3.93;

        GYaw += ((lgyroZ+gyroZ)/2)*0.01*2;
        GPitch += ((lgyroY+gyroY)/2)*0.01*2;
        FPitch = (0.9*(FPitch+(((lgyroY+gyroY)/2)*2*0.01)))+0.1*APitch;//complementary fillter
        FRoll = (0.9*(FRoll+(((lgyroX+gyroX)/2)*2*0.01)))+0.1*ARoll;//complementary fillter
        lgyroZ = gyroZ;
        lgyroY = gyroY;
        // pc.printf("pitch: %f  ",FPitch);
        // pc.printf("roll: %f\n",FRoll);
        
        Work_IMU=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(IMU_UPDATE_RATE-(Work_IMU-Now_IMU)));
    }
}

void imu_cal() {
    if (FPitch < Pmin || FPitch > Pmax || FRoll < Rmin || FRoll > Rmax) {
        imu_timerST();
        if (t_imu > 1000) {
            imu_bool = true;
            //imu_timerEND();
            //pc.printf("imu on\n");
        }
    }
    else {
        imu_bool = false;
        imu_timerEND();
        //pc.printf("imu off\n");
    }
}

// void SENSER_thread_loop(){

//     uint64_t Now_IR,Work_IR;
//     psd_sensor_read(); // 센서값 읽기
//     setup();

//     while(1){
//         Now_IR = rtos::Kernel::get_ms_count();

//         //pc.printf("SENSER   \n");

//         ir_senser_run();
//         psd_sensor_read(); // 센서값 읽기
//         psd_sensor_run();

//         Work_IR=rtos::Kernel::get_ms_count();
//         ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(SENSER_UPDATE_RATE-(Work_IR-Now_IR)));
//     }
// }


void print_thread_loop(){
    
    uint64_t Now_p,Work_p;
    
    while(1){
        Now_p=rtos::Kernel::get_ms_count();

        //pc.printf("print\n   ");

        // pc.printf("pitch: %f\n",FPitch);
        // pc.printf("roll: %f\n",FRoll);

        // pc.printf("%d  ", ir_val0);
        // pc.printf("%d  ", ir_val1);
        // pc.printf("%d  ", ir_val2);
        // pc.printf("%d  ", ir_val3);
        // pc.printf("%d  ", ir_val4);
        // pc.printf("%d  ", ir_val5);
        // pc.printf("%d  ", ir_val6);
        // pc.printf("%d  ", ir_val7);
        // pc.printf("%d\n", ir_val8);

        // pc.printf("%lf   ", psd_val1);  // 필터링 된 psd 값
        // pc.printf("%lf   ", psd_val2);
        // pc.printf("%lf   ", psd_val3);
        // pc.printf("%lf\n", psd_val4);

        // pc.printf("%lf   ", psdf_val1);   // 필터링 안된 psd 값
        // pc.printf("%lf   ", psdf_val2);
        // pc.printf("%lf   ", psdf_val3);
        // pc.printf("%lf\n", psdf_val4);

        
        Work_p=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(PRINT_RATE-(Now_p-Work_p)));
    }
}

//-----------------------------------------------------------------------------------------------------------------------


int hex2dec(char hex1, char hex2) {//hex1=buf[0], hex2=buf[1]
    int first;
    int second;
    char hex[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

    if(hex1==hex[0]) first = 0;
    else if(hex1==hex[1]) first = 1;
    else if(hex1==hex[2]) first = 2;
    else if(hex1==hex[3]) first = 3;
    else if(hex1==hex[4]) first = 4;
    else if(hex1==hex[5]) first = 5;
    else if(hex1==hex[6]) first = 6;
    else if(hex1==hex[7]) first = 7;
    else if(hex1==hex[8]) first = 8;
    else if(hex1==hex[9]) first = 9;
    else if(hex1==hex[10]) first = 10;
    else if(hex1==hex[11]) first = 11;
    else if(hex1==hex[12]) first = 12;
    else if(hex1==hex[13]) first = 13;
    else if(hex1==hex[14]) first = 14;
    else if(hex1==hex[15]) first = 15;

    if(hex2==hex[0]) second = 0;
    else if(hex2==hex[1]) second = 1;
    else if(hex2==hex[2]) second = 2;
    else if(hex2==hex[3]) second = 3;
    else if(hex2==hex[4]) second = 4;
    else if(hex2==hex[5]) second = 5;
    else if(hex2==hex[6]) second = 6;
    else if(hex2==hex[7]) second = 7;
    else if(hex2==hex[8]) second = 8;
    else if(hex2==hex[9]) second = 9;
    else if(hex2==hex[10]) second = 10;
    else if(hex2==hex[11]) second = 11;
    else if(hex2==hex[12]) second = 12;
    else if(hex2==hex[13]) second = 13;
    else if(hex2==hex[14]) second = 14;
    else if(hex2==hex[15]) second = 15;

    int cur_degree = first*16 + second;

    return cur_degree;
}


// void serial_thread_loop() {
//     int i = 0;
//     int degree;
//     int dist;
//     double N;
//     double D;
//     if (raspi.readable()) {//시리얼 통신 문자열 루프백
//         char buf[16];
//         char c;
//         c = raspi.getc();
//         if (c == '#') {
//             if (i>0){
//                 buf[i] = '\0';
//                 if(i == 4){
//                     //pc.printf("%s\n",buf);
//                     degree = hex2dec(buf[0],buf[1]);
//                     dist = hex2dec(buf[2],buf[3]);
//                     pc.printf("%d   ",degree);
//                     pc.printf("%d\n",dist);
//                 }
//             }
//             i = 0;
//         }
//         else {
//                 buf[i] = c;
//                 i++;
//         }
        

//         N = (degree-90.0f)/90.0f;
//         D = (180.0f-dist)/180.0f;
//     }
// }

void in_SerialRx_main(){ // interrupt 전용
    if(gotPacket == true) {
        gotPacket = false;
        All_move = true;
    }
}

void in_SerialRx(){ // interrupt 전용 - thread 사용안함

    if (raspi.readable()) {//시리얼 통신 문자열 루프백
        char buf[16];
        char c;
        c = raspi.getc();
        if (c == '#') {
            if (i>0){
                buf[i] = '\0';
                if(i == 4){
                    //pc.printf("%s\n",buf);
                    degree = hex2dec(buf[0],buf[1]);
                    dist = hex2dec(buf[2],buf[3]);
                    //pc.printf("%d   ",degree);
                    //pc.printf("%d\n",dist);
                    gotPacket = true;
                }
            }
            i = 0;
        }
        else {
            buf[i] = c;
            i++;
        }
    }
}

void bool_cal(){
    if(All_move == false) {
        if (t_bool > 1000) {
            pi_stop = true;
            booltimerEND();
        }
        else {
            booltimerST();
            //moter(0,0,0,0);
        }
    }
}

// void th_SerialRx(){ // thread 전용

//     uint64_t Now_s,Work_s;

//     // int i = 0;
//     // int degree;
//     // int dist;
//     // double N;
//     // double D;
    
//     while(1){
//         Now_s = rtos::Kernel::get_ms_count();
//         if (raspi.readable()) {  //시리얼 통신 문자열 루프백
//             char buf[16];
//             char c;
//             c = raspi.getc();
//             if (c == '#') {
//                 if (i>0){
//                     buf[i] = '\0';
//                     if(i == 4){
//                         //pc.printf("%s\n",buf);
//                         degree = hex2dec(buf[0],buf[1]);
//                         dist = hex2dec(buf[2],buf[3]);
//                         pc.printf("%d   ",degree);
//                         pc.printf("%d\n",dist);
//                         gotPacket = true;
//                     }
//                 }
//                 i = 0;
//             }
//             else {
//                 buf[i] = c;
//                 i++;
//             }
//         }
//         N = (degree-90.0f)/90.0f;
//         D = (180.0f-dist)/180.0f;

//         Work_s=rtos::Kernel::get_ms_count();
//         ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(serial_UPDATE_RATE-(Now_s-Work_s)));
//     }
// }

int main() {

    osThreadSetPriority(osThreadGetId(),osPriorityRealtime7);

    pwmL.period_us(66); // 15KHz
    pwmR.period_us(66); // 15KHz

    // int i = 0;
    // int degree;
    // int dist;
    // double N;
    // double D;

    int DL=0;
    int DR=0;
    double PL;
    double PR;

    psd_sensor_read(); // 센서값 읽기
    setup();

    uint64_t Now_M,Work_M;
    
    //serial_thread.start(&th_SerialRx);

    //SENSER_thread.start(&SENSER_thread_loop);
    IMU_thread.start(&IMU_thread_loop);
    //PRINT_thread.start(&print_thread_loop);

    raspi.attach(&in_SerialRx); // interrupt 전용

    //timer4.start();

    while(1) {
        Now_M = rtos::Kernel::get_ms_count();

        //timer4.reset();

        in_SerialRx_main(); // interrupt 전용

        imu_cal(); // imu 상태

        bool_cal();

        ir_senser_run();
        psd_sensor_read(); // 센서값 읽기
        psd_sensor_run();

        if (All_move == true) {

            pi_stop = false;
            booltimerEND();

            //pc.printf("pi run\n");
            //pc.printf("%d\n", mode);

            // if (raspi.readable()) {//시리얼 통신 문자열 루프백
            //     char buf[16];
            //     char c;
            //     c = raspi.getc();
            //     if (c == '#') {
            //         if (i>0){
            //             buf[i] = '\0';
            //             if(i == 4){
            //                 //pc.printf("%s\n",buf);
            //                 degree = hex2dec(buf[0],buf[1]);
            //                 dist = hex2dec(buf[2],buf[3]);
            //                 pc.printf("%d   ",degree);
            //                 pc.printf("%d\n",dist);
            //             }
            //         }
            //         i = 0;
            //     }
            //     else {
            //             buf[i] = c;
            //             i++;
            //     }
            // }

            // N = (degree-90.0f)/90.0f;
            // D = (180.0f-dist)/180.0f;
            //pc.printf("%f\n", N);

//--------------------------------------------------------------------------------------------------------------------

            // if(button == 0) {  // 버튼 누르면 시작
            //     mode = 0;
            // }

//--------------------------------------------------------------------------------------------------------------------

            if (mode == 0) {    //처음 시작 회전
                moter(0, 1, 0.4, 0.4);
                if (dist != 255) {
                    if (85 <= degree && degree <= 95){
                        mode = 50;
                    }
                    else {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

            else if (mode == 50) {
                if (imu_bool == true) {
                    mode = 100;
                }
                else if (ir_val3 < val3 || ir_val8 < val8) {
                    //mode = 34;
                    mode = 35;
                }
                else {
                    moter(0, 0, 0.3, 0.3);
                }
            }

    //---------------------------------------------------------------------------------------------------------------------
            
            else if (mode == 1) {   //검은색 영역 상대 공격, 주행

                //pre_mode = 1;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if (ir_Wh[0] == true || ir_Wh[1] == true || ir_Wh[2] == true || ir_Wh[3] == true || ir_Wh[4] == true || ir_Wh[5] == true) {
                    mode = 2;
                }

                else if (dist == 255) {  //상대 안보임
                    if (degree < 90) {
                        moter(1, 0, 0.3, 0.3);
                    }

                    else if (degree >= 90) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if (65<=dist && dist != 255) {  //상대 작음, 보통

                    // PL = 0.4+N*1.6+D*0.3;
                    // PR = 0.4-N*1.6+D*0.3;
                    // moter(DL, DR, PL, PR);

                    if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }

                    else if (40 <= degree && degree < 140) {
                        PL = 0.3+N*0.25+D*0.3;
                        PR = 0.3-N*0.25+D*0.3;
                        moter(DL, DR, PL, PR);
                    }

                    else if (140 <= degree) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if (30<=dist && dist<65) {  //상대 큼
                    if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }

                    else if (40 <= degree && degree < 75) {
                        PL = 0.3;
                        PR = 0.3-N*0.55;
                        moter(0, 0, PL, PR);
                    }

                    else if (75 <= degree && degree < 105) {
                        moter(0, 0, 0.4, 0.4);
                    }

                    else if (105 <= degree && degree < 140) {
                        PL = 0.3+N*0.55;
                        PR = 0.3;
                        moter(0, 0, PL, PR);
                    }

                    else if (140 <= degree) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if (dist<30) {  //상대 매우 큼
                    if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }

                    else if (70 <= degree && degree < 110) {
                        moter(0, 0, 0.5, 0.5);
                    }

                    else if (110 <= degree) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //-----------------------------------------------------------------------------------------------------------------------------
            
            else if (mode == 2) {   //색영역 코드

                //pre_mode = 2;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0] == true && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    if(ir_Wh[4] == false) {  // 앞 왼 바퀴
                        mode = 3;
                    }
                    else if(ir_Wh[4] == true) {  // 앞왼 + 앞ir
                        mode = 4;
                    }
                }

                else if(ir_Wh[0] == false && ir_Wh[1] == true && ir_Wh[2] == false && ir_Wh[3] == false) {
                    if(ir_Wh[4] == false) {  // 앞 오 바퀴
                        mode = 5;
                    }
                    else if(ir_Wh[4] == true) {  // 앞오 + 앞ir
                        mode = 6;
                    }
                }

                else if(ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == true && ir_Wh[3] == false) {
                    if(ir_Wh[5] == false) {  // 뒤 왼 바퀴
                        mode = 7;
                    }
                    else if(ir_Wh[5] == true) {  // 뒤왼 + 뒤ir
                        mode = 8;
                    }
                }

                else if(ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == true) {
                    if(ir_Wh[5] == false) {  // 뒤 오 바퀴
                        mode = 9;
                    }
                    else if(ir_Wh[5] == true) {  // 뒤오 + 뒤ir
                        mode = 10;
                    }
                }

                else if(ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false &&ir_Wh[3] == false) {
                    if(ir_Wh[4] == true && ir_Wh[5] == false) {  // 앞 ir 전체
                        mode = 11;
                    }
                    else if(ir_Wh[4] == false && ir_Wh[5] == true) {  // 뒤 ir 전체
                        mode = 12;
                    }
                    else if(ir_Wh[4] == false && ir_Wh[5] == false) {  // 검은색 영역
                        mode = 1;
                    }
                    else {
                        mode = 31;
                    }
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2] == false && ir_Wh[3] == false) { // 앞바퀴 2개
                    mode = 13;
                }

                else if(ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2]== true && ir_Wh[3]== true) { // 뒷바퀴 2개
                    mode = 14;
                }

                else if(ir_Wh[0]== true && ir_Wh[1] == false && ir_Wh[2]== true && ir_Wh[3] == false) {
                    if(ir_Wh[4]== false && ir_Wh[5]== false) {
                        if (ir_val1 >= val1) {  // 왼쪽 바퀴 2개
                            mode = 15;
                        }
                        else {
                            mode = 16;  // 왼쪽 바퀴 2개 + 앞 중앙 ir
                        }
                    }
                    else if(ir_Wh[4]== true && ir_Wh[5]== false) {  // 왼쪽 바퀴 2개 + 앞 ir
                        mode = 17;
                    }
                    else if(ir_Wh[4]== false && ir_Wh[5]== true) {
                        if (ir_val1 >= val1) {  // 왼쪽 바퀴 2개 + 뒤 ir
                            mode = 18;
                        }
                        else {
                            mode = 19;  // 왼쪽 바퀴 2개 + 뒤 ir + 앞 중앙 ir
                        }
                    }
                    else {  // 왼쪽 바퀴 2개 + 앞 ir + 뒤 ir
                        mode = 20;
                    }
                }

                else if(ir_Wh[0] == false && ir_Wh[1]== true && ir_Wh[2] == false && ir_Wh[3]== true) {
                    if(ir_Wh[4]== false && ir_Wh[5]== false) {
                        if (ir_val1 >= val1) {  // 오른쪽 바퀴 2개
                            mode = 21;
                        }
                        else {
                            mode = 22;  // 오른쪽 바퀴 2개 + 앞 중앙 ir
                        }
                    }
                    else if(ir_Wh[4]== true && ir_Wh[5]== false) {  // 오른쪽 바퀴 2개 + 앞 ir
                        mode = 23;
                    }
                    else if(ir_Wh[4]== false && ir_Wh[5]== true) {
                        if (ir_val1 >= val1) {  // 오른쪽 바퀴 2개 + 뒤 ir
                            mode = 24;
                        }
                        else {
                            mode = 25;  // 오른쪽 바퀴 2개 + 뒤 ir + 앞 중앙 ir
                        }
                    }
                    else {  // 오른쪽 바퀴 2개 + 앞 ir + 뒤 ir
                        mode = 26;
                    }
                }

                else if(ir_Wh[0] == true && ir_Wh[1]== true && ir_Wh[2] == true && ir_Wh[3]== false) { // 앞+ 뒤왼 3개
                    mode = 27;
                }

                else if(ir_Wh[0] == true && ir_Wh[1]== true && ir_Wh[2] == false && ir_Wh[3]== true) { // 앞 + 뒤오 3개
                    mode = 28;
                }

                else if(ir_Wh[0] == true && ir_Wh[1]== false && ir_Wh[2] == true && ir_Wh[3]== true) { // 뒤 + 앞왼 3개
                    mode = 29;
                }

                else if(ir_Wh[0] == false && ir_Wh[1]== true && ir_Wh[2] == true && ir_Wh[3]== true) { // 뒤 + 앞오 3개
                    mode = 30;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else {
                    mode = 1;
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 3){  // 앞 왼 바퀴

                //pre_mode = 3;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_Wh[6] == true) {
                        mode = 13;
                    }
                    else if (ir_Wh[6] == false) {
                        moter (0, 0, 0.1, 0.35);
                    }
                }

                else if (dist == 255) {
                    moter (1, 1, 0.4, 0.2);
                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        timerST();
                        if (t < 400) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else {
                            mode = 1;
                            timerEND();
                        }
                    }
                }

                else if(60 >= dist && dist > 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        // PL = 0.3;
                        // PR = 0.3-N*0.3;
                        // moter(0, 0, PL, PR);
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (75 <= degree && degree < 105) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }
                    else if (105 <= degree && degree < 140) {
                        // PL = 0.3+N*0.3;
                        // PR = 0.3;
                        // moter(0, 0, PL, PR);
                        moter(0, 1, 0.3, 0.3);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if(dist <= 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (70 <= degree && degree < 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 4) {  // 앞왼 + 앞ir

                //pre_mode = 4;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_Wh[6] == true) {
                        mode = 13;
                    }
                    else if (ir_Wh[6] == false) {
                        moter (0, 0, 0.1, 0.3);
                    }
                }

                else if (dist == 255) {
                    moter (1, 1, 0.3, 0.2);
                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        timerST();
                        if (t < 400) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else {
                            mode = 1;
                            timerEND();
                        }
                    }
                }

                else if(60 >= dist && dist > 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        // PL = 0.3;
                        // PR = 0.3-N*0.3;
                        // moter(0, 0, PL, PR);
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (75 <= degree && degree < 105) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }
                    else if (105 <= degree && degree < 140) {
                        // PL = 0.3+N*0.3;
                        // PR = 0.3;
                        // moter(0, 0, PL, PR);
                        moter(0, 1, 0.3, 0.3);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if(dist <= 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (70 <= degree && degree < 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 5){  // 앞 오 바퀴

                //pre_mode = 5;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_Wh[6] == true) {
                        mode = 13;
                    }
                    else if (ir_Wh[6] == false) {
                        moter (0, 0, 0.35, 0.1);
                    }
                }

                else if (dist == 255) {
                    moter (1, 1, 0.2, 0.4);
                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        timerST();
                        if (t < 400) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else {
                            mode = 1;
                            timerEND();
                        }
                    }
                }

                else if(60 >= dist && dist > 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        // PL = 0.3;
                        // PR = 0.3-N*0.3;
                        // moter(0, 0, PL, PR);
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (75 <= degree && degree < 105) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }
                    else if (105 <= degree && degree < 140) {
                        // PL = 0.3+N*0.3;
                        // PR = 0.3;
                        // moter(0, 0, PL, PR);
                        moter(0, 1, 0.3, 0.3);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if(dist <= 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (70 <= degree && degree < 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 6) {  // 앞오 + 앞ir

                //pre_mode = 6;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_Wh[6] == true) {
                        mode = 13;
                    }
                    else if (ir_Wh[6] == false) {
                        moter (0, 0, 0.3, 0.1);
                    }
                }

                else if (dist == 255) {
                    moter (1, 1, 0.2, 0.3);
                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        timerST();
                        if (t < 400) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else {
                            mode = 1;
                            timerEND();
                        }
                    }
                }

                else if(60 >= dist && dist > 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        // PL = 0.3;
                        // PR = 0.3-N*0.3;
                        // moter(0, 0, PL, PR);
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (75 <= degree && degree < 105) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }
                    else if (105 <= degree && degree < 140) {
                        // PL = 0.3+N*0.3;
                        // PR = 0.3;
                        // moter(0, 0, PL, PR);
                        moter(0, 1, 0.3, 0.3);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if(dist <= 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (70 <= degree && degree < 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 7) {  // 뒤 왼 바퀴

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 8) {  // 뒤왼 + 뒤ir

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 9) {  // 뒤 오 바퀴

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 10) {  // 뒤오 + 뒤ir

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 11) {  // 앞 ir 전체

                //pre_mode = 11;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_Wh[6] == true) {
                        mode = 13;
                    }
                    else if (ir_Wh[6] == false) {
                        moter(0, 0, 0.1, 0.1);
                    }
                }

                else if (dist == 255) {
                    moter (1, 1, 0.3, 0.3);
                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        timerST();
                        if (t < 400) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else {
                            mode = 1;
                            timerEND();
                        }
                    }
                }

                else if(60 >= dist && dist > 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        // PL = 0.3;
                        // PR = 0.3-N*0.55;
                        // moter(0, 0, PL, PR);
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (75 <= degree && degree < 105) {
                        if (ir_val0 < val0 && ir_val1 < val1 && ir_val2 < val2) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }
                    else if (105 <= degree && degree < 140) {
                        // PL = 0.3+N*0.55;
                        // PR = 0.3;
                        // moter(0, 0, PL, PR);
                        moter(0, 1, 0.3, 0.3);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if(dist <= 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (70 <= degree && degree < 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 12) {  // 뒤 ir 전체

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.4, 0.4);
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 13){  // 앞바퀴 2개

                //pre_mode = 13;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_Wh[7] == true) {
                        moter(1, 1, 0.2, 0.2);
                    }
                    else if (ir_Wh[7] == false) {
                        mode = 35;
                    }
                }
                else if (dist == 255) {
                    moter (1, 1, 0.4, 0.4);
                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        timerST();
                        if (t < 400) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else {
                            mode = 1;
                            timerEND();
                        }
                    }
                }
                else if(60 >= dist && dist > 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        // PL = 0.3;
                        // PR = 0.3-N*0.55;
                        // moter(0, 0, PL, PR);
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (75 <= degree && degree < 105) {
                        if (ir_val0 < val0 && ir_val1 < val1 && ir_val2 < val2) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }
                    else if (105 <= degree && degree < 140) {
                        // PL = 0.3+N*0.55;
                        // PR = 0.3;
                        // moter(0, 0, PL, PR);
                        moter(0, 1, 0.3, 0.3);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if(dist <= 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (70 <= degree && degree < 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 14) {  // 뒷바퀴 2개

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.4, 0.4);
                }
            }

    //------------------------------------------------------------------------------------------------------------------
    //----------------------- 왼쪽 바퀴 2개 -------------------------------------------------------------------------15~20
    //------------------------------------------------------------------------------------------------------------------

            else if (mode == 15) {  // 왼쪽 바퀴 2개

                //pre_mode = 15;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist == 255) {
                    if (ir_val5 >= val5 && ir_val6 >= val6) {
                        moter(1, 1, 0.3, 0.3);
                        if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                            timerST();
                            if (t < 400) {
                                moter(1, 1, 0.3, 0.3);
                            }
                            else {
                                mode = 1;
                                timerEND();
                            }
                        }
                    }
                    else {
                        moter(1, 1, 0.4, 0.1);
                    }
                }
                else {
                    mode = 40;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 16) {  // 왼쪽 바퀴 2개 + 앞 중앙 ir

                //pre_mode = 16;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_val1 < val1) {
                        moter(0, 1, 0.3, 0.2);
                    }
                    else if (ir_val1 >= val1) {
                        mode = 40;
                    }
                }
                else if (dist == 255) {
                    if (ir_val5 >= val5 && ir_val6 >= val6) {
                        moter(1, 1, 0.3, 0.3);
                        if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                            timerST();
                            if (t < 400) {
                                moter(1, 1, 0.3, 0.3);
                            }
                            else {
                                mode = 1;
                                timerEND();
                            }
                        }
                    }
                    else {
                        moter(1, 1, 0.35, 0.1);
                    }
                }
                else {
                    mode = 40;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 17) {  // 왼쪽 바퀴 2개 + 앞 ir

                //pre_mode = 17;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_val1 < val1) {
                        moter(0, 0, 0.3, 0.1);
                    }
                    else if (ir_val1 >= val1) {
                        mode = 40;
                    }
                }
                else if (dist == 255) {
                    if (ir_val5 >= val5 && ir_val6 >= val6) {
                        moter(1, 1, 0.3, 0.3);
                        if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                            timerST();
                            if (t < 400) {
                                moter(1, 1, 0.3, 0.3);
                            }
                            else {
                                mode = 1;
                                timerEND();
                            }
                        }
                    }
                    else {
                        moter(1, 1, 0.3, 0.1);
                    }
                }
                else {
                    mode = 40;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 18) {  // 왼쪽 바퀴 2개 + 뒤 ir

                //pre_mode = 18;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 19) {  // 왼쪽 바퀴 2개 + 뒤 ir + 앞 중앙 ir

                //pre_mode = 19;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 20) {  // 왼쪽 바퀴 2개 + 앞 ir + 뒤 ir

                //pre_mode = 20;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.35);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------
    //---------------------- 오른쪽 바퀴 2개 -------------------------------------------------------------------------21~26
    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 21) {  // 오른쪽 바퀴 2개

                //pre_mode = 21;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist == 255) {
                    if (ir_val5 >= val5 && ir_val6 >= val6) {
                        moter(1, 1, 0.3, 0.3);
                        if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                            timerST();
                            if (t < 400) {
                                moter(1, 1, 0.3, 0.3);
                            }
                            else {
                                mode = 1;
                                timerEND();
                            }
                        }
                    }
                    else {
                        moter(1, 1, 0.1, 0.4);
                    }
                }
                else {
                    mode = 40;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 22) {  // 오른쪽 바퀴 2개 + 앞 중앙 ir

                //pre_mode = 22;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_val1 < val1) {
                        moter(1, 0, 0.2, 0.3);
                    }
                    else if (ir_val1 >= val1) {
                        mode = 40;
                    }
                }
                else if (dist == 255) {
                    if (ir_val5 >= val5 && ir_val6 >= val6) {
                        moter(1, 1, 0.3, 0.3);
                        if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                            timerST();
                            if (t < 400) {
                                moter(1, 1, 0.3, 0.3);
                            }
                            else {
                                mode = 1;
                                timerEND();
                            }
                        }
                    }
                    else {
                        moter(1, 1, 0.1, 0.35);
                    }
                }
                else {
                    mode = 40;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 23) {  // 오른쪽 바퀴 2개 + 앞 ir

                //pre_mode = 23;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_val1 < val1) {
                        moter(0, 0, 0.1, 0.3);
                    }
                    else if (ir_val1 >= val1) {
                        mode = 40;
                    }
                }
                else if (dist == 255) {
                    if (ir_val5 >= val5 && ir_val6 >= val6) {
                        moter(1, 1, 0.3, 0.3);
                        if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                            timerST();
                            if (t < 400) {
                                moter(1, 1, 0.3, 0.3);
                            }
                            else {
                                mode = 1;
                                timerEND();
                            }
                        }
                    }
                    else {
                        moter(1, 1, 0.1, 0.3);
                    }
                }
                else {
                    mode = 40;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 24) {  // 오른쪽 바퀴 2개 + 뒤 ir

                //pre_mode = 24;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 25) {  // 오른쪽 바퀴 2개 + 뒤 ir + 앞 중앙 ir

            //pre_mode = 25;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 26) {  // 오른쪽 바퀴 2개 + 앞 ir + 뒤 ir

                //pre_mode = 26;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.35, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------------------------------------


            else if (mode == 27) {  // 앞+ 뒤왼 3개

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(1, 1, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(1, 1, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 28) {  // 앞 + 뒤오 3개

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(1, 1, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(1, 1, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 29) {  // 뒤 + 앞왼 3개

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(1, 1, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 30) {  // 뒤 + 앞오 3개

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(1, 1, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------
    //-----------원 다 들어감ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ----------------

            else if (mode == 31) {

                if (imu_bool == true) {
                    mode = 100;
                }

                else if (dist == 255) {
                    if (degree < 90) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (degree >= 90) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if (dist != 255) {
                    if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    // else if (45 <= degree && degree < 80) {
                    //     moter(1, 0, 0.3, 0.3);
                    // }
                    else if (70 <= degree && degree < 110) {
                        mode = 42;
                    }
                    // else if (100 <= degree && degree < 135) {
                    //     moter(0, 1, 0.3, 0.3);
                    // }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

            else if (mode == 42) {

                if (imu_bool == true) {
                    mode = 100;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                }

                else if (dist == 255) {
                    if (degree < 90) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (degree >= 90) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                // else if (dist < 15) {
                //     moter(1, 1, 0.2, 0.2);
                // }

                else if (dist < 70) {
                    if (psd_val4 < val4) {
                        if (psd_val1 < val1 && psd_val3 >= val3) {
                            mode = 43;
                        }
                        else if (psd_val1 >= val1 && psd_val3 < val3) {
                            mode = 44;
                        }
                        else if (psd_val1 < val1 && psd_val3 < val3) {
                            if (psd_val1 < psd_val3) {
                                mode = 43;
                            }
                            else if (psd_val1 >= psd_val3) {
                                mode = 44;
                            }
                        }
                        else {
                            if (psd_val1 < psd_val3) {
                                mode = 43;
                            }
                            else if (psd_val1 >= psd_val3) {
                                mode = 44;
                            }
                        }
                    }
                    else if (psd_val4 >= val4) {
                        mode = 45;
                    }
                }

                else if (dist >= 70 && dist != 255) {
                    if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        PL = 0.3;
                        PR = 0.3-N*0.55;
                        moter(0, 0, PL, PR);
                    }
                    else if (75 <= degree && degree < 105) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else if (105 <= degree && degree < 140) {
                        PL = 0.3+N*0.55;
                        PR = 0.3;
                        moter(0, 0, PL, PR);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

            else if (mode == 43) {

                if (imu_bool == true) {
                    mode = 100;
                }

                else if (degree < 20) {
                    moter(0, 0, 0.3, 0.3);
                }
                else {
                    moter(0, 1, 0.3, 0.3);
                }
            }

            else if (mode == 44) {

                if (imu_bool == true) {
                    mode = 100;
                }

                else if (degree > 160) {
                    moter(0, 0, 0.3, 0.3);
                }
                else {
                    moter(1, 0, 0.3, 0.3);
                }
            }

            else if (mode == 45) {

                if (imu_bool == true) {
                    mode = 100;
                }

                moter(1, 1, 0.4, 0.4);
            }

    //-------------------------------------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 34) {   // ir 못 볼때 타이머 직진 
                timerST();
                if (t < 700) {
                    moter(0, 0, 0.3, 0.3);
                }
                else {
                    //moter(0, 0, 0, 0);
                    timerEND();
                    mode = 35;
                }
            }
            
            else if (mode == 35) {  // 원 돌기전 회전
                if (degree < 90) {
                    mode = 36;
                }
                else {
                    mode = 37;
                }
            }

            else if (mode == 36) {
                moter(1, 0, 0.3, 0.3);
                if (ir_val1 > val1) {
                    mode = 40;
                    //moter(0,0,0,0);
                }
            }

            else if (mode == 37) {
                moter(0, 1, 0.3, 0.3);
                if (ir_val1 > val1) {
                    mode = 40;
                    //moter(0,0,0,0);
                }
            }


    //-------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------

            else if (mode == 40) {   //빨간원 돌기 코드

                if (imu_bool == true) {
                    mode = 100;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    mode = 1;
                }

                else if (ir_Wh[0] == true && ir_Wh[1] == true && ir_Wh[2] == true && ir_Wh[3] == true) {
                    mode = 31;
                }

                else if (dist == 255) {  //상대 안보임
                    if (degree < 90) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (degree >= 90) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
                

                else if (0<=dist && dist<30) {  //상대 매우 큼
                    if (degree <= 70) {
                        moter(1, 0, 0.4, 0.4);
                    }

                    else if (70 < degree && degree <= 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }

                    else if (110 < degree) {
                        moter(0, 1, 0.4, 0.4);
                    }
                }
                

                else if (30<=dist && dist<50) {  //상대 큼

                    if (degree <= 40) {
                        moter(1, 0, 0.3, 0.3);
                    }

                    else if (40 < degree && degree <= 75) {
                        if ((ir_val3 < val3 || ir_val4 < val4) && (ir_val7 >= val7 || ir_val8 >= val8)) {
                            state = 1;
                        }
                        if ((ir_val3 >= val3 || ir_val4 >= val4) && (ir_val7 < val7 || ir_val8 < val8)) {
                            state = 2;
                        }
                        else {
                            state = 0;
                            moter(1, 0, 0.3, 0.3);
                        }
                        if (state == 1) {
                            PL = 0.3;
                            PR = 0.3-N*0.55;
                            moter(0, 0, PL, PR);
                        }
                        if (state == 2) {
                            moter(1, 0, 0.3, 0.3);
                        }
                    }

                    else if (75 < degree && degree <= 105) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }

                    else if (105 < degree && degree <= 140) {
                        if ((ir_val3 <= val3 || ir_val4 <= val4) && (ir_val7 > val7 || ir_val8 > val8)) {
                            state = 1;
                        }
                        else if ((ir_val3 > val3 || ir_val4 > val4) && (ir_val7 <= val7 || ir_val8 <= val8)) {
                            state = 2;
                        }
                        else {
                            state = 0;
                            moter(0, 1, 0.3, 0.3);
                        }
                        if (state == 1) {
                            PL = 0.3+N*0.55;
                            PR = 0.3;
                            moter(0, 0, PL, PR);
                        }
                        if (state == 2) {
                            moter(0, 1, 0.3, 0.3);
                        }
                    }

                    else if (140 < degree) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if (50<=dist && dist != 255) {  //상대 작음, 보통

                    if (ir_val0 > val0) {
                        if (ir_val2 > val2) {
                            moter(0, 0, 0.71, 0.2);
                        }
                        else if (ir_val1 <= val1) {
                            moter(0, 0, 0.65, 0.2);
                        }
                        else {
                            moter(0, 0, 0.68, 0.2);
                        }
                    }

                    else if (ir_val2 > val2) {
                        if (ir_val0 > val0) {
                            moter(0, 0, 0.2, 0.71);
                        }
                        else if (ir_val1 <= val1) {
                            moter(0, 0, 0.2, 0.65);
                        }
                        else {
                            moter(0, 0, 0.2, 0.68);
                        }
                    }
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 100) {   // 로봇 들림

                //pc.printf("looool");

                if (imu_bool == false) {
                    mode = 1;
                }

                else if (FRoll < Rmin) {  //  뒤 들림
                    if (ir_Wh[4] == true) {
                        if (ir_Wh[6] == true) {
                            moter(0, 0, 0.3, 0.3);
                        }
                        else if (ir_Wh[6] == false) {
                            moter(1, 1, 0.5, 0.5);
                        }
                    }
                    else if (ir_Wh[4] == false) {
                        if (ir_Wh[6] == true) {
                            moter(0, 0, 0.5, 0.5);
                        }
                        else if (ir_Wh[6] == false) {
                            moter(1, 1, 0.5, 0.5);
                        }
                    }
                }

                else if (FRoll > Rmax) {  // 앞 들림
                    if (ir_Wh[5] == true) {
                        if (ir_Wh[7] == true) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else if (ir_Wh[7] == false) {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (ir_Wh[5] == false) {
                        if (ir_Wh[7] == true) {
                            moter(1, 1, 0.5, 0.5);
                        }
                        else if (ir_Wh[7] == false) {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                }

                else {
                    if (FPitch < Pmin) {  // 오른쪽 들림
                        if (psd_val2 < 50 && psd_val4 >= 50) {
                            mode = 101;
                        }
                        else if (psd_val2 >= 50 && psd_val4 < 50) {
                            mode = 102;
                        }
                        else if (psd_val2 >= 50 && psd_val4 >= 50) {
                            mode = 103;
                        }
                        else {
                            mode = 104;
                        }
                    }

                    else if (FPitch > Pmax) {  // 왼쪽 들림
                        if (psd_val2 < 50 && psd_val4 >= 50) {
                            mode = 105;
                        }
                        else if (psd_val2 >= 50 && psd_val4 < 50) {
                            mode = 106;
                        }
                        else if (psd_val2 >= 50 && psd_val4 >= 50) {
                            mode = 107;
                        }
                        else {
                            mode = 108;
                        }
                    }
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 101) {
                if (imu_bool == false) {
                    mode = 1;
                }
                else if (ir_val6 < val6 || ir_val7 < val7) {
                    mode = 102;
                }
                else {
                    moter(1, 1, 0.5, 0.5);
                }
            }

            else if (mode == 102) {
                if (imu_bool == false) {
                    mode = 1;
                }
                else if (ir_val0 < val0 || ir_val1 < val1 || ir_val8 < val8) {
                    mode = 101;
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

            else if (mode == 103) {
                if (psd_val2 < psd_val4) {
                    mode = 101;
                }
                else if (psd_val2 >= psd_val4) {
                    mode = 102;
                }
            }

            else if (mode == 104) {
                if (psd_val2 < psd_val4) {
                    mode = 101;
                }
                else if (psd_val2 >= psd_val4) {
                    mode = 102;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 105) {
                if (imu_bool == false) {
                    mode = 1;
                }
                else if (ir_val5 < val5 || ir_val4 < val4) {
                    mode = 105;
                }
                else {
                    moter(1, 1, 0.5, 0.5);
                }
            }

            else if (mode == 106) {
                if (imu_bool == false) {
                    mode = 1;
                }
                else if (ir_val1 < val1 || ir_val2 < val2 || ir_val3 < val3) {
                    mode = 105;
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

            else if (mode == 107) {
                if (psd_val2 < psd_val4) {
                    mode = 105;
                }
                else if (psd_val2 >= psd_val4) {
                    mode = 106;
                }
            }

            else if (mode == 108) {
                if (psd_val2 < psd_val4) {
                    mode = 105;
                }
                else if (psd_val2 >= psd_val4) {
                    mode = 106;
                }
            }

            All_move = false;
        }

    //-----------------------------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------------------------
    //--------------파이 맛탱이 감--------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------------------------

        else if(pi_stop == true) {

            //pc.printf("pi stop\n");

            if (mode == 0) {    //처음 시작 회전
                moter(0, 1, 0.4, 0.4);
                if (ir_val1 < 90 || ir_val2 < 50 || ir_val3 < 90) {
                    mode = 1;
                }
            }

    //---------------------------------------------------------------------------------------------------------------------
            
            else if (mode == 1) {   //검은색 영역 상대 공격, 주행

                //pre_mode = 1;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if (ir_Wh[0] == true || ir_Wh[1] == true || ir_Wh[2] == true || ir_Wh[3] == true || ir_Wh[4] == true || ir_Wh[5] == true) {
                    mode = 2;
                }

                else if (ir_val1 < 90 && ir_val3 < 90) {
                    if (ir_val2 < 50) {

                    }
                    else if (ir_val2 >= 50) {
                        
                    }
                }
            }

    //-----------------------------------------------------------------------------------------------------------------------------
            
            else if (mode == 2) {   //색영역 코드

                //pre_mode = 2;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0] == true && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    if(ir_Wh[4] == false) {  // 앞 왼 바퀴
                        mode = 3;
                    }
                    else if(ir_Wh[4] == true) {  // 앞왼 + 앞ir
                        mode = 4;
                    }
                }

                else if(ir_Wh[0] == false && ir_Wh[1] == true && ir_Wh[2] == false && ir_Wh[3] == false) {
                    if(ir_Wh[4] == false) {  // 앞 오 바퀴
                        mode = 5;
                    }
                    else if(ir_Wh[4] == true) {  // 앞오 + 앞ir
                        mode = 6;
                    }
                }

                else if(ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == true && ir_Wh[3] == false) {
                    if(ir_Wh[5] == false) {  // 뒤 왼 바퀴
                        mode = 7;
                    }
                    else if(ir_Wh[5] == true) {  // 뒤왼 + 뒤ir
                        mode = 8;
                    }
                }

                else if(ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == true) {
                    if(ir_Wh[5] == false) {  // 뒤 오 바퀴
                        mode = 9;
                    }
                    else if(ir_Wh[5] == true) {  // 뒤오 + 뒤ir
                        mode = 10;
                    }
                }

                else if(ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false &&ir_Wh[3] == false) {
                    if(ir_Wh[4] == true && ir_Wh[5] == false) {  // 앞 ir 전체
                        mode = 11;
                    }
                    else if(ir_Wh[4] == false && ir_Wh[5] == true) {  // 뒤 ir 전체
                        mode = 12;
                    }
                    else if(ir_Wh[4] == false && ir_Wh[5] == false) {  // 검은색 영역
                        mode = 1;
                    }
                    else {
                        mode = 31;
                    }
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2] == false && ir_Wh[3] == false) { // 앞바퀴 2개
                    mode = 13;
                }

                else if(ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2]== true && ir_Wh[3]== true) { // 뒷바퀴 2개
                    mode = 14;
                }

                else if(ir_Wh[0]== true && ir_Wh[1] == false && ir_Wh[2]== true && ir_Wh[3] == false) {
                    if(ir_Wh[4]== false && ir_Wh[5]== false) {
                        if (ir_val1 >= val1) {  // 왼쪽 바퀴 2개
                            mode = 15;
                        }
                        else {
                            mode = 16;  // 왼쪽 바퀴 2개 + 앞 중앙 ir
                        }
                    }
                    else if(ir_Wh[4]== true && ir_Wh[5]== false) {  // 왼쪽 바퀴 2개 + 앞 ir
                        mode = 17;
                    }
                    else if(ir_Wh[4]== false && ir_Wh[5]== true) {
                        if (ir_val1 >= val1) {  // 왼쪽 바퀴 2개 + 뒤 ir
                            mode = 18;
                        }
                        else {
                            mode = 19;  // 왼쪽 바퀴 2개 + 뒤 ir + 앞 중앙 ir
                        }
                    }
                    else {  // 왼쪽 바퀴 2개 + 앞 ir + 뒤 ir
                        mode = 20;
                    }
                }

                else if(ir_Wh[0] == false && ir_Wh[1]== true && ir_Wh[2] == false && ir_Wh[3]== true) {
                    if(ir_Wh[4]== false && ir_Wh[5]== false) {
                        if (ir_val1 >= val1) {  // 오른쪽 바퀴 2개
                            mode = 21;
                        }
                        else {
                            mode = 22;  // 오른쪽 바퀴 2개 + 앞 중앙 ir
                        }
                    }
                    else if(ir_Wh[4]== true && ir_Wh[5]== false) {  // 오른쪽 바퀴 2개 + 앞 ir
                        mode = 23;
                    }
                    else if(ir_Wh[4]== false && ir_Wh[5]== true) {
                        if (ir_val1 >= val1) {  // 오른쪽 바퀴 2개 + 뒤 ir
                            mode = 24;
                        }
                        else {
                            mode = 25;  // 오른쪽 바퀴 2개 + 뒤 ir + 앞 중앙 ir
                        }
                    }
                    else {  // 오른쪽 바퀴 2개 + 앞 ir + 뒤 ir
                        mode = 26;
                    }
                }

                else if(ir_Wh[0] == true && ir_Wh[1]== true && ir_Wh[2] == true && ir_Wh[3]== false) { // 앞+ 뒤왼 3개
                    mode = 27;
                }

                else if(ir_Wh[0] == true && ir_Wh[1]== true && ir_Wh[2] == false && ir_Wh[3]== true) { // 앞 + 뒤오 3개
                    mode = 28;
                }

                else if(ir_Wh[0] == true && ir_Wh[1]== false && ir_Wh[2] == true && ir_Wh[3]== true) { // 뒤 + 앞왼 3개
                    mode = 29;
                }

                else if(ir_Wh[0] == false && ir_Wh[1]== true && ir_Wh[2] == true && ir_Wh[3]== true) { // 뒤 + 앞오 3개
                    mode = 30;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else {
                    mode = 1;
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 3){  // 앞 왼 바퀴

                //pre_mode = 3;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_Wh[6] == true) {
                        mode = 13;
                    }
                    else if (ir_Wh[6] == false) {
                        moter (0, 0, 0.1, 0.35);
                    }
                }

                else if (dist == 255) {
                    moter (1, 1, 0.4, 0.2);
                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        timerST();
                        if (t < 400) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else {
                            mode = 1;
                            timerEND();
                        }
                    }
                }

                else if(60 >= dist && dist > 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        // PL = 0.3;
                        // PR = 0.3-N*0.3;
                        // moter(0, 0, PL, PR);
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (75 <= degree && degree < 105) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }
                    else if (105 <= degree && degree < 140) {
                        // PL = 0.3+N*0.3;
                        // PR = 0.3;
                        // moter(0, 0, PL, PR);
                        moter(0, 1, 0.3, 0.3);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if(dist <= 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (70 <= degree && degree < 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 4) {  // 앞왼 + 앞ir

                //pre_mode = 4;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_Wh[6] == true) {
                        mode = 13;
                    }
                    else if (ir_Wh[6] == false) {
                        moter (0, 0, 0.1, 0.3);
                    }
                }

                else if (dist == 255) {
                    moter (1, 1, 0.3, 0.2);
                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        timerST();
                        if (t < 400) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else {
                            mode = 1;
                            timerEND();
                        }
                    }
                }

                else if(60 >= dist && dist > 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        // PL = 0.3;
                        // PR = 0.3-N*0.3;
                        // moter(0, 0, PL, PR);
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (75 <= degree && degree < 105) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }
                    else if (105 <= degree && degree < 140) {
                        // PL = 0.3+N*0.3;
                        // PR = 0.3;
                        // moter(0, 0, PL, PR);
                        moter(0, 1, 0.3, 0.3);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if(dist <= 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (70 <= degree && degree < 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 5){  // 앞 오 바퀴

                //pre_mode = 5;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_Wh[6] == true) {
                        mode = 13;
                    }
                    else if (ir_Wh[6] == false) {
                        moter (0, 0, 0.35, 0.1);
                    }
                }

                else if (dist == 255) {
                    moter (1, 1, 0.2, 0.4);
                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        timerST();
                        if (t < 400) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else {
                            mode = 1;
                            timerEND();
                        }
                    }
                }

                else if(60 >= dist && dist > 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        // PL = 0.3;
                        // PR = 0.3-N*0.3;
                        // moter(0, 0, PL, PR);
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (75 <= degree && degree < 105) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }
                    else if (105 <= degree && degree < 140) {
                        // PL = 0.3+N*0.3;
                        // PR = 0.3;
                        // moter(0, 0, PL, PR);
                        moter(0, 1, 0.3, 0.3);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if(dist <= 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (70 <= degree && degree < 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 6) {  // 앞오 + 앞ir

                //pre_mode = 6;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_Wh[6] == true) {
                        mode = 13;
                    }
                    else if (ir_Wh[6] == false) {
                        moter (0, 0, 0.3, 0.1);
                    }
                }

                else if (dist == 255) {
                    moter (1, 1, 0.2, 0.3);
                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        timerST();
                        if (t < 400) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else {
                            mode = 1;
                            timerEND();
                        }
                    }
                }

                else if(60 >= dist && dist > 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        // PL = 0.3;
                        // PR = 0.3-N*0.3;
                        // moter(0, 0, PL, PR);
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (75 <= degree && degree < 105) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }
                    else if (105 <= degree && degree < 140) {
                        // PL = 0.3+N*0.3;
                        // PR = 0.3;
                        // moter(0, 0, PL, PR);
                        moter(0, 1, 0.3, 0.3);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if(dist <= 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (70 <= degree && degree < 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 7) {  // 뒤 왼 바퀴

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 8) {  // 뒤왼 + 뒤ir

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 9) {  // 뒤 오 바퀴

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 10) {  // 뒤오 + 뒤ir

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 11) {  // 앞 ir 전체

                //pre_mode = 11;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_Wh[6] == true) {
                        mode = 13;
                    }
                    else if (ir_Wh[6] == false) {
                        moter(0, 0, 0.1, 0.1);
                    }
                }

                else if (dist == 255) {
                    moter (1, 1, 0.3, 0.3);
                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        timerST();
                        if (t < 400) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else {
                            mode = 1;
                            timerEND();
                        }
                    }
                }

                else if(60 >= dist && dist > 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        // PL = 0.3;
                        // PR = 0.3-N*0.55;
                        // moter(0, 0, PL, PR);
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (75 <= degree && degree < 105) {
                        if (ir_val0 < val0 && ir_val1 < val1 && ir_val2 < val2) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }
                    else if (105 <= degree && degree < 140) {
                        // PL = 0.3+N*0.55;
                        // PR = 0.3;
                        // moter(0, 0, PL, PR);
                        moter(0, 1, 0.3, 0.3);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if(dist <= 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (70 <= degree && degree < 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 12) {  // 뒤 ir 전체

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.4, 0.4);
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 13){  // 앞바퀴 2개

                //pre_mode = 13;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_Wh[7] == true) {
                        moter(1, 1, 0.2, 0.2);
                    }
                    else if (ir_Wh[7] == false) {
                        mode = 35;
                    }
                }
                else if (dist == 255) {
                    moter (1, 1, 0.4, 0.4);
                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        timerST();
                        if (t < 400) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else {
                            mode = 1;
                            timerEND();
                        }
                    }
                }
                else if(60 >= dist && dist > 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        // PL = 0.3;
                        // PR = 0.3-N*0.55;
                        // moter(0, 0, PL, PR);
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (75 <= degree && degree < 105) {
                        if (ir_val0 < val0 && ir_val1 < val1 && ir_val2 < val2) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }
                    else if (105 <= degree && degree < 140) {
                        // PL = 0.3+N*0.55;
                        // PR = 0.3;
                        // moter(0, 0, PL, PR);
                        moter(0, 1, 0.3, 0.3);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if(dist <= 30) {

                    if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                    }

                    else if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (70 <= degree && degree < 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

    //----------------------------------------------------------------------------------------------------------------------

            else if (mode == 14) {  // 뒷바퀴 2개

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.4, 0.4);
                }
            }

    //------------------------------------------------------------------------------------------------------------------
    //----------------------- 왼쪽 바퀴 2개 -------------------------------------------------------------------------15~20
    //------------------------------------------------------------------------------------------------------------------

            else if (mode == 15) {  // 왼쪽 바퀴 2개

                //pre_mode = 15;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist == 255) {
                    if (ir_val5 >= val5 && ir_val6 >= val6) {
                        moter(1, 1, 0.3, 0.3);
                        if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                            timerST();
                            if (t < 400) {
                                moter(1, 1, 0.3, 0.3);
                            }
                            else {
                                mode = 1;
                                timerEND();
                            }
                        }
                    }
                    else {
                        moter(1, 1, 0.4, 0.1);
                    }
                }
                else {
                    mode = 40;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 16) {  // 왼쪽 바퀴 2개 + 앞 중앙 ir

                //pre_mode = 16;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_val1 < val1) {
                        moter(0, 1, 0.3, 0.2);
                    }
                    else if (ir_val1 >= val1) {
                        mode = 40;
                    }
                }
                else if (dist == 255) {
                    if (ir_val5 >= val5 && ir_val6 >= val6) {
                        moter(1, 1, 0.3, 0.3);
                        if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                            timerST();
                            if (t < 400) {
                                moter(1, 1, 0.3, 0.3);
                            }
                            else {
                                mode = 1;
                                timerEND();
                            }
                        }
                    }
                    else {
                        moter(1, 1, 0.35, 0.1);
                    }
                }
                else {
                    mode = 40;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 17) {  // 왼쪽 바퀴 2개 + 앞 ir

                //pre_mode = 17;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_val1 < val1) {
                        moter(0, 0, 0.3, 0.1);
                    }
                    else if (ir_val1 >= val1) {
                        mode = 40;
                    }
                }
                else if (dist == 255) {
                    if (ir_val5 >= val5 && ir_val6 >= val6) {
                        moter(1, 1, 0.3, 0.3);
                        if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                            timerST();
                            if (t < 400) {
                                moter(1, 1, 0.3, 0.3);
                            }
                            else {
                                mode = 1;
                                timerEND();
                            }
                        }
                    }
                    else {
                        moter(1, 1, 0.3, 0.1);
                    }
                }
                else {
                    mode = 40;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 18) {  // 왼쪽 바퀴 2개 + 뒤 ir

                //pre_mode = 18;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 19) {  // 왼쪽 바퀴 2개 + 뒤 ir + 앞 중앙 ir

                //pre_mode = 19;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 20) {  // 왼쪽 바퀴 2개 + 앞 ir + 뒤 ir

                //pre_mode = 20;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.35);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------
    //---------------------- 오른쪽 바퀴 2개 -------------------------------------------------------------------------21~26
    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 21) {  // 오른쪽 바퀴 2개

                //pre_mode = 21;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist == 255) {
                    if (ir_val5 >= val5 && ir_val6 >= val6) {
                        moter(1, 1, 0.3, 0.3);
                        if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                            timerST();
                            if (t < 400) {
                                moter(1, 1, 0.3, 0.3);
                            }
                            else {
                                mode = 1;
                                timerEND();
                            }
                        }
                    }
                    else {
                        moter(1, 1, 0.1, 0.4);
                    }
                }
                else {
                    mode = 40;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 22) {  // 오른쪽 바퀴 2개 + 앞 중앙 ir

                //pre_mode = 22;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_val1 < val1) {
                        moter(1, 0, 0.2, 0.3);
                    }
                    else if (ir_val1 >= val1) {
                        mode = 40;
                    }
                }
                else if (dist == 255) {
                    if (ir_val5 >= val5 && ir_val6 >= val6) {
                        moter(1, 1, 0.3, 0.3);
                        if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                            timerST();
                            if (t < 400) {
                                moter(1, 1, 0.3, 0.3);
                            }
                            else {
                                mode = 1;
                                timerEND();
                            }
                        }
                    }
                    else {
                        moter(1, 1, 0.1, 0.35);
                    }
                }
                else {
                    mode = 40;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 23) {  // 오른쪽 바퀴 2개 + 앞 ir

                //pre_mode = 23;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (dist > 60 && dist != 255) {
                    if (ir_val1 < val1) {
                        moter(0, 0, 0.1, 0.3);
                    }
                    else if (ir_val1 >= val1) {
                        mode = 40;
                    }
                }
                else if (dist == 255) {
                    if (ir_val5 >= val5 && ir_val6 >= val6) {
                        moter(1, 1, 0.3, 0.3);
                        if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                            timerST();
                            if (t < 400) {
                                moter(1, 1, 0.3, 0.3);
                            }
                            else {
                                mode = 1;
                                timerEND();
                            }
                        }
                    }
                    else {
                        moter(1, 1, 0.1, 0.3);
                    }
                }
                else {
                    mode = 40;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 24) {  // 오른쪽 바퀴 2개 + 뒤 ir

                //pre_mode = 24;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 25) {  // 오른쪽 바퀴 2개 + 뒤 ir + 앞 중앙 ir

            //pre_mode = 25;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 26) {  // 오른쪽 바퀴 2개 + 앞 ir + 뒤 ir

                //pre_mode = 26;

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.35, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------------------------------------


            else if (mode == 27) {  // 앞+ 뒤왼 3개

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(1, 1, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(1, 1, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 28) {  // 앞 + 뒤오 3개

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(1, 1, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(1, 1, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 29) {  // 뒤 + 앞왼 3개

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(1, 1, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 30) {  // 뒤 + 앞오 3개

                if (imu_bool == true) {
                    mode = 100;
                }

                else if(ir_Wh[0]== true && ir_Wh[1]== true && ir_Wh[2]== true && ir_Wh[3]== true) { // 전체 다
                    mode = 31;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    timerST();
                    if (t < 400) {
                        moter(1, 1, 0.3, 0.3);
                    }
                    else {
                        mode = 1;
                        timerEND();
                    }
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

    //-------------------------------------------------------------------------------------------------------------------
    //-----------원 다 들어감ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ----------------

            else if (mode == 31) {

                if (imu_bool == true) {
                    mode = 100;
                }

                else if (dist == 255) {
                    if (degree < 90) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (degree >= 90) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if (dist != 255) {
                    if (degree < 70) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    // else if (45 <= degree && degree < 80) {
                    //     moter(1, 0, 0.3, 0.3);
                    // }
                    else if (70 <= degree && degree < 110) {
                        mode = 42;
                    }
                    // else if (100 <= degree && degree < 135) {
                    //     moter(0, 1, 0.3, 0.3);
                    // }
                    else if (degree >= 110) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

            else if (mode == 42) {

                if (imu_bool == true) {
                    mode = 100;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                        mode = 1;
                }

                else if (dist == 255) {
                    if (degree < 90) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (degree >= 90) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                // else if (dist < 15) {
                //     moter(1, 1, 0.2, 0.2);
                // }

                else if (dist < 70) {
                    if (psd_val4 < val4) {
                        if (psd_val1 < val1 && psd_val3 >= val3) {
                            mode = 43;
                        }
                        else if (psd_val1 >= val1 && psd_val3 < val3) {
                            mode = 44;
                        }
                        else if (psd_val1 < val1 && psd_val3 < val3) {
                            if (psd_val1 < psd_val3) {
                                mode = 43;
                            }
                            else if (psd_val1 >= psd_val3) {
                                mode = 44;
                            }
                        }
                        else {
                            if (psd_val1 < psd_val3) {
                                mode = 43;
                            }
                            else if (psd_val1 >= psd_val3) {
                                mode = 44;
                            }
                        }
                    }
                    else if (psd_val4 >= val4) {
                        mode = 45;
                    }
                }

                else if (dist >= 70 && dist != 255) {
                    if (degree < 40) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (40 <= degree && degree < 75) {
                        PL = 0.3;
                        PR = 0.3-N*0.55;
                        moter(0, 0, PL, PR);
                    }
                    else if (75 <= degree && degree < 105) {
                        moter(0, 0, 0.3, 0.3);
                    }
                    else if (105 <= degree && degree < 140) {
                        PL = 0.3+N*0.55;
                        PR = 0.3;
                        moter(0, 0, PL, PR);
                    }
                    else if (degree >= 140) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
            }

            else if (mode == 43) {

                if (imu_bool == true) {
                    mode = 100;
                }

                else if (degree < 20) {
                    moter(0, 0, 0.3, 0.3);
                }
                else {
                    moter(0, 1, 0.3, 0.3);
                }
            }

            else if (mode == 44) {

                if (imu_bool == true) {
                    mode = 100;
                }

                else if (degree > 160) {
                    moter(0, 0, 0.3, 0.3);
                }
                else {
                    moter(1, 0, 0.3, 0.3);
                }
            }

            else if (mode == 45) {

                if (imu_bool == true) {
                    mode = 100;
                }

                moter(1, 1, 0.4, 0.4);
            }

    //-------------------------------------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------------------------------------
    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 34) {   // ir 못 볼때 타이머 직진 
                timerST();
                if (t < 700) {
                    moter(0, 0, 0.3, 0.3);
                }
                else {
                    //moter(0, 0, 0, 0);
                    timerEND();
                    mode = 35;
                }
            }
            
            else if (mode == 35) {  // 원 돌기전 회전
                if (degree < 90) {
                    mode = 36;
                }
                else {
                    mode = 37;
                }
            }

            else if (mode == 36) {
                moter(1, 0, 0.3, 0.3);
                if (ir_val1 > val1) {
                    mode = 40;
                    //moter(0,0,0,0);
                }
            }

            else if (mode == 37) {
                moter(0, 1, 0.3, 0.3);
                if (ir_val1 > val1) {
                    mode = 40;
                    //moter(0,0,0,0);
                }
            }


    //-------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------

            else if (mode == 40) {   //빨간원 돌기 코드

                if (imu_bool == true) {
                    mode = 100;
                }

                else if (ir_Wh[0] == false && ir_Wh[1] == false && ir_Wh[2] == false && ir_Wh[3] == false) {
                    mode = 1;
                }

                else if (ir_Wh[0] == true && ir_Wh[1] == true && ir_Wh[2] == true && ir_Wh[3] == true) {
                    mode = 31;
                }

                else if (dist == 255) {  //상대 안보임
                    if (degree < 90) {
                        moter(1, 0, 0.3, 0.3);
                    }
                    else if (degree >= 90) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }
                

                else if (0<=dist && dist<30) {  //상대 매우 큼
                    if (degree <= 70) {
                        moter(1, 0, 0.4, 0.4);
                    }

                    else if (70 < degree && degree <= 110) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }

                    else if (110 < degree) {
                        moter(0, 1, 0.4, 0.4);
                    }
                }
                

                else if (30<=dist && dist<50) {  //상대 큼

                    if (degree <= 40) {
                        moter(1, 0, 0.3, 0.3);
                    }

                    else if (40 < degree && degree <= 75) {
                        if ((ir_val3 < val3 || ir_val4 < val4) && (ir_val7 >= val7 || ir_val8 >= val8)) {
                            state = 1;
                        }
                        if ((ir_val3 >= val3 || ir_val4 >= val4) && (ir_val7 < val7 || ir_val8 < val8)) {
                            state = 2;
                        }
                        else {
                            state = 0;
                            moter(1, 0, 0.3, 0.3);
                        }
                        if (state == 1) {
                            PL = 0.3;
                            PR = 0.3-N*0.55;
                            moter(0, 0, PL, PR);
                        }
                        if (state == 2) {
                            moter(1, 0, 0.3, 0.3);
                        }
                    }

                    else if (75 < degree && degree <= 105) {
                        if (ir_Wh[4] == true) {
                            if (ir_Wh[6] == true) {
                                if (ir_Wh[7] == true) {
                                    moter(1, 1, 0.1, 0.1);
                                }
                                else if (ir_Wh[7] == false) {
                                    moter(0, 0, 0, 0);
                                }
                            }
                            else if (ir_Wh[6] == false) {
                                moter(0, 0, 0.1, 0.1);
                            }
                        }
                        else {
                            moter(0, 0, 0.4, 0.4);
                        }
                    }

                    else if (105 < degree && degree <= 140) {
                        if ((ir_val3 <= val3 || ir_val4 <= val4) && (ir_val7 > val7 || ir_val8 > val8)) {
                            state = 1;
                        }
                        else if ((ir_val3 > val3 || ir_val4 > val4) && (ir_val7 <= val7 || ir_val8 <= val8)) {
                            state = 2;
                        }
                        else {
                            state = 0;
                            moter(0, 1, 0.3, 0.3);
                        }
                        if (state == 1) {
                            PL = 0.3+N*0.55;
                            PR = 0.3;
                            moter(0, 0, PL, PR);
                        }
                        if (state == 2) {
                            moter(0, 1, 0.3, 0.3);
                        }
                    }

                    else if (140 < degree) {
                        moter(0, 1, 0.3, 0.3);
                    }
                }

                else if (50<=dist && dist != 255) {  //상대 작음, 보통

                    if (ir_val0 > val0) {
                        if (ir_val2 > val2) {
                            moter(0, 0, 0.71, 0.2);
                        }
                        else if (ir_val1 <= val1) {
                            moter(0, 0, 0.65, 0.2);
                        }
                        else {
                            moter(0, 0, 0.68, 0.2);
                        }
                    }

                    else if (ir_val2 > val2) {
                        if (ir_val0 > val0) {
                            moter(0, 0, 0.2, 0.71);
                        }
                        else if (ir_val1 <= val1) {
                            moter(0, 0, 0.2, 0.65);
                        }
                        else {
                            moter(0, 0, 0.2, 0.68);
                        }
                    }
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 100) {   // 로봇 들림

                //pc.printf("looool");

                if (imu_bool == false) {
                    mode = 1;
                }

                else if (FRoll < Rmin) {  //  뒤 들림
                    if (ir_Wh[4] == true) {
                        if (ir_Wh[6] == true) {
                            moter(0, 0, 0.3, 0.3);
                        }
                        else if (ir_Wh[6] == false) {
                            moter(1, 1, 0.5, 0.5);
                        }
                    }
                    else if (ir_Wh[4] == false) {
                        if (ir_Wh[6] == true) {
                            moter(0, 0, 0.5, 0.5);
                        }
                        else if (ir_Wh[6] == false) {
                            moter(1, 1, 0.5, 0.5);
                        }
                    }
                }

                else if (FRoll > Rmax) {  // 앞 들림
                    if (ir_Wh[5] == true) {
                        if (ir_Wh[7] == true) {
                            moter(1, 1, 0.3, 0.3);
                        }
                        else if (ir_Wh[7] == false) {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                    else if (ir_Wh[5] == false) {
                        if (ir_Wh[7] == true) {
                            moter(1, 1, 0.5, 0.5);
                        }
                        else if (ir_Wh[7] == false) {
                            moter(0, 0, 0.5, 0.5);
                        }
                    }
                }

                else {
                    if (FPitch < Pmin) {  // 오른쪽 들림
                        if (psd_val2 < 50 && psd_val4 >= 50) {
                            mode = 101;
                        }
                        else if (psd_val2 >= 50 && psd_val4 < 50) {
                            mode = 102;
                        }
                        else if (psd_val2 >= 50 && psd_val4 >= 50) {
                            mode = 103;
                        }
                        else {
                            mode = 104;
                        }
                    }

                    else if (FPitch > Pmax) {  // 왼쪽 들림
                        if (psd_val2 < 50 && psd_val4 >= 50) {
                            mode = 105;
                        }
                        else if (psd_val2 >= 50 && psd_val4 < 50) {
                            mode = 106;
                        }
                        else if (psd_val2 >= 50 && psd_val4 >= 50) {
                            mode = 107;
                        }
                        else {
                            mode = 108;
                        }
                    }
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 101) {
                if (imu_bool == false) {
                    mode = 1;
                }
                else if (ir_val6 < val6 || ir_val7 < val7) {
                    mode = 102;
                }
                else {
                    moter(1, 1, 0.5, 0.5);
                }
            }

            else if (mode == 102) {
                if (imu_bool == false) {
                    mode = 1;
                }
                else if (ir_val0 < val0 || ir_val1 < val1 || ir_val8 < val8) {
                    mode = 101;
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

            else if (mode == 103) {
                if (psd_val2 < psd_val4) {
                    mode = 101;
                }
                else if (psd_val2 >= psd_val4) {
                    mode = 102;
                }
            }

            else if (mode == 104) {
                if (psd_val2 < psd_val4) {
                    mode = 101;
                }
                else if (psd_val2 >= psd_val4) {
                    mode = 102;
                }
            }

    //-------------------------------------------------------------------------------------------------------------------

            else if (mode == 105) {
                if (imu_bool == false) {
                    mode = 1;
                }
                else if (ir_val5 < val5 || ir_val4 < val4) {
                    mode = 105;
                }
                else {
                    moter(1, 1, 0.5, 0.5);
                }
            }

            else if (mode == 106) {
                if (imu_bool == false) {
                    mode = 1;
                }
                else if (ir_val1 < val1 || ir_val2 < val2 || ir_val3 < val3) {
                    mode = 105;
                }
                else {
                    moter(0, 0, 0.5, 0.5);
                }
            }

            else if (mode == 107) {
                if (psd_val2 < psd_val4) {
                    mode = 105;
                }
                else if (psd_val2 >= psd_val4) {
                    mode = 106;
                }
            }

            else if (mode == 108) {
                if (psd_val2 < psd_val4) {
                    mode = 105;
                }
                else if (psd_val2 >= psd_val4) {
                    mode = 106;
                }
            }
        }



        Work_M=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(MCU_CONTROL_RATE-(Work_M-Now_M)));
    }
}