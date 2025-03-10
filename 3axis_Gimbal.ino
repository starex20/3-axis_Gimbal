
// i2c 레지스터(MPU6050과 통신)
#define TWI_MCTRLA      (unsigned char *) (0x08A3)
#define TWI_MCTRLB      (unsigned char *) (0x08A4)
#define TWI_MSTATUS     (unsigned char *) (0x08A5)
#define TWI_MBAUD       (unsigned char *) (0x08A6)
#define TWI_MADDR       (unsigned char *) (0x08A7)
#define TWI_MDATA       (unsigned char *) (0x08A8)


// TCA0 레지스터 (서보모터 제어)
#define PORTMUX_TCA     (unsigned char *) (0x05E4)
#define TCA_CTRLA       (unsigned char *) (0x0A00)
#define TCA_CTRLB       (unsigned char *) (0x0A01)
#define TCA_PER         (unsigned short*) (0x0A26)
#define TCA_CMP0        (unsigned short*) (0x0A28)
#define TCA_CMP1        (unsigned short*) (0x0A2A)
#define TCA_CMP2        (unsigned short*) (0x0A2C)



// CPU 레지스터 (clk 분주, 전역 인터럽트 활성화)
#define _CPU_CCP        (unsigned char *) (0x0034)
#define _CPU_SREG       (unsigned char *) (0x003F)


// CLKCTRL 레지스터 (clk 분주)
#define CLKCTRL_MCTRLA  (unsigned char *) (0x0060)
#define CLKCTRL_MCTRLB  (unsigned char *) (0x0061)
#define CLKCTRL_MSTATUS (unsigned char *) (0x0063) 


// TCB0 레지스터 (ms 단위 타이머 = millis() )
#define TCB_CTRLA       (unsigned char *) (0x0A80)
#define TCB_CTRLB       (unsigned char *) (0x0A81)
#define TCB_INTCTRL     (unsigned char *) (0x0A85)
#define TCB_INTFLAGS    (unsigned char *) (0x0A86)
#define TCB_CCMP        (unsigned short*) (0x0A8C)


// MPU6050 내부 레지스터(datasheet 참조)
#define PWR_MGMT_1                        (0x6B) 
#define SMPLRT_DIV                        (0x19)
#define CONFIG                            (0x1A) 
#define GYRO_CONFIG                       (0x1B)
#define ACCEL_CONFIG                      (0x1C)
#define ACCEL_XOUT_H                      (0x3B)
#define GYRO_XOUT_H                       (0x43)


#include <avr/interrupt.h> // sei(), cli() 사용하기 위함


#define MPU               (0x68)                                  // MPU6050의 i2c address
#define TWI0_BAUD(F_SCL)  ((((float)F_CPU / (float)F_SCL)) - 10 ) // Baud rate 계산
#define I2C_SCL_FREQ      400000                                  // SCL 클락 주파수
#define CALIBRATION_CNT   100                                     // offset calibration count


static volatile uint32_t ms_cnt = 0;   // 프로그램 시작 후 몇 ms 만큼 지났는지


// roll, pitch, yaw값 받아오는 변수들
float   roll, pitch, yaw;
int16_t raw_Acc_x, raw_Acc_y, raw_Acc_z;
int16_t raw_temp;
int16_t raw_Gyro_x, raw_Gyro_y, raw_Gyro_z;
float   Acc_x, Acc_y, Acc_z;
float   Gyro_x, Gyro_y, Gyro_z;
float   angle_Acc_x, angle_Acc_y, angle_Acc_z;
float   angle_Gyro_x, angle_Gyro_y, angle_Gyro_z;
float   angle_x = 0, angle_y = 0, angle_z = 0;
float   Gyro_x_offset, Gyro_y_offset, Gyro_z_offset;
float   Gyro_x_avg, Gyro_y_avg, Gyro_z_avg;
float   fin_x, fin_y, fin_z;
float   cali_gyro;
float   AccCoef = 0.98f;
float   GyroCoef = 0.02f;


// roll, pitch, yaw값 오차 보정하는 변수들
uint8_t  sign, up_down, average=10, average_reg[15];
float    AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float    elapsedTime, currentTime, previousTime;
float    tmp;
int      tmp_map_yaw;
uint8_t  count=0, count_init=0, count_init2=0;
int      count2=0; 
int      map_roll, map_pitch, map_yaw, map_yaw_fixed;
int      final_roll, final_pitch, final_yaw;



uint32_t Mymillis();  
void TCB0_init(); 
void TCA0_init();
void I2C_0_init();
void I2C_setup();
void I2C_0_stop();
uint8_t I2C_start(uint8_t, uint8_t);
uint8_t I2C_writingPacket(uint8_t);
uint8_t I2C_receivingPacket(uint8_t);
void calcoffset(); // 각도 오차 보정
int getAngle(int); 




void setup() {
  TCB0_init();
  TCA0_init();
  I2C_0_init();
  I2C_setup();
  calcoffset();

}


void loop() {
  
  I2C_0_start(MPU, 0);                //  i2c write
  I2C_0_writingPacket(ACCEL_XOUT_H);  //  0x3B(ACCEL_XOUT_H) 레지스터에서 시작
  I2C_0_start(MPU, 1);                //  i2c read
  
  // 센서 값 읽어옴
  raw_Acc_x = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0);  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  raw_Acc_y = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0);  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  raw_Acc_z = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0);  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  raw_temp =  (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0);  // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  raw_Gyro_x = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0); // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  raw_Gyro_y = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0); // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  raw_Gyro_z = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(1); // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)

      
  
  previousTime = currentTime;        
  currentTime = Mymillis();                           // 현재시간 측정
  elapsedTime = (currentTime - previousTime) / 1000;  // 나누기 1000 : ms -> s


  // raw값 -> data
  // 16384 : 가속도 센서 Full Scale Range 2g일 때의 LSB 감도 (데이터시트 참조)
  Acc_x = ((float)raw_Acc_x) / 16384.0; 
  Acc_y = ((float)raw_Acc_y) / 16384.0;
  Acc_z = ((float)raw_Acc_z) / 16384.0;

  angle_Acc_x = atan2(Acc_y, sqrt(Acc_z * Acc_z + Acc_x * Acc_x)) * 360 / 2.0 / PI;  // X축 회전 각도 산출 식
  angle_Acc_y = atan2(Acc_x, sqrt(Acc_z * Acc_z + Acc_y * Acc_y)) * 360 / -2.0 / PI; // Y축 회전 각도 산출 식

  // 65.5 : 자이로 센서 Full Scale Range 500 deg/s 일 때의 LSB 감도 (데이터시트 참조)
  Gyro_x = ((float)raw_Gyro_x - (Gyro_x_offset)) / 65.5; 
  Gyro_y = ((float)raw_Gyro_y - (Gyro_y_offset)) / 65.5;
  Gyro_z = ((float)raw_Gyro_z - (Gyro_z_offset)) / 65.5;


  // 실제 측정 값 바탕으로 오차 보정
  Gyro_x = Gyro_x + 0.56; // GyroErrorX ~(-0.56)
  Gyro_y = Gyro_y - 2; // GyroErrorY ~(2)
  Gyro_z = Gyro_z + 0.79; // GyroErrorZ ~ (-0.8)
  
  // 상보 필터 적용
  angle_Gyro_x = 0.96 * angle_Gyro_x + 0.04 * angle_Acc_x;
  angle_Gyro_y = 0.96 * angle_Gyro_y + 0.04 * angle_Acc_y;
  
  yaw =  yaw + Gyro_z * elapsedTime;
  roll = angle_Gyro_x;
  pitch = angle_Gyro_y;
  
 //------------------------------------------------------------------------
 
 // TCA0 CMP : 2000 = 0도, 3000 : 90도, 4000 : 180도 
 // map_roll, map_pitch, map_yaw : -90.xx ~ 90.xx도를 2000, 4000으로 mapping,
 //                                각도가 실수이므로 100을 곱하여 -9000 ~ 9000으로 범위 바꿈 
 
  map_yaw = map((int)(yaw*100),-9000,9000,2000,4000);


//  yaw drift 보정 : 소프트웨어 카운터를 이용하여, yaw축 값이 1씩 증가할 때 마다 카운터도 1씩 증가하여, 증가한 만큼 다시 빼줌 
  if(count_init2 <= 14){
    
    if(tmp_map_yaw/10 + 1 == map_yaw/10 || tmp_map_yaw/10 - 1 == map_yaw/10) 
    {
      average_reg[count_init2] = count_init;
      
      if(count_init2 == 0) average = average_reg[0];
      else if (count_init2 == 2)  average = (average_reg[1] + average_reg[2])/2;
      else if (count_init2 == 5)  average = (average_reg[3] + average_reg[4] + average_reg[5])/3;
      else if (count_init2 == 9)  average = (average_reg[6] + average_reg[7] + average_reg[8] + average_reg[9])/4;
      else if (count_init2 == 14) average = (average_reg[10] + average_reg[11] + average_reg[12] + average_reg[13] + average_reg[14])/5;
      
      count_init = 0;
      count_init2++;
    }
    else
      count_init++;
  }

  if(sign == 0){
     if(tmp > yaw)     // 감소 drift
     {
        up_down = 0;
        sign = 1;  
     }
     else if(tmp < yaw) // 증가 drift
     {
        up_down = 1;
        sign = 1;
     }
  }

  if(count >= average) {
    count = 0;
  
    if(up_down) count2--;  // 증가 drift 일 땐 1씩 감소
    else        count2++;  // 감소 drift 일 땐 1씩 증가
  }
  else count++;

  
  
  map_roll = angle_Acc_x*100;
  map_pitch = (angle_Acc_y+(float)3)*100;

  map_roll = map(map_roll,-8800,8600,2000,4000);
  map_pitch = map(map_pitch,-8800,8600,2000,4000);
  

  map_yaw_fixed = map_yaw + count2*10;

  final_roll = getAngle(map_roll);
  final_pitch = getAngle(map_pitch);
  final_yaw = getAngle(map_yaw_fixed);
   
  
  
  *TCA_CMP0 = final_roll;
  *TCA_CMP1 = final_pitch; 
  *TCA_CMP2 = final_yaw; 
  
  tmp = yaw;
  tmp_map_yaw = map_yaw;

  delay(30);
}


void TCA0_init() {
   (*PORTMUX_TCA) = 0x03;                     // TCA0핀 PD0~5로 바꿈

  
  PORTD_DIR = (1<<0) | (1<<1) | (1<<2);       // PD0, PD1, PD2 : servo motor output
 
  
  *_CPU_CCP = 0xD8;                           // Configuration Change Protection 해제
  //*CLKCTRL_MCTRLB = (0<<1) | (1<<0);          // 2분주기 사용 & prescaler enable
  *CLKCTRL_MCTRLA = CLKCTRL_CLKSEL_OSC20M_gc; // 16MHz 클락 사용
  while (*CLKCTRL_MSTATUS & (1<<0) );         // clock이 안정화될 때 까지 대기


  *TCA_CTRLA = (2<<1) | (1<<0);             // 8 분주 & TCA0 enable
   
  *TCA_CTRLB = (1<<6)|(1<<5)|(1<<4) | 0x03; // CMP0,1,2 enable & sngle slope pwm 모드
 
  *TCA_PER = 40000; // TOP 
  *TCA_CMP0 = 3000; // 0도 
  *TCA_CMP1 = 3000; 
  *TCA_CMP2 = 3000;
    
    
}


void TCB0_init() {
  *_CPU_CCP    = 0xD8;                          
  *TCB_CTRLA   = 0x00;                          // 1 분주
  *TCB_CTRLB   = (1<<4);                        // Periodic Interrupt Mode
  *TCB_INTCTRL = (1<<0);                        // Capture Interrupt Enable
  *TCB_CCMP    = (1000*(F_CPU/1000000)) - 1;    // 1ms 계산 식
  *TCB_CTRLA  |= (1<<0);                        // TCB0 enable

  sei(); 
}


void I2C_0_init() {
  *TWI_MBAUD = TWI0_BAUD(I2C_SCL_FREQ);  // = 30
  *TWI_MCTRLA = (1<<0);                  // TWI enable
  *TWI_MCTRLB |= (1<<3);                 // TWI flush
  *TWI_MSTATUS |= (1<<7) | (1<<6);       // Read & write Interrupt Flag clear
  *TWI_MSTATUS |= (1<<1);                // bus 초기 상태
}

void I2C_setup() {
  I2C_0_init();

  I2C_0_start(MPU, 0);
  I2C_0_writingPacket(PWR_MGMT_1);
  I2C_0_writingPacket(0x00);        // clksel : 내부 8MHz oscillator 사용
  I2C_0_stop();

  I2C_0_start(MPU, 0);
  I2C_0_writingPacket(SMPLRT_DIV);
  I2C_0_writingPacket( 4 );        // ** Sampling rate : 1000 / (1+4) = 200Hz
  I2C_0_stop();

  I2C_0_start(MPU, 0);
  I2C_0_writingPacket(CONFIG);
  I2C_0_writingPacket( (6<<0) );    // 내부 Digital LPF의 주파수 : 5Hz, delay : 19ms 
  I2C_0_stop();

  I2C_0_start(MPU, 0);
  I2C_0_writingPacket(GYRO_CONFIG);
  I2C_0_writingPacket( (1<<3) );    // 자이로 센서의 Full scale Range : +-500 deg/s
  I2C_0_stop();

  I2C_0_start(MPU, 0);
  I2C_0_writingPacket(ACCEL_CONFIG);
  I2C_0_writingPacket(0x00);        // 가속도 센서의 Full scale Range : +- 2g
  I2C_0_stop();
}


uint8_t I2C_0_start(uint8_t address, uint8_t wr) {
  *TWI_MADDR = (address << 1) + wr;                // wr 0이면 write, 1이면 read 
  while (!(*TWI_MSTATUS & ( (1<<7) | (1<<6) )));   // write나 read interrupt flag 기다림
  if ((*TWI_MSTATUS & (1<<3) )) return 0 ;         // bus error가 나면 return 0
  return !(*TWI_MSTATUS & (1<<4) );                // mpu6050이 ack 보내오면 return 1
}

uint8_t I2C_0_writingPacket(uint8_t data) {
  while (!(*TWI_MSTATUS & (1<<6) ));               // 다 입력될 때 까지 대기
  *TWI_MDATA = data;                               
  *TWI_MCTRLB = (1<<1);
  return (!(*TWI_MSTATUS & (1<<4) ));              // mpu6050이 ack 보내오면 return 1
}

uint8_t I2C_0_receivingPacket(uint8_t acknack) {
  while (!(*TWI_MSTATUS & (1<<7) ));               // 다 읽어올 때 까지 대기
  uint8_t data = *TWI_MDATA;
  if (acknack == 0) {
    *TWI_MCTRLB = (1<<1);                          // ACK 전송
  }
  else {
    *TWI_MCTRLB = (3<<0) | (1<<2);                 // NACK 전송 & STOP
  }
  return data;
}

void I2C_0_stop(void) {
  *TWI_MCTRLB = 0x03; // STOP

}

void calcoffset() {
  int x = 0, y = 0, z = 0;
  uint16_t rx, ry, rz;

  // offset calibration logic
  for (int i = 0; i < CALIBRATION_CNT; i++) { 
    I2C_0_start(0x68, 0);
    I2C_0_writingPacket(GYRO_XOUT_H); // 0x43(GYRO_XOUT_H) 
    I2C_0_start(0x68, 1);

    //Gyro 센서 값 받아옴
    rx = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    ry = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    rz = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(1); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    x += ((float)rx); 
    y += ((float)ry);
    z += ((float)rz);
  }

  // calibration 횟수만큼 평균을 내서 보정
  Gyro_x_offset = x / CALIBRATION_CNT;
  Gyro_y_offset = y / CALIBRATION_CNT;
  Gyro_z_offset = z / CALIBRATION_CNT;
}

int getAngle(int angle){
  int Angle;
  if(angle < 2000)      Angle = 2000; // 2000 : 0도
  else if(angle > 4000) Angle = 4000; // 4000 : 180도
  else                  Angle = angle;

  return Angle;
}

uint32_t Mymillis(void)
{
  uint32_t ms_now;               // 현재 몇 ms 지났는지
  uint8_t old_sreg = *_CPU_SREG; // SREG;

  cli();                         // cnt를 읽어오는 동안 인터럽트 발생 방지
  ms_now = ms_cnt;
  *_CPU_SREG = old_sreg;

  return ms_now;
}


ISR(TCB0_INT_vect) // millis, TCB0 1ms 될 때 interrupt 발생
{
  ms_cnt++;
  //*TCB_INTFLAGS = (1<<0); // 다른 interrupt 방지
}
