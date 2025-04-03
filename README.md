# 3-axis Gimbal
![image](https://github.com/user-attachments/assets/8a0d9580-661d-46ac-a428-d63fcca138b8)


<br/><br/>

## 구현 내용

+ ### MPU6050

![image](https://github.com/user-attachments/assets/fd4315e0-d639-4202-957e-786e02ee7be6)

6축의 자이로-가속도 센서로  3축 회전(roll, pitch, yaw)의 자이로 및 가속도 값을 측정. **I2C 인터페이스**를 통해 ATmega4809 MCU와 통신하며, roll, pitch, yaw 데이터를 받아온다. 
<br/>
 + Sampling Frequency : MPU6050은 최대 1kHz의 샘플링 속도를 지원하지만, 최대 속도로 샘플링할 경우 Atmega4809가 연산량을 처리하지 못할 가능성이 있으며, 노이즈도 증가할 수 있다. 또한, 모터의 동작 주파수가 50Hz이므로 1kHz의 높은 샘플링 속도는 불필요하다. 따라서 샘플링 주파수를 **200Hz**로 설정





   
<br/><br/>


+ ### 상보 필터

![image](https://github.com/user-attachments/assets/ff1c16ca-90d3-4cb5-97ca-b7dc2c5a5044)

가속도 센서는 중력 방향을 기반으로 기울기를 측정하고, 자이로 센서는 회전 속도(동적 움직임)을 측정하므로, 가속도 센서는 고주파 영역에서 불안정, 자이로 센서는 저주파 영역에서 불안정
<br/>
-> **상보 필터**를 통해 가속도 센서의 장점과 자이로 센서의 장점을 융합하여 서로의 단점을 보완
<br/>
(𝛼 = 상보필터 상수, 0.96 ~ 0.99 사이 값을 가짐)

</br>

+ ### yaw drift

yaw(수평 회전) 값의 경우 중력에 수직이므로 가속도 센서는 측정을 못하고 오로지 자이로스코프만 이용하여 측정을 해야 함.
<br/>
-> 자이로스코프는 적분 방식으로 동작하기 때문에 시간이 지남에 따라 오차가 누적되어, 센서가 정지 상태임에도 yaw 값이 점진적으로 증가하거나 감소하는 **'yaw drift'** 현상이 발생

![image](https://github.com/user-attachments/assets/47367b5e-fe82-440c-bfbe-1372ce9f7de5)

해결 방법 : 센서 데이터를 폴링 방식으로 읽어오도록 하고 main 루프 1회를 1주기로 설정, 초기 yaw 값 변화 주기를 측정하고 평균을 계산해 소프트웨어 카운터의 주기를 설정한 뒤에 카운터가 yaw 값의 변화를 상쇄시키는 방식으로 설계.

</br>

+ ### 서보 모터 제어
![image](https://github.com/user-attachments/assets/457815c5-239c-4ba5-ac7e-591a79d5eaa8)
![image](https://github.com/user-attachments/assets/073e4078-b112-416b-a2f7-0315df9814f5)

atmega4809에 내장된 타이머를 이용하여 모터에 들어갈 pwm 신호 생성. (주기 : 20ms, -90° : 1ms, 90° : 2ms)
<br/><br/>
타이머 주기 : 클락 소스는 내부 16M 오실레이터 사용하고, 8 분주기 거침 -> 16M/8 = 2MHz, 주기 = 0.5us.
<br/>
모터 동작 주기 : 20ms 
<br/><br/>
최대 카운트 : 20m / 0.5u = 40000
<br/>
-90도 카운트 : 1m / 0.5u = 2000
<br/>
90도 카운트 : 1m / 0.5u = 4000




<br/><br/>


## 사용 부품

| 상품명       | 수량 | 금액(원) | 비고     |
|-------------|------|--------|---------|
| ATmega4809  | 1    | 0   | 학교에서 제공    |
| MG966R 서보 모터  | 3    | 9,870      |   |
| MPU6050 자이로 가속도 센서  | 1    | 3,800      |   |
| 만능 기판  | 1    | 1,370      |   |
| AA 건전지 홀더  | 1    | 1,100      |   |
| ON/OFF 스위치  | 1    | 300     |   |
| 점퍼 케이블 40p  | 1    | 850      |   |
| MCP22221 USB to UART 컨버터  | 1    | 0      | 학교에서 제공  |
| LM1085 레귤레이터  | 1    | 0      | 학교에서 제공  |

총 금액 : 17,290 원

<br/><br/>

## Block Diagram

![Image](https://github.com/user-attachments/assets/e93b685f-9f1d-4041-94fe-b9ae50016de8)

<br/><br/>

## 회로도

![회로도](https://github.com/user-attachments/assets/56cd2202-b614-4b39-abbf-1b404bf69390)


<br/><br/>


## Demo


https://github.com/user-attachments/assets/5dc261b4-7d32-4869-98aa-a7512ed5200d

