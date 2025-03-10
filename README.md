# 3-axis_Gimbal
Atmega4809를 활용한 3축 짐벌 임베디드 시스템 프로젝트입니다.

<br/><br/>

## 구현 내용

+ ### MPU6050

![image](https://github.com/user-attachments/assets/fd4315e0-d639-4202-957e-786e02ee7be6)

6축의 자이로-가속도 센서로  3축 회전(roll, pitch, yaw)의 자이로 및 가속도 값을 측정. I2C 인터페이스를 통해 ATmega4809 MCU와 통신하며, roll, pitch, yaw 데이터를 받아옴. 
<br/>
  - SCL frequency : MPU6050 데이터시트를 참조하여 400kHz로 설정.
<br/>
   - Sampling frequency : 서보 모터의 동작 주파수가 50Hz임을 고려하여 200Hz로 설정


+ ### 상보 필터

![image](https://github.com/user-attachments/assets/ff1c16ca-90d3-4cb5-97ca-b7dc2c5a5044)

가속도 센서는 중력 방향을 기반으로 기울기를 측정하고, 자이로 센서는 회전 속도(동적 움직임)을 측정하므로, 가속도 센서는 고주파 영역에서 불안정, 자이로 센서는 저주파 영역에서 불안정
<br/>
-> 상보 필터를 통해 가속도 센서의 장점과 자이로 센서의 장점을 융합하여 서로의 단점을 보완
<br/>
(𝛼 = 상보필터 상수, 0.96 ~ 0.99 사이 값을 가짐)



<br/><br/>


## 사용 부품

| 상품명       | 수량 | 금액(원) | 비고     |
|-------------|------|--------|---------|
| ATmega4809  | 1    | 0   | 학교에서 제공    |
| MG966R 서보 모터  | 3    | 19,500      |   |
| MPU6050 자이로 가속도 센서  | 1    | 3,800      |   |
| 만능 기판  | 1    | 1,370      |   |
| AA 건전지 홀더  | 1    | 1,100      |   |
| ON/OFF 스위치  | 1    | 300     |   |
| 점퍼 케이블 40p  | 1    | 850      |   |
| MCP22221 USB to UART 컨버터  | 1    | 0      | 학교에서 제공  |
| LM1085 레귤레이터  | 1    | 0      | 학교에서 제공  |

총 금액 : 26,920

<br/><br/>

## Block Diagram

![Image](https://github.com/user-attachments/assets/e93b685f-9f1d-4041-94fe-b9ae50016de8)

<br/><br/>

## 회로도

![회로도](https://github.com/user-attachments/assets/56cd2202-b614-4b39-abbf-1b404bf69390)


<br/><br/>


## Demo


https://github.com/user-attachments/assets/5dc261b4-7d32-4869-98aa-a7512ed5200d

