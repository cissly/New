# 2024 임베디드 시스템 2분반 캐리 머신
# 2024-11-19 개발 회의 내용 정리

## 일정
- 날짜: 2024년 11월 19일
- 시간: 16:00~

## 준비사항
- **팀별 5분 발표** 및 **3분 질의응답**
- 발표자료 포함 내용:
    1. **주제 소개**:
       - 주요 기능: 강의실 서빙로봇
    2. **전체 시스템 구조**:
       - 블루투스 OR 리모컨 기반
    3. **현재 위치에서 리모컨으로 주문 입력**
    4. **배송**
    5. **도착 시 스피커로 인사**:
       - 돌아다닐 때 노래 재생
       - 사람이나 장애물을 마주치면 "비켜주세요" 또는 "다른 길로 가기"
    6. **도착 시 확인 버튼**:
       - 경로 및 이동 방법: 추후 고민 예정
    7. **주요 기능별 사용 센서/액추에이터**:
       - UART 블루투스 모듈
       - 스피커
       - 거리 센서
       - 스텝 모터 <- 추가고려사항, 리프트
       - DC 모터
       - 버튼
       - 가속도 센서
    8. **업무 분장 및 개발 일정**:
       - **블루투스(주문 입력) + 비동기**: 이상진 (멀티 쓰레드)
       - **돌아다니기 + 다른 길로 가기 OR 비켜주세요 + 균형 이상 해지면**: 시완
       - **도착 시 + 노래 + 실행 되는 거**: 재연

## 질의응답 대비
1. 닌머함
2. 배달 중 요청이 오면 어떻게 하나요?
3. 로봇 경로 이탈 시 처리 방법
4. 넘어지면 어떻게 하나요? 
   - 비명 지름
5. 경고 거리 기준? 
   - 추후 변경
---

# 2024-11-24 자료 조사 내용 정리
## 개발 자료조사 및 의견 정리

- 자료조사 내용 : 추가 고려 사항인 리프트 방법 및 경로 이동 방법에 대한 조사
- 추가 내용 : 조사한 내용에 대한 자신의 의견

|  이름    | 조사 항목           | 세부 내용                                                                                           | 의견                                                                                      | 근거                                                                                     |
|--------|---------------------|----------------------------------------------------------------------------------------------------|-------------------------------------------------------|------------------------------------------------------------------------------------------|
|  김시완  | 리프트 원리         | 지게차 리프트 원리 [YouTube](https://youtu.be/bU2lPymCacI?si=279GiUGxgk2QhaCB)                     | 참고 자료로 유용                                                                         | 지게차 원리를 응용하면 리프트 구조를 설계할 수 있음                                     |
|          |                     | 가위 리프트 원리 [YouTube](https://www.youtube.com/watch?v=YjjmJ-7lmbQ)                           | 가위 리프트가 이상적이라 판단                                                           | 유압 실린더 대신 다른 대체 방안을 적용해 효율적으로 구현 가능                            |
|          | 리프트 대체 방안    | 유압 실린더 대신 스텝 모터의 회전 운동을 직선 운동으로 변환 [YouTube](https://youtu.be/xjsScOWa5Vo) | 대체 방안으로 적합                                                                      | 스텝 모터를 활용한 대체 방안이 구현 가능성이 높음                                       |
|          | 리프트 구매         | 가위 리프트 구매 [Gmarket](https://item.gmarket.co.kr/Item?goodscode=2069485619)                  | 뼈대를 구매하는 것이 현실적                                                             | 뼈대를 구매하고 내부 메커니즘만 직접 구현하는 방식이 시간 효율적                          |
|          | 위치 판단 방식      | 배민 서빙로봇 천장 마커 방식 [Brobotics](https://www.brobotics.kr/use-it/story/559)                | 천장 마커 대신 바닥 색깔 방식이 적합                                                    | 제한된 환경에서는 바닥 색깔 방식이 더 현실적이고 효과적                                  |
|          | 센서 활용           | 가속도 센서로 넘어짐, 각도 센싱 고려                                                               | 안정성 확보 가능                                                                        | 장애물 회피와 안정성 확보를 위해 센서 활용 필수                                         |
|  이상진  | 리프트 방식         | 밑에서 올리는 방식                                                                                 | 구현 난이도가 높음                                                                      | 밑에서 올리는 방식은 구조적으로 복잡하므로 비효율적                                      |
|          |                     | 위에서 올리는 방식                                                                                 | 기둥이 필요하지만 구현은 쉬움                                                           | 단순한 구현을 위해 위에서 올리는 방식을 우선 검토                                        |
|          | 위치 파악 - 마커    | 마커 인식 [Tistory](https://pg365.tistory.com/m/127)                                              | 교차로 많을 경우, 라인트레이싱과 병행 필요                                              | 마커 인식 실패를 대비해 라인트레이싱 보조가 필요.                                        |
|          | 위치 파악 - 라인    | 라인트레이싱 기술 [네이버 블로그](https://blog.naver.com/wlsepdb/222209379471)                   | 교차로 인식 추가 검토 필요                                                              | 라인트레이싱은 교차로가 없을 경우 간단히 구현 가능                                       |
|  황재연  | 리프트 방식         | 가위형 리프트                                                                                     | 구조 제작이 어렵지만 가장 적합                                                          | 가위형 리프트가 가장 효율적이지만 구현 난이도는 높음                                     |
|          |                     | 리드 스크류를 사용하여 직선 운동 변환 가능                                                        | 적합한 대체 방안으로 판단                                                               | 리드 스크류가 리프트 대체 기구로 적합                                                   |
|          | 이동 방법           | 바닥 카메라와 조명으로 색상 인식 이동                                                              | 경로 설정 및 이동 안정성 확보 가능                                                     | 바닥 색상으로 경로를 지정하면 간단한 환경에서도 구현 가능                                |
|          |                     | 카메라 한 대 사용 시 적절성 검토 필요                                                             | 추가 카메라 설치로 경로 이탈 대응 가능                                                  | 카메라 두 대를 앞뒤로 설치하면 경로 이탈 감지가 용이함                                   |
|          | 프로토타입 제안     | 리프트 대신 서빙 기능에 집중                                                                       | 기능 완성도가 중요하며 리프트는 간소화 필요                                             | 서빙 기능에 집중하여 프로토타입 정의 후 리프트는 추가 개발로 고려 가능                   |

## 정리 
- 직접적으로 들어 올리는 것 보다는 가위 리프트 방식이 효과적일 것으로 예상
- 이동 방식으로는 카메라를 이용한 천장 또는 바닥 마커 인식과 바닥 색상 인식을 이용한 라인트레이싱 방식을 통한 절대 위치 초기화 방법이 좋을 것으로 예상
- 추후 회의에서 이동 방식에 대한 결정 후 프로토타입 제작 시작, 리프트는 추가 고려 사항으로 나중에 고려

우민식 의견 : 고점 설계보다는 저점을 올리는 것을 목표로 개발 방향을 잡아야 할 듯 함.

---

# 2024-11-26 개발 회의 내용 정리
## 개인이 정하기 힘든 것들에 관한 회의

재연 : 트랙을 만들어서 작게 만들자
식 : 안된다

**라인트레이싱 + 마커**

상진 : 능동부저, 모터 때문에 PWM 개수가 부족하다

식 : DC모터 GPIO쓰면 되지 않나?

상진 : 그러면 회전어케함

식 : 되게하셈

시완 : 바퀴 두개 거꾸로 돌면 제자리 회전 된다

재연 : 재질, 부품 등

상진 : 병행성 처리 머해야됨

식 : 블루투스, 노래, 초음파, 경로, 카메라처리, 버튼 인식

## 추후 계획
주말에 하드웨어 만들기 도전
바퀴, 카메라, DC모터, 몸체

## 주말 간 내용
![image](https://github.com/user-attachments/assets/fa8d1806-8da0-4318-9315-55eca631e165)
박스 형태의 하드웨어 및 내부  


**약간의 역할 재조정**

상진 - 초음파 및 가속도 센서를 이용한 순간 정지 및 균형 탐지, 재출발

재연 - 카메라처리(마커, RGB카메라를 이용한 라인트레이싱-빨간색) 등 처리

시완 - 이동 경로(목표 설정), 마커를 통해 얻은 정보로 절대 위치 초기화, 이동 알고리즘 작성

### 구매 물품

아크릴 판 3.0\*210\*300mm 3개

아크릴 칼/칼날(후크형) 1개

바닥라인테이프 적색 25mm*33m

전기절연 테이프 흑색 1개

# 2024-12-02 임베디드 시스템 보강 B,C
## 하드웨어 제작 계획
내부
![image](https://github.com/user-attachments/assets/e9797638-ae5b-45a2-9b8e-b24b6b318a67)
외부
![image](https://github.com/user-attachments/assets/4c3e4a41-25f0-4ff5-ad29-7106827a0a0f)

## 확인 사항
DC 모터로 바퀴가 굴러가는지 확인, PWM이 아닌 softPWM을 이용하여 PWM포트를 사용하지 않음
![KakaoTalk_20241202_193611514](https://github.com/user-attachments/assets/7ce4c97a-1146-40c6-a0f5-b76d557df04d)

초음파 센서 사용법 숙지

## 제작 시 주의 사항
아크릴판 절단 과정에서 다치지 않도록 사포를 이용해 옆면 손질

# 2024-12-03 임베디드 시스템 수업
## 아크릴 보드 절단
![image](https://github.com/user-attachments/assets/882f7bfd-b56f-446d-9cbb-213bea1c03e6)

### 변경 사항
DC 모터 -> 스텝모터 변경

변경 사유 : DC모터로 바퀴 중심 맞추기 힘듦(바퀴에 맞는 모터를 구매하지 않음), 스텝모터의 토크, GPIO

### 진행 사항
아크릴 판 절단

스텝모터 연결 및 모터 확인
![image](https://github.com/user-attachments/assets/f4962789-aeac-4695-9904-2d1d9991a7f1)

# 2024-12-05 진행사항
### 스텝모터를 이용한 이동
![KakaoTalk_20241205_214932334](https://github.com/user-attachments/assets/f23e81ca-6c43-48ff-bd2e-1c0da4e7af4d)

### 회로 수정
pdf 파일로 업로드
[회로설계_1205.pdf](https://github.com/user-attachments/files/18024385/_1205.pdf)
# 2024-12-06 임베디드 시스템 강의 시간
**약간의 역할 재조정 2트**

상진 - 초음파 및 가속도 센서를 이용한 순간 정지 및 균형 탐지, 동기화, 블루투스, 큐 관리

재연 - 카메라처리(마커, RGB카메라를 이용한 라인트레이싱-빨간색)

시완 - 이동 경로(목표 설정), 마커를 통해 얻은 정보로 절대 위치 초기화, 이동 알고리즘 작성, 완료 후 복귀

### 스텝모터를 이용한 회전(라인트레이싱, 교차로에서 사용)
![KakaoTalk_20241206_135850846](https://github.com/user-attachments/assets/28bfd99b-2af1-4da2-9334-2dc8c274b7f7)

### 블루투스를 이용한 주문 큐의 인큐, 디큐
![KakaoTalk_20241206_141920053 (1)](https://github.com/user-attachments/assets/b4e54c1a-e8fa-41f2-812b-fb53e5cdd444)

### 라인트레이싱을 위한 실시간 빨간색 인식
![KakaoTalk_20241206_142714005](https://github.com/user-attachments/assets/4d0cf735-15cf-457e-bf92-910fba64005f)

# 2024-12-09 추가사항
### 사운드 테스트
이동 시 

https://github.com/user-attachments/assets/1fc5275b-68b2-41b7-ad4b-30825ab7aa66

위험 시

https://github.com/user-attachments/assets/be3d0d74-f5c5-4c17-9557-6a9718dfbf99

도착 시

https://github.com/user-attachments/assets/941deff0-1e6d-4dce-b40a-a9e696fa1953

**위험 및 이동 간 음악 변경 예정**

# 2024-12-10 임베디드 시스템 수업
### 진행 사항
- 전체 시스템 비동기 확인
- 메인 라즈베리파이 opencv 설치
- 초음파 및 부저 설치
 
**고려 사항**

배터리, opencv에서 얻은 값을 c로 넘기기, 케이스, 방향 전환, 복귀 버튼

# 2024-12-13 평일 수정
- 라인트레이싱 구현

- 배터리 탑재 후 가동 확인

https://github.com/user-attachments/assets/f991286a-3d07-49c1-8313-5c6d1b322982









