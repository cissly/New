#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <wiringPi.h>
#include <time.h> 
#include "robot_moving_event.h"

pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int commandReady = 0; // 명령이 준비되었는지 여부
int stopFlag = 0;
int waitThreadCount = 0;

int goalRow = 0;
int goalCol = 0;
int goalDir = 0;

int nowRobotRow = DEFAULT_START_ROW;
int nowRobotCol = DEFAULT_START_COL;
int nowRobotDir = DEFAULT_ROBOT_DIR;

#define RIGHT_PIN_COUNT 4
#define LEFT_PIN_COUNT 4
#define DEFAULT_DELAY_TIME 5
#define THRESHOLD_SEC 2


int leftFlag = 0;
int rightFlag = 0;
// 스텝 모터 핀 배열
// A(상) B(우) A'(하) B'(좌)
// 1, 2상 제어 : 4096 스텝
int right_arr[RIGHT_PIN_COUNT] = {20, 5, 12, 16};  // 오른쪽 모터 핀
int left_arr[LEFT_PIN_COUNT] = {26, 19, 13, 6};      // 왼쪽 모터 핀

// 스텝 패턴
int one_phase[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

// 모터 초기화 함수
void init_Step(int* pin_arr) {
    for(int i = 0; i < 4; i++) {
        pinMode(pin_arr[i], OUTPUT);
    }
}

void moveFront(int* pin_arr, int delay_time, int stepNum) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(pin_arr[i], one_phase[stepNum % 8][i]);
    }
    delay(delay_time);
}
void moveBack(int* pin_arr, int delay_time, int stepNum) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(pin_arr[i], one_phase[stepNum % 8][i]);
    }
    delay(delay_time);
}
void moveLeft(int* pin_arr, int isLeft, int delay_time) {
    if (isLeft) {
        // TODO : 실험을 통해 스텝 수 조절 (직각으로 도는데 걸리는 시간 or 왼쪽으로 도는데 걸리는 시간)
        for(int i = 256 ; i > 0 ; i--) {
            moveBack(pin_arr, delay_time, i);
        }
    } else {
        // TODO : 실험을 통해 스텝 수 조절 (직각으로 도는데 걸리는 시간 or 왼쪽으로 도는데 걸리는 시간)
        for(int i = 0 ; i < 256 ; i++) {
            moveFront(pin_arr, delay_time, i);
        }
    }
}
void moveRight(int* pin_arr, int isLeft, int delay_time) {
    if (isLeft) {
        // TODO : 실험을 통해 스텝 수 조절 (직각으로 도는데 걸리는 시간 or 오른쪽으로 도는데 걸리는 시간)
        for(int i = 0 ; i < 256 ; i++) {
            moveFront(pin_arr, delay_time, i);
        }
    } else {
        // TODO : 실험을 통해 스텝 수 조절 (직각으로 도는데 걸리는 시간 or 오른쪽으로 도는데 걸리는 시간)
        for(int i = 256 ; i > 0 ; i--) {
            moveBack(pin_arr, delay_time, i);
        }
    }
}

void moveWheel(int* pin_arr, int isLeft) {
    int delay_time = DEFAULT_DELAY_TIME;
    int leftFlagDuration = 0;
    int rightFlagDuration = 0;
    time_t leftFlagStartTime = 0;
    time_t rightFlagStartTime = 0;
    // 90도 회전 or 조금 회전
    if(nowRobotDir != goalDir) {
        if(nowRobotDir == 1) {
            if(goalDir == 3) {
                // 좌회전
                moveLeft(pin_arr, isLeft, delay_time);
            } else if(goalDir == 4) {
                // 우회전
                moveRight(pin_arr, isLeft, delay_time);
            }
        } else if (nowRobotDir == 2) {
            if(goalDir == 4) {
                // 좌회전
                moveLeft(pin_arr, isLeft, delay_time);
            } else if(goalDir == 3) {
                // 우회전
                moveRight(pin_arr, isLeft, delay_time);
            }
        } else if (nowRobotDir == 3) {
            if(goalDir == 2) {
                // 좌회전
                moveLeft(pin_arr, isLeft, delay_time);
            } else if(goalDir == 1) {
                // 우회전
                moveRight(pin_arr, isLeft, delay_time);
            }
        } else if (nowRobotDir == 4) {
            if(goalDir == 1) {
                // 좌회전
                moveLeft(pin_arr, isLeft, delay_time);
            } else if(goalDir == 2) {
                // 우회전
                moveRight(pin_arr, isLeft, delay_time);
            }
        }
    }
    while(1) {
        for(int i = 0; i < 256; i++) {
            // if(ultrasoundFlag) {
            // TODO : ultrasoundFlag가 true가 될 때까지 대기
            // }

            if(stopFlag) {
                printf("stop!\n");
                return;
            }

            if (leftFlag) {
                // 플래그가 활성화된 시간 기록
                if (leftFlagStartTime == 0) {
                    leftFlagStartTime = time(NULL); 
                    // 플래그가 최초로 설정되면 임계시간을 넘지않아도 바로 회전을 할 수 있도록 설정
                    leftFlagDuration = THRESHOLD_SEC+1;
                } else{
                    leftFlagDuration = time(NULL) - leftFlagStartTime;
                }
            } else {
                // 플래그 비활성화 시 시작 시간 및 유지 시간 리셋
                leftFlagDuration = 0; 
                leftFlagStartTime = 0;
            }

            if (rightFlag) {
                // 플래그가 활성화된 시간 기록
                if (rightFlagStartTime == 0) {
                    rightFlagStartTime = time(NULL);
                    // 플래그가 최초로 설정되면 임계시간을 넘지않아도 바로 회전을 할 수 있도록 설정
                    rightFlagDuration = THRESHOLD_SEC+1;
                } else {
                    rightFlagDuration = time(NULL) - rightFlagStartTime; 
                }
                
            } else {
                // 플래그 비활성화 시 시작 시간 및 유지 시간 리셋
                rightFlagDuration = 0; 
                rightFlagStartTime = 0;
            }

            if(isLeft) {
                if (rightFlag && rightFlagDuration > THRESHOLD_SEC) {
                    rightFlagStartTime = time(NULL); 
                    delay_time -= 4;
                    if (delay_time <= 0) {
                        delay_time = 1;
                    }
                }
                if (leftFlag && leftFlagDuration > THRESHOLD_SEC) {
                    leftFlagStartTime = time(NULL); 
                    delay_time += 4;
                }
            } else {
                if (rightFlag && (rightFlagDuration > THRESHOLD_SEC)) {
                    rightFlagStartTime = time(NULL); 
                    delay_time += 4;
                }
                if (leftFlag && leftFlagDuration > THRESHOLD_SEC) {
                    leftFlagStartTime = time(NULL); 
                    delay_time -= 4;
                    if (delay_time <= 0) {
                        delay_time = 1;
                    }
                }
            }

            // 플래그가 모두 0일 때 기본 지연 시간으로 설정
            if (!leftFlag && !rightFlag) {
                delay_time = DEFAULT_DELAY_TIME; 
            }

            moveFront(pin_arr, delay_time, i);
        }
    }
}

void* leftWheelThread(void* arg) {
    while (1) {
        pthread_mutex_lock(&mutex); // 뮤텍스 잠금
        while (!commandReady) {
            waitThreadCount++;
            pthread_cond_wait(&cond, &mutex);
        }
        pthread_mutex_unlock(&mutex); // 뮤텍스 해제

        moveWheel(left_arr, 1);
    }
    return NULL;
}

void* rightWheelThread(void* arg) {
    while (1) {
        pthread_mutex_lock(&mutex); // 뮤텍스 잠금
        while (!commandReady) {
            waitThreadCount++;
            pthread_cond_wait(&cond, &mutex);
        }
        pthread_mutex_unlock(&mutex); // 뮤텍스 해제

        moveWheel(right_arr, 0);
    }
}

void* startMoveWheelThread(void* arg) {
    printf("start move\n");
    wiringPiSetupGpio(); // WiringPi 초기화
    init_Step(right_arr);
    init_Step(left_arr);

    pthread_t leftThread, rightThread;
    
    pthread_create(&leftThread, NULL, leftWheelThread, NULL);
    pthread_create(&rightThread, NULL, rightWheelThread, NULL);
    // 바퀴가 동시에 돌기 위해서 둘 다 wait 상태에 걸리도록 조금 대기해야함!!
    delay(10);

    while(1) {
        MoveDestinationTask* task = dequeue(&moveDestinationQueue);
        printf("dequeue direction\n");
        delay(10);

        goalRow = task->row;
        goalCol = task->col;
        goalDir = task->direction;
        commandReady = 1; // 명령 준비 완료
        while (1) {
            if (waitThreadCount == 2) {
                waitThreadCount = 0;
                pthread_cond_broadcast(&cond); // 모든 스레드에 신호 전송
                break;
            }
        }
        
        // 실험을 위한 코드(15초 동안 진행)
        delay(30000);
        commandReady = 0;
        stopFlag = 1;

        // 마커인식쪽에서 마커를 인식하면 인식된 마커 번호를 전달하게 하여, 목표 위치와 일치하는지 확인 및 동작 중지
        while(1) {
            MarkerRecognitionTask* marker = dequeue(&markerRecognitionLogQueue);
            nowRobotRow = marker->row;
            nowRobotCol = marker->col;
            if (goalRow == nowRobotRow && goalCol == nowRobotCol) {
                nowRobotDir = goalDir;
                commandReady = 0;
                stopFlag = 1;
                break;
            } else {
                printf("잘못된 위치입니다!\n");
                return;
                // TODO : 잘못된 위치라면 큐에 있는 경로 모두 삭제 후 aStar 호출
            }
        }
    }
    printf("end move\n");
    return NULL;
}