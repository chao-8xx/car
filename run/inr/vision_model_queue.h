#ifndef VISION_MODEL_QUEUE_H
#define VISION_MODEL_QUEUE_H

#define  MAX_SIZE    20                         //定义队列最大容量

//动作类型枚举              
typedef enum{               
    ACTION_STRAIGHT = 0,                        // 直行
    ACTION_LEFT = 1,                            // 左绕
    ACTION_RIGHT = 2                            // 右绕
} ActionType;               

typedef ActionType ElemType;                    //定义队列元素类型为动作类型

typedef struct Queue{               
    ElemType data[MAX_SIZE];                    //队列容量
    int front;                                  //队头
    int rear;                                   //队尾
} Queue;

class Model_Queue{
public:
void queue_init(Queue *queue);                  //初始化队列

int queue_empty(Queue *queue);                  //判断队列是否为空
int queue_full(Queue *queue);                   //判断队列是否满

int queue_adjust(Queue *queue);                 //调整队列

int queue_out(Queue *queue, ElemType* e);       //出队
int queue_in(Queue *queue, ElemType e);         //入队

int get_queue_head(Queue *queue, ElemType *e);  //获取队头数据

private:


};

extern Model_Queue queue_manager;

#endif

