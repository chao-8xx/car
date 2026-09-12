#include "vision_model_queue.h"
#include "headfile.h"

//初始化队列
void Model_Queue::queue_init(Queue *queue)
{
    //参数保护
    assert(queue != NULL);

    queue -> front = 0;
    queue -> rear = 0;
}

//判断队列是否为空
int Model_Queue::queue_empty(Queue *queue)
{
    //参数保护
    assert(queue != NULL);

    if(queue->front == queue->rear)     //判断队头队位索引是否相等
    {
        return 1;   //队列为空时，返回1
    }
    else
    {
        return 0;   //队列不为空时，返回0
    }
}

//判断队列是否满
int Model_Queue::queue_full(Queue *queue)
{
    //参数保护
    assert(queue != NULL);

    if(queue->rear >= MAX_SIZE) //队尾索引大于等于最大值则说明队列已满
    {
        return 1;   //队列满了返回1
    }
    else
    {
        return 0;   //未满返回0
    }
}

//调整队列
int Model_Queue::queue_adjust(Queue *queue)
{
    //参数保护
    assert(queue != NULL);

    if(queue->front > 0)    //判断队头是否为0，若大于0则说明队头前面还有空间
    {
        int step = queue->front;
        // rear 指向下一个可写位置，有效数据范围是 [front, rear)。
        // 这里不能写成 <= rear，否则会多搬一个无效元素，队列满时还有越界风险。
        for(int i = queue->front; i < queue->rear; ++i) 
        {
            queue->data[i - step] = queue->data[i];     //让队头及其身后的成员往队伍前面多余的空间补
        }
        queue->front = 0;   //队头的数据补到队列最前面
        queue->rear = queue->rear - step;

        return 1;   //若成功补齐队前多余空间，返回1
    }
    else
    {
        return 0 ;  //队头已在起始位置，无需调整
    }
}

//出队
int Model_Queue::queue_out(Queue *queue, ElemType* e)
{
    //参数保护
    assert(queue != NULL);

    if(queue_empty(queue))  //判断队列是否为空
    {
        return 0;   //队列为空则返回0，出队失败
    }
    *e = queue->data[queue->front];  //队头优先出队
    queue->front ++;

    return 1;
}

//入队
int Model_Queue::queue_in(Queue *queue, ElemType e)
{
    //参数保护
    assert(queue != NULL);

    //判断调整后的队列是否满
    if(queue_full(queue)) 
    {
        if(! queue_adjust(queue))
        {
            return 0;   //队列已满，入队失败，返回0
        }
    }

    queue->data[queue->rear] = e;   //数据从队尾进入
    queue->rear ++;
    return 1;
}

//获取队头数据
int Model_Queue::get_queue_head(Queue *queue, ElemType *e)
{
    //参数保护
    assert(queue != NULL);

    if(queue_empty(queue)) return 0;    //队列为空，获取队头数据失败，返回0

    *e = queue->data[queue->front]; //解引用指针,e用于存放获取的队头数据
    return 1;   //获取队头数据成功，返回1
}

Model_Queue queue_manager;
