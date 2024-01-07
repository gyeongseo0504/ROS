#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>


pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;


int sum;

void* func1(void* arg)
{

    for (int i = 1; i <= 100; i++)
    {
        pthread_mutex_lock(&mutex);
        sum += i;
        pthread_mutex_unlock(&mutex);
    }
    fprintf(stderr, "1���� 100������ ��: %d\n", sum);
    pthread_exit(NULL);
}

void* func2(void* arg)
{
    for (int i = 101; i <= 200; i++)
    {
        pthread_mutex_lock(&mutex);
        sum += i;
        pthread_mutex_unlock(&mutex);
    }
    fprintf(stderr, "1���� 200������ ��: %d\n", sum);
    pthread_exit(NULL);
}

void* func3(void* arg)
{
    for (int i = 201; i <= 300; i++)
    {
        pthread_mutex_lock(&mutex);
        sum += i;
        pthread_mutex_unlock(&mutex);
    }
    fprintf(stderr, "1���� 300������ ��: %d\n", sum);
    pthread_exit(NULL);
}

int calculate_total_sum(int sum1, int sum2, int sum3) 
{
    int total_sum = sum1 + sum2 + sum3;
    return total_sum;
}

int main()
{
    pthread_t tid1, tid2, tid3;

    sum = 0;


    if (pthread_create(&tid1, NULL, func1, NULL) != 0)
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }

    if (pthread_create(&tid2, NULL, func2, NULL) != 0)
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }

    if (pthread_create(&tid3, NULL, func3, NULL) != 0)
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }

    if (pthread_join(tid1, NULL) != 0)
    {
        fprintf(stderr, "pthread join error\n");
        exit(1);
    }

    if (pthread_join(tid2, NULL) != 0)
    {
        fprintf(stderr, "pthread join error\n");
        exit(1);
    }

    if (pthread_join(tid3, NULL) != 0)
    {
        fprintf(stderr, "pthread join error\n");
        exit(1);
    }
    pthread_mutex_destroy(&mutex); // mutex ���� ����

    int result = calculate_total_sum(sum1, sum2, sum3);

    printf("0~ 300 Total Sum: %d\n", result); // ���� ���

    exit(0);
}