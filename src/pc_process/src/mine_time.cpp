#include "mine_time.hpp"


int64_t CurrentTime_ms()      //直接调用这个函数就行了，返回值最好是int64_t，long long应该也可以
    {    
       struct timeval tv;    
       gettimeofday(&tv, NULL);    //该函数在sys/time.h头文件中
       return tv.tv_sec * 1000 + tv.tv_usec / 1000;    
    } 


void Delay_ms(double time)
{
    clock_t  now = clock();
    while(clock() - now < time*1000);
    CLOCKS_PER_SEC;
    
}