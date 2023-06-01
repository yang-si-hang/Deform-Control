#ifndef MINE_TIME
#define MINE_TIME
#include <sys/time.h>
#include <stdlib.h>
#include <ctime>


int64_t CurrentTime_ms();      //直接调用这个函数就行了，返回值最好是int64_t，long long应该也可以
void Delay_ms(double time);



#endif //MINE_TIME