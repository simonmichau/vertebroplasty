#ifndef MYLIB_H
#define MYLIB_H

#include "MyLib_global.h"



class MYLIB_EXPORT MyLib
{
public:
    MyLib();
    static int renderDepthBuffer(const short* depthBuffer, int width, int height, short* shadedBuffer);
};

#endif // MYLIB_H
