#pragma once

#include <string>
#include <iostream>
#include <algorithm>
#include <map>

#include "ThreadMutexObject.h"

class CameraInterface
{
    public:
      virtual ~CameraInterface() {}

      virtual bool ok() = 0;
      virtual std::string error() = 0;

      static const int numBuffers = 10;
      ThreadMutexObject<int> latestDepthIndex;
      std::pair<std::pair<uint8_t *,uint8_t *>,int64_t> frameBuffers[numBuffers];    //int64_t存的是时间戳,std::pair<uint8_t *,uint8_t *>存的是深度图像和RGB图像

      virtual void setAutoExposure(bool value) = 0;
      virtual void setAutoWhiteBalance(bool value) = 0;
};
