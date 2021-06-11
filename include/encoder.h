#ifndef __PCC_ENCODER__
#define __PCC_ENCODER_

#include <cv.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libavutil/frame.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
}

#include "utils.h"

class Encoder
{
  private:
    AVCodec *enc;
    AVCodecContext *encCtx;
    AVFrame *encFrame;

  public:
    Encoder();
    void init(std::string codec, int width, int height, int br, int fps);
    cv::Mat gray2yuv(cv::Mat &inFrame);
    void encodeYUV(cv::Mat &inFrame, AVPacket &outPkt);
    void saveAsFile(AVPacket &inPkt, std::string fn);
};

#endif

