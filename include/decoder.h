#ifndef __PCC_DECODER__
#define __PCC_DECODER__

#include <opencv2/opencv.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libavutil/frame.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
}

#include "utils.h"

class Decoder
{
  private:
    AVCodec *dec;
    AVCodecContext *decCtx;
    AVFrame *decFrame;
    int frameWidth, frameHeight;
    uint8_t *decFB;
    int decFrameSize;

  public:
    Decoder();
    void init(std::string codec, int width, int height);
    cv::Mat yuv2rgb(cv::Mat &inFrame);
    void decodeYUV(AVPacket &inPkt, cv::Mat &outFrame);
    void saveAsFile(cv::Mat &inFrame, std::string fn);
};

#endif

