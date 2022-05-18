#pragma once

#include <opencv2/opencv.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libavutil/frame.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
}

namespace flicr
{
class Encoder
{
  private:
    AVCodec *enc;
    AVCodecContext *encCtx;
    AVFrame *encFrame;

  public:
    Encoder();
    void init(std::string codec, int width, int height, int br, int fps, int qp, int crf);
    cv::Mat rgb2yuv(cv::Mat &inFrame);
    cv::Mat gray2yuv(cv::Mat &inFrame);
    void encodeYUV(cv::Mat &inFrame, AVPacket &outPkt);
    void saveAsFile(AVPacket &inPkt, std::string fn);
};
}

