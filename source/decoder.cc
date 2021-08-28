#include "decoder.h"

Decoder::Decoder()
{
  av_register_all();
  avcodec_register_all();

  frameWidth = 0;
  frameHeight = 0;
}


void Decoder::init(std::string codec, int width, int height)
{
  frameWidth = width;
  frameHeight = height;

  dec = avcodec_find_decoder_by_name(codec.c_str());
  if(!dec) {
    debug_print("codec %s not found", codec.c_str());
    exit(1);
  }

  decCtx = avcodec_alloc_context3(dec);
  decCtx->max_b_frames = 0;
  decCtx->time_base = AVRational{1, 60};
  decCtx->framerate = AVRational{60, 1};

  int ret = avcodec_open2(decCtx, dec, NULL);
  if(ret < 0) {
    debug_print("codec failed to open with ctx");
    return;
  }

  AVPixelFormat fmt;
  if (strcmp(codec.c_str(), "h264") == 0 ||
      strcmp(codec.c_str(), "h264_cuvid")) {
    fmt = AV_PIX_FMT_YUV420P;
  }
  if (strcmp(codec.c_str(), "mjpeg") == 0) {
    fmt = AV_PIX_FMT_YUVJ420P;
  }

  decFrame = av_frame_alloc();
  decFrame->width = frameWidth;
  decFrame->height = frameHeight;
  decFrame->format = fmt;
  decFrameSize = av_image_get_buffer_size(fmt, decFrame->width, decFrame->height, 1);
  decFB = (uint8_t*)av_malloc(decFrameSize);

  av_image_fill_arrays(decFrame->data, decFrame->linesize, decFB, fmt,
                       decFrame->width, decFrame->height, 1);
}


cv::Mat Decoder::yuv2rgb(cv::Mat &inFrame)
{
  cv::Mat rgbFrame;

  //cv::cvtColor(inFrame, rgbFrame, cv::COLOR_YUV2RGB_NV12);
  debug_print("flagA");
  if(!inFrame.empty()){
    //cv::cvtColor(inFrame, rgbFrame, cv::COLOR_YUV2RGB_I420);
    cv::cvtColor(inFrame, rgbFrame, cv::COLOR_YUV2RGB_NV12);
    debug_print("non empty -- A");
  }
  else debug_print("empty -- A");
  return rgbFrame;
}


cv::Mat Decoder::yuv2gray(cv::Mat &inFrame)
{
  cv::Mat grayFrame;
  cv::cvtColor(inFrame, grayFrame, cv::COLOR_YUV2GRAY_I420);

  return grayFrame;
}


void Decoder::decodeYUV(AVPacket &inPkt, cv::Mat &outFrame)
{
  outFrame = cv::Mat::zeros(frameHeight*1.5, frameWidth, CV_8UC1);

  //double st = getTsNow();
  int ret = avcodec_send_packet(decCtx, &inPkt);
  while(ret >= 0) {
    ret = avcodec_receive_frame(decCtx, decFrame);
    if(ret == 0) {
      av_image_copy_to_buffer(outFrame.data, outFrame.total(), decFrame->data, decFrame->linesize,
                              static_cast<AVPixelFormat>(decFrame->format), decFrame->width, decFrame->height, 1);

      //double et = getTsNow();
      //debug_print("decoding time: %f ms", et-st);
      return;
    }
  }
}


void Decoder::saveAsFile(cv::Mat &inFrame, std::string fn)
{
  cv::imwrite(fn, inFrame);
  inFrame.release();
}

