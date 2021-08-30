#include "encoder.h"

Encoder::Encoder()
{
  av_register_all();
  avcodec_register_all();
}


void Encoder::init(std::string codec, int width, int height, int br, int fps)
{
  enc = avcodec_find_encoder_by_name(codec.c_str());
  if(!enc) {
    debug_print("codec %s not found", codec.c_str());
    exit(1);
  }

  encCtx            = avcodec_alloc_context3(enc);
  encCtx->width     = width;
  encCtx->height    = height;
  encCtx->bit_rate  = br;
  encCtx->pix_fmt   = AV_PIX_FMT_YUV420P;
  encCtx->time_base = AVRational{1, fps};
  encCtx->framerate = AVRational{fps, 1};
  encCtx->delay     = 0;
  encCtx->max_b_frames = 0;
  encCtx->gop_size  = 12;

  if (strcmp(encCtx->codec->name, "libx264") == 0)
  {
    debug_print("libx264 Setting");
    av_opt_set(encCtx->priv_data, "preset", "fast", 0);
    av_opt_set(encCtx->priv_data, "tune", "zerolatency", 0);
    av_opt_set(encCtx->priv_data, "vsink", "0", 0);
    av_opt_set_int(encCtx->priv_data, "crf", 0, AV_OPT_SEARCH_CHILDREN);
  }
  if (strcmp(encCtx->codec->name, "h264_nvenc") == 0 ||
      strcmp(encCtx->codec->name, "nvenc_h264") == 0 )
  {
    debug_print("h264_nvenc Setting");
    av_opt_set(encCtx->priv_data, "preset", "slow", 0);
    av_opt_set(encCtx->priv_data, "delay", 0, 0);
    av_opt_set(encCtx->priv_data, "vsink", "0", 0);
    //av_opt_set(encCtx->priv_data, "zerolatency", "true", 0);
    //av_opt_set(encCtx->priv_data, "2pass", "true", 0);
    //av_opt_set_int(encCtx->priv_data, "level", 51, AV_OPT_SEARCH_CHILDREN);
  }
  if (strcmp(encCtx->codec->name, "mjpeg") == 0) {
    debug_print("mjpeg Setting");
    encCtx->pix_fmt = AV_PIX_FMT_YUVJ420P;
  }

  int ret = avcodec_open2(encCtx, enc, NULL);
  if(ret < 0) {
    debug_print("codec failed to open with ctx");
    return;
  }

  encFrame = av_frame_alloc();
  encFrame->width = encCtx->width;
  encFrame->height = encCtx->height;
  encFrame->format = encCtx->pix_fmt;

  av_image_fill_arrays(encFrame->data, encFrame->linesize, NULL, static_cast<AVPixelFormat>(encFrame->format),
                       encFrame->width, encFrame->height, 1);
}


cv::Mat Encoder::rgb2yuv(cv::Mat &rgbFrame)
{
  cv::Mat yuvFrame;
  cv::cvtColor(rgbFrame, yuvFrame, cv::COLOR_RGB2YUV_I420);
  return yuvFrame;
}


cv::Mat Encoder::gray2yuv(cv::Mat &grayFrame)
{
  /*
  std::vector<cv::Mat> yuvTemp;
  yuvTemp.push_back(grayFrame);
  yuvTemp.push_back(cv::Mat::ones(grayFrame.size(), CV_8UC1));
  yuvTemp.push_back(cv::Mat::ones(grayFrame.size(), CV_8UC1));
  */
  cv::Mat rgbTemp;
  cv::cvtColor(grayFrame, rgbTemp, cv::COLOR_GRAY2RGB);
  return rgb2yuv(rgbTemp);
}


void Encoder::encodeYUV(cv::Mat &inFrame, AVPacket &outPkt)
{
  av_image_fill_arrays(encFrame->data, encFrame->linesize, inFrame.data, static_cast<AVPixelFormat>(encFrame->format),
                       encFrame->width, encFrame->height, 1);

  //double st = getTsNow();
  int ret = avcodec_send_frame(encCtx, encFrame);
  while(ret >= 0) {
    ret = avcodec_receive_packet(encCtx, &outPkt);
    if(ret == 0) {
      //double et = getTsNow();
      //debug_print("encoding time: %f ms", et-st);
      inFrame.release();
      return;
    }
  }
}


void Encoder::saveAsFile(AVPacket &inPkt, std::string fn)
{
  FILE* f = fopen(fn.c_str(), "wb");
  fwrite(inPkt.data, 1, inPkt.size, f);
  av_packet_unref(&inPkt);
  fclose(f);
}

