#include "decoder.h"

Decoder::Decoder()
{
  av_register_all();
  avcodec_register_all();
}


void Decoder::init(std::string codec, int width, int height)
{
  dec = avcodec_find_decoder_by_name(codec.c_str());
  if(!dec) {
    debug_print("codec %s not found", codec.c_str());
    exit(1);
  }

  decCtx = avcodec_alloc_context3(dec);
  decCtx->width   = width;
  decCtx->height  = height;
  if (strcmp(codec.c_str(), "h264") == 0 ||
      strcmp(codec.c_str(), "h264_cuvid") == 0)
  {
    decCtx->pix_fmt = AV_PIX_FMT_YUV420P;
  }
  if (strcmp(codec.c_str(), "mjpeg") == 0) {
    decCtx->pix_fmt = AV_PIX_FMT_YUVJ420P;
  }

  int ret = avcodec_open2(decCtx, dec, NULL);
  if(ret < 0) {
    debug_print("codec failed to open with ctx");
    return;
  }

  decFrame = av_frame_alloc();
  decFrame->width = width;
  decFrame->height = height;
  decFrame->format = decCtx->pix_fmt;

  decFrameSize = av_image_get_buffer_size(decCtx->pix_fmt, decFrame->width, decFrame->height, 1);
  decFB = (uint8_t*)av_malloc(decFrameSize);

  av_image_fill_arrays(decFrame->data, decFrame->linesize, decFB, static_cast<AVPixelFormat>(decFrame->format),
                       decFrame->width, decFrame->height, 1);
}


cv::Mat Decoder::yuv2gray(cv::Mat &inFrame)
{
  cv::Mat rgbFrame, grayFrame;

  cv::cvtColor(inFrame, rgbFrame, cv::COLOR_YUV2RGB_YV12);
  cv::cvtColor(rgbFrame, grayFrame, cv::COLOR_RGB2GRAY);

  return grayFrame;
}


void Decoder::decodeYUV(uint8_t *inFrameBuffer, int inFrameSize, cv::Mat &outFrame)
{
  AVPacket decPkt;
  av_packet_from_data(&decPkt, inFrameBuffer, inFrameSize);
  decPkt.side_data_elems = 0;

  outFrame = cv::Mat::zeros(decFrame->height*1.5, decFrame->width, CV_8UC1);

  //double st = getTsNow();
  int ret = avcodec_send_packet(decCtx, &decPkt);
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

