#pragma once

#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <defs.h>
#include <types.h>

namespace flicr
{
class IntrIndexInfo
{
public:
  int index;
  int prevGradient;
  int curGradient;

  void print()
  {
    printf("===== IntrIndexInfo =====\n");
    printf("\tIndex: %d\n", index);
    printf("\tprevious Gradient: %d\n", prevGradient);
    printf("\tcurrent Gradient: %d\n", curGradient);
  }
};



class RiInterpolator
{
  public:
    int origRow, origCol;
    int intrRow, intrCol;
    int hWnd, vWnd;
    int hIter, vIter;


    enum IntrPolicy
    {
      LeastGradient,
      GreatestGradient
    };



    RiInterpolator(): origRow(0), origCol(0), intrRow(0), intrCol(0),hWnd(1), vWnd(1) {}


    void setIntr(int _origRow, int _origCol, int _hWnd, int _vWnd)
    {
      this->origRow = _origRow;
      this->origCol = _origCol;
      this->hWnd    = _hWnd;
      this->vWnd    = _vWnd;

      hIter = (int)(_origCol/_hWnd);
      vIter = (int)(_origRow/_vWnd);

      if(_hWnd == 1)
      {
        intrCol = _origCol;
      }
      else
      {
        intrCol = hIter * (_hWnd + 1);
      }

      if(_vWnd == 1)
      {
        intrRow = _origRow;
      }
      else
      {
        intrRow = vIter * (_vWnd + 1);
      }
    }

    void printSetting()
    {
      printf("===== RiInterpolator Setting =====\n");
      printf("\t origRi: %dx%d\n", origRow, origCol);
      printf("\t intrRi: %dx%d\n", intrRow, intrCol);
      printf("\t vertical window size: %d, vIter %d\n", vWnd, vIter);
      printf("\t horizontal window size: %d, hIter %d\n", hWnd, hIter);
    }


    // INTER_LINEAR - a bilinear interpolation (used by default)
    // INTER_NEAREST - a nearest-neighbor interpolation
    // INTER_CUBIC - a bicubic interpolation over 4x4 pixel neighborhood
    // INTER_LANCZOS4 - a Lanczos interpolation over 8x8 pixel neighborhood
    // INTER_AREA - resampling using pixel area relation. It may be a preferred method for image decimation, as it gives moire’-free results. But when the image is zoomed, it is similar to the INTER_NEAREST method.
    cv::Mat cvInterpolate(cv::Mat original, int cvInterFormat)
    {
      cv::Mat outRi(intrRow, intrCol, CV_8UC1, cv::Scalar(0));
      cv::resize(original, outRi, cv::Size(intrCol, intrRow), cvInterFormat);
      return outRi;
    }


    IntrIndexInfo getHorizontalIntrIndex(cv::Mat original, int hWndStart, int vIdx, IntrPolicy policy, int gradThresh)
    {
      uchar curP = 0, nextP = 0;
      uchar curGrad = 0, prevGrad = 0;

      std::vector<IntrIndexInfo> firstIntrIdx;
      std::vector<IntrIndexInfo> secondIntrIdx;

      // 1. Iterate window & create IntrIndicesInfo by priority
      for(int hWndIdx = hWndStart; hWndIdx < hWndStart + hWnd; hWndIdx++)
      {
        curP     = original.at<uchar>(vIdx, hWndIdx);
        nextP    = original.at<uchar>(vIdx, hWndIdx+1);
        prevGrad = curGrad;
        curGrad  = abs(curP - nextP);

        IntrIndexInfo idxInfo;
        idxInfo.index        = hWndIdx;
        idxInfo.curGradient  = curGrad;
        idxInfo.prevGradient = prevGrad;

        // Priority...
        // 1. curP is not 0 && nextP is not 0
        // 2. curP is not 0 || nextP is not 0
        // 3. curP is 0     && nextP is 0
        if(curP != 0 && nextP != 0)
          firstIntrIdx.push_back(idxInfo);
        else if(curP != 0 && nextP == 0)
          secondIntrIdx.push_back(idxInfo);
      }

      // 2. Find the interpolation index from the created IntrIndicesInfo
      IntrIndexInfo intrIdxInfo;

      // 1st priority
      if(firstIntrIdx.size() != 0)
      {
        intrIdxInfo = firstIntrIdx[0];
        for(int i = 1; i < (int)firstIntrIdx.size(); i++)
        {
          if(intrIdxInfo.curGradient < firstIntrIdx[i].curGradient &&
              firstIntrIdx[i].curGradient < gradThresh &&
              policy == IntrPolicy::GreatestGradient)
            intrIdxInfo = firstIntrIdx[i];
          else if(intrIdxInfo.curGradient > firstIntrIdx[i].curGradient &&
              firstIntrIdx[i].curGradient < gradThresh &&
              policy == IntrPolicy::LeastGradient)
            intrIdxInfo = firstIntrIdx[i];
        }
      }
      // 2nd priority
      else if(secondIntrIdx.size() != 0)
      {
        intrIdxInfo = secondIntrIdx[0];
        for(int i = 1; i < (int)secondIntrIdx.size(); i++)
        {
          if(intrIdxInfo.curGradient < secondIntrIdx[i].curGradient && policy == IntrPolicy::GreatestGradient)
            intrIdxInfo = secondIntrIdx[i];
          else if(intrIdxInfo.curGradient > secondIntrIdx[i].curGradient && policy == IntrPolicy::LeastGradient)
            intrIdxInfo = secondIntrIdx[i];
        }
      }
      // 3rd priority
      else
      {
        intrIdxInfo.index        = hWndStart;
        intrIdxInfo.curGradient  = 0;
        intrIdxInfo.prevGradient = 0;
      }

      return intrIdxInfo;
    }


    cv::Mat interpolateHorizontal(cv::Mat original, IntrPolicy policy)
    {
      cv::Mat outRi(intrRow, intrCol, CV_8UC1, cv::Scalar(0));
      return outRi;
    }
};
}

