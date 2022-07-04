#pragma once

#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <defs.h>
#include <types.h>

namespace flicr
{
class IntrInfo
{
public:
  int intrValue;
  int intrX, intrY;

  ////////////////////
  int xWndIdx, yWndIdx;
  int xWndStart, yWndStart;
  int prevGradient;
  int curGradient;
  int intrPriority;

  void print()
  {
    printf("===== IntrInfo =====\n");
    printf("\thIndex: %d\n", hIndex);
    printf("\tvIndex: %d\n", vIndex);
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


    enum IntrIndexPolicy
    {
      LeastGradient,
      GreatestGradient
    };


    enum IntrPriority
    {
      NONZ_NONZ,
      NONZ_ZERO,
      ZERO
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


    int getNextX(int x, int riCol, int offset=1, bool circular=true)
    {
      int nextX = x+offset;
      if(circular)
        return (nextX < riCol) ? nextX : nextX-riCol;
      else
        return (nextX < riCol) ? nextX : -1;
    }

    int getPrevX(int x, int riCol, int offset=1, bool circular=true)
    {
      int prevX = x-offset;
      if(circular)
        return (prevX < 0) ? riCol-prevX : prevX;
      else
        return (prevX < 0) ? -1 : prevX;
    }


    cv::Mat interpolate(cv::Mat original, int searchWndSize, int insertions, int gradThresh, IntrIndexPolicy intrIndexPolicy=IntrIndexPolicy::LeastGradient, bool circular=true)
    {
      int xIter = original.cols/searchWndSize;
      int intrCols = xIter*(searchWndSize+insertions);

      cv::Mat outRi(original.rows, intrCols, CV_8UC1, cv::Scalar(0));

      for(int y = 0; y < original.rows; y++)
      {
        for(int xWnd = 0; xWnd < xIter; xWnd++)
        {

        }
      }




      for(int x = 0; x < hIter; x++)
      {
        for(int y = 0; y < vIter; y++)
        {
          // IntrIndexInfo info = getHorizontalIntrIndex(original, x*hWnd, y, intrIndexPolicy, gradThresh);
          IntrInfo info = getIntrIndex(original, y, x, searchWndSize, gradThresh, intrIndexPolicy, circular);

          // interpolateHorizontalWindow(original, outRi, gradThresh, x, y, info);
        }
      }

      return outRi;
    }

    // find the position within a window to interpolate by policies...
    // TODO: priority... depth-awareness
    IntrInfo getIntrIndex(cv::Mat original, int y, int x, int searchWndSize, int gradThresh, IntrIndexPolicy intrIndexPolicy, bool circular)
    {
      // search forward & backward...
      uchar curP = 0, nextP = 0;
      uchar curGrad = 0, prevGrad = 0;

      IntrInfo info;

      // non-gradient -- searchWndSize -> 1
      if(searchWndSize == 1)
      {
        int prevX, nextX;
        if(circular)
        {
          prevX  = (x-1 < 0) ? original.cols-1 : x-1;
          nextX = (x+1 >= original.cols) ? 0 : x+1;
        }
        else
        {
          prevX  = x-1;
          nextX = (x+1 >= original.cols) ? -1 : x+1;
        }

        if(prevX >= 0) curP = original.at<uchar>(y, prevX);
        if(nextX >= 0) nextP = original.at<uchar>(y, nextX);

        info.intrValue = (curP > nextP) ? curP : nextP;
        info.intrX     = (curP > nextP)
      }



      std::vector<IntrIndexInfo> firstIntrIdx;
      std::vector<IntrIndexInfo> secondIntrIdx;

      // 1. Iterate window & create IntrIndicesInfo by priority
      for(int hWndIdx = hWndStart; hWndIdx < hWndStart + hWnd; hWndIdx++)
      {
        curP     = original.at<uchar>(vIdx, hWndIdx);
        nextP    = original.at<uchar>(vIdx, hWndIdx+1);
        if(curP != 0)
        {
          prevGrad = curGrad;
          curGrad  = abs(curP - nextP);
        }
        else
        {
          prevGrad = 0;
          curGrad  = 0;
        }

        IntrIndexInfo idxInfo;
        idxInfo.hIndex       = hWndIdx;
        idxInfo.vIndex       = vIdx;
        idxInfo.curGradient  = curGrad;
        idxInfo.prevGradient = prevGrad;
        idxInfo.intrPriority = IntrPriority::ZERO;

        // Priority...
        // 1. curP is not 0 && nextP is not 0
        // 2. curP is not 0 && nextP is 0
        // 3. curP is 0
        if(curP != 0 && nextP != 0)
        {
          idxInfo.intrPriority = IntrPriority::NONZ_NONZ;
          firstIntrIdx.push_back(idxInfo);
        }
        else if(curP != 0 && nextP == 0)
        {
          idxInfo.intrPriority = IntrPriority::NONZ_ZERO;
          secondIntrIdx.push_back(idxInfo);
        }
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
              policy == IntrIndexPolicy::GreatestGradient)
            intrIdxInfo = firstIntrIdx[i];
          else if(intrIdxInfo.curGradient > firstIntrIdx[i].curGradient &&
              firstIntrIdx[i].curGradient < gradThresh &&
              policy == IntrIndexPolicy::LeastGradient)
            intrIdxInfo = firstIntrIdx[i];
        }
      }
      // 2nd priority
      else if(secondIntrIdx.size() != 0)
      {
        intrIdxInfo = secondIntrIdx[0];
        for(int i = 1; i < (int)secondIntrIdx.size(); i++)
        {
          if(intrIdxInfo.curGradient < secondIntrIdx[i].curGradient && policy == IntrIndexPolicy::GreatestGradient)
            intrIdxInfo = secondIntrIdx[i];
          else if(intrIdxInfo.curGradient > secondIntrIdx[i].curGradient && policy == IntrIndexPolicy::LeastGradient)
            intrIdxInfo = secondIntrIdx[i];
        }
      }
      // 3rd priority
      else
      {
        intrIdxInfo.hIndex        = hWndStart;
        intrIdxInfo.vIndex        = vIdx;
        intrIdxInfo.curGradient  = 0;
        intrIdxInfo.prevGradient = 0;
        intrIdxInfo.intrPriority = IntrPriority::ZERO;
      }

      intrIdxInfo.hStart = hWndStart;
      intrIdxInfo.vStart = vIdx;

      return intrIdxInfo;
    }


    // interpolate a point within a given window...
    void interpolateHorizontalWindow(cv::Mat original, cv::Mat intrRi, int gradThresh, int hIter, int vIter, IntrIndexInfo intrIndexInfo)
    {
      int intrHidx = hIter * (hWnd+1);

      for(int origHidx = intrIndexInfo.hStart; origHidx < intrIndexInfo.hStart + hWnd; origHidx++, intrHidx++)
      {
        if(origHidx == intrIndexInfo.hIndex)
        {
          switch(intrIndexInfo.intrPriority)
          {
            case IntrPriority::NONZ_NONZ:
              if(intrIndexInfo.curGradient < gradThresh)
              {
                // curGrad < thresh --> linear interpolation
                int curP = original.at<uchar>(vIter, origHidx);
                int nextP = original.at<uchar>(vIter, origHidx+1);
                double tempGrad = (nextP-curP)/2;

                intrRi.at<uchar>(vIter, intrHidx) = curP;
                intrHidx++;
                intrRi.at<uchar>(vIter, intrHidx) = curP + tempGrad;
              }
              else // if(intrIndexInfo.curGradient >= gradThresh)
              {
                // curGrad > thresh --> put empty -- distancing between objects...
                intrRi.at<uchar>(vIter, intrHidx) = original.at<uchar>(vIter, origHidx);
                intrHidx++;
                intrRi.at<uchar>(vIter, intrHidx) = 0;
              }
              break;

            case IntrPriority::NONZ_ZERO:
              if(intrIndexInfo.prevGradient == 0)
              {
                // prevGrad == 0 --> put empty
                intrRi.at<uchar>(vIter, intrHidx) = original.at<uchar>(vIter, origHidx);
                intrHidx++;
                intrRi.at<uchar>(vIter, intrHidx) = 0;
              }
              else // if(intrIndexInfo.prevGradient != 0)
              {
                // prevGrad != 0 --> put interpolated point with prevGrad
                int curP = original.at<uchar>(vIter, origHidx);
                int prevP = original.at<uchar>(vIter, origHidx-1);
                int tempGrad = curP - prevP;

                intrRi.at<uchar>(vIter, intrHidx) = curP;
                intrHidx++;
                intrRi.at<uchar>(vIter, intrHidx) = curP + tempGrad;
              }
              break;

            case IntrPriority::ZERO:
            default:
              // put empty...
              intrRi.at<uchar>(vIter, intrHidx++) = 0;
              intrRi.at<uchar>(vIter, intrHidx) = 0;
              break;
          }
        }
        else // if(origHidx != intrIndexInfo.hIndex)
        {
          intrRi.at<uchar>(vIter, intrHidx) = original.at<uchar>(vIter, origHidx);
        }
      }
    }



    void interpolateEmptySpace(cv::Mat ri, int gradWndSize, int gradThresh=5, int maxZeros=2, bool circular=true)
    {
      for(int y = 0; y < ri.rows; y++)
      {
        for(int x = 0; x < ri.cols; x++)
        {
          int zeros = zeroWalk(y, x, ri);

          if(zeros > 0)
          {
            if(zeros <= maxZeros)
            {
              fillZeros(ri, y, x, gradWndSize, gradThresh, zeros, circular);
            }
            x += (zeros-1);
          }
        }
      }
    }

    protected:
    int zeroWalk(int y, int x, cv::Mat ri)
    {
      int curP;
      int zeros = 0;
      do
      {
        curP = ri.at<uchar>(y, x);
        if(curP == 0)
        {
          zeros++;
          x++;
        }
      } while(curP == 0);

      return zeros;
    }

    protected:
    void fillZeros(cv::Mat ri, int y, int x, int wndSize, int gradThresh, int zeros, bool circular)
    {
      for(int zero = 0; zero < zeros; zero++)
      {
        int curX = x+zero;
        int wndX = curX-wndSize;

        int gradSum  = 0;
        int numGrads = 0;

        // iterate...
        for(int i = 0; i < wndSize-1; i++)
        {
          int wndIdx = wndX+i;
          int nextWndIdx = wndIdx + 1;

          if(circular && wndIdx < 0)
          {
            wndIdx = ri.cols-wndIdx;
            nextWndIdx = (nextWndIdx >= ri.cols) ? nextWndIdx-ri.cols : nextWndIdx;
          }

          int curP  = ri.at<uchar>(y, wndIdx);
          int nextP = ri.at<uchar>(y, nextWndIdx);
          if(curP != 0 && nextP != 0)
          {
            gradSum += (nextP - curP);
            numGrads++;
          }
          else
          {
            gradSum  = 0;
            numGrads = 0;
          }
        }

        if(numGrads > 0)
        {
          int grad = (gradSum/numGrads);
          if(abs(grad) < gradThresh)
          {
            int pixVal = ri.at<uchar>(y, curX-1) + (gradSum/numGrads);
            pixVal = (pixVal < 0) ? 0 : (pixVal > 255) ? 255 : pixVal;
            ri.at<uchar>(y, curX) = pixVal;
          }
        }
      }
    }

};
}

