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
  enum IntrDir
  {
    LEFT,
    RIGHT
  };

  int baseVal;
  int baseIntVal;
  int wndX, wndY;
  int gradient;
  IntrDir dir;

  void print()
  {
    printf("===== IntrInfo =====\n");
    printf("\tBase Value: %d\n", baseVal);
    printf("\tLocation (y, x): (%d, %d)\n", wndY, wndX);
    printf("\tInterpolation Direction: %s\n", (dir == LEFT) ? "LEFT" : "RIGHT");
    printf("\tGradient Magnitude: %d\n", gradient);
  }

  static bool compareBaseValDesc(IntrInfo info1, IntrInfo info2)
  {
    return (info1.baseVal > info2.baseVal); // for descending order by baseVal
  }

  static bool compareBaseValAsc(IntrInfo info1, IntrInfo info2)
  {
    return (info1.baseVal < info2.baseVal); // for descending order by baseVal
  }

  static bool compareWndX(IntrInfo info1, IntrInfo info2)
  {
    return (info1.wndX > info2.wndX); // for descending order by wndX
  }
};



class RiInterpolator
{
  public:
    int origRow, origCol;
    int intrRow, intrCol;
    int hWnd, vWnd;
    int hIter, vIter;

    enum InterpolationPriority
    {
      FARTHEST,
      NEAREST,
      RANDOM
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
    cv::Mat cvInterpolate(cv::Mat original, int row, int col, int cvInterFormat)
    {
      cv::Mat outRi(row, col, CV_8UC1, cv::Scalar(0));
      cv::resize(original, outRi, cv::Size(col, row), cvInterFormat);
      return outRi;
    }

    // Get the X index in RI with circular iteration
    int getRiX(int x, int riCol, int offset, bool circular)
    {
      int xOffset = x + offset;
      int remainedOffset = xOffset % riCol;

      if(remainedOffset < 0)
      {
        return circular ? riCol + remainedOffset : INVALID_INDEX;
      }
      else // remainedOffset >= 0
      {
        if(xOffset >= riCol)
          return circular ? remainedOffset : INVALID_INDEX;
        else
          return remainedOffset;
      }
    }

    // Get the gradient to the next pixel with the given curX, curY
    int getNextGradient(int curY, int curX, cv::Mat ri, bool circular, int gradThresh)
    {
      int riCols = ri.cols;

      int nextX, nextnextX;

      char curP = ri.at<char>(curY, curX);
      char nextP, nextnextP;

      int nextGrad = INVALID_GRADIENT;

      if(curP != 0) // priority -- NZ
      {
        nextX = getRiX(curX, riCols, 1, circular);
        if(nextX != INVALID_INDEX)
        {
          nextP = ri.at<char>(curY, nextX);
          nextGrad = (nextP == 0) ? INVALID_GRADIENT :
            (abs(nextP-curP) > gradThresh) ? INVALID_GRADIENT : nextP - curP;
        }
        else
        {
          nextGrad = INVALID_GRADIENT;
        }
      }
      else // curP == 0, priority -- Z
      {
        nextX = getRiX(curX, riCols, 1, circular);
        nextnextX = getRiX(curX, riCols, 2, circular);

        if(nextX != INVALID_INDEX && nextnextX != INVALID_INDEX)
        {
          nextP = ri.at<char>(curY, nextX);
          nextnextP = ri.at<char>(curY, nextnextX);
          nextGrad = (nextP == 0 || nextnextP == 0) ? INVALID_GRADIENT :
            (abs(nextnextP-nextP) > gradThresh) ? INVALID_GRADIENT : nextP - nextnextP;
        }
        else
        {
          nextGrad = INVALID_GRADIENT;
        }
      }
      return nextGrad;
    }

    // Get the gradient to the previous pixel with the given curX, curY
    int getPrevGradient(int curY, int curX, cv::Mat ri, bool circular, int gradThresh)
    {
      int riCols = ri.cols;

      int prevX, prevprevX;

      char curP = ri.at<char>(curY, curX);
      char prevP, prevprevP;

      int prevGrad = INVALID_GRADIENT;

      if(curP != 0) // priority -- NZ
      {
        prevX = getRiX(curX, riCols, -1, circular);
        if(prevX != INVALID_INDEX)
        {
          prevP = ri.at<char>(curY, prevX);
          prevGrad = (prevP == 0) ? INVALID_GRADIENT :
            (abs(curP - prevP) > gradThresh) ? INVALID_GRADIENT : prevP - curP;
        }
        else
        {
          prevGrad = INVALID_GRADIENT;
        }
      }
      else // curP == 0, priority -- Z
      {
        prevX = getRiX(curX, riCols, -1, circular);
        prevprevX = getRiX(curX, riCols, -2, circular);
        if(prevX != INVALID_INDEX && prevprevX != INVALID_INDEX)
        {
          prevP = ri.at<char>(curY, prevX);
          prevprevP =ri.at<char>(curY, prevprevX);
          prevGrad = (prevP == 0 || prevprevP == 0) ? INVALID_GRADIENT :
            (abs(prevP-prevprevP) > gradThresh) ? INVALID_GRADIENT : prevP - prevprevP;
        }
        else
        {
          prevGrad = INVALID_GRADIENT;
        }
      }
      return prevGrad;
    }


    void interpolateInWnd(cv::Mat mat, IntrInfo info, int y, int xWndStart, int xWndEnd)
    {
      int intrWndX = info.wndX + ((info.dir == IntrInfo::RIGHT) ? 1 : 0);
      int insertionX = xWndStart + intrWndX;
      for(int x = xWndEnd-1; x >= insertionX; x--)
        mat.at<char>(y, x) = mat.at<char>(y, x-1);

      mat.at<char>(y, insertionX) = info.baseVal + ((info.baseVal == 0) ? info.gradient : info.gradient/2);
    }


    void interpolateInWnd(cv::Mat mat, cv::Mat matInt, IntrInfo info, int y, int xWndStart, int xWndEnd)
    {
      int intrWndX = info.wndX + ((info.dir == IntrInfo::RIGHT) ? 1 : 0);
      int insertionX = xWndStart + intrWndX;
      for(int x = xWndEnd-1; x >= insertionX; x--)
      {
        mat.at<char>(y, x) = mat.at<char>(y, x-1);
        matInt.at<char>(y, x) = matInt.at<char>(y, x-1);
      }

      mat.at<char>(y, insertionX)    = info.baseVal + ((info.baseVal == 0) ? info.gradient : info.gradient/2);
      matInt.at<char>(y, insertionX) = info.baseIntVal;
    }


    void interpolate(cv::Mat &original, cv::Mat &result, int sWndSize, int insertions, bool circular, int gradThresh, InterpolationPriority intrPriority = InterpolationPriority::FARTHEST)
    {
      if(original.cols % sWndSize != 0)
      {
        debug_print("invalid sWndSize: original.cols %d, sWndSize %d", original.cols, sWndSize);
        exit(0);
      }

      int xIter = original.cols / sWndSize;
      int riCols = original.cols;
      int intrCols = xIter * (sWndSize+insertions);
      result = cv::Mat(original.rows, intrCols, CV_8UC1, cv::Scalar(0));

      debug_print("xIter: %d, riCols: %d", xIter, riCols);

      for(int y = 0; y < original.rows; y++) // column iteration...
      {
        for(int xWnd = 0; xWnd < xIter; xWnd++) // X windows (row) iteration...
        {

          /* 1. Find the interpolation info */
          int xWndStart     = xWnd     * sWndSize;
          int xWndEnd       = (xWnd+1) * sWndSize;
          int xIntrWndStart = xWnd     * (sWndSize+insertions);
          int xIntrWndEnd   = (xWnd+1) * (sWndSize+insertions);

          char curP;
          int zNextX, zPrevX;

          int nextGrad, prevGrad, leastGrad;

          // Interpolation Infos within a window...
          std::vector<IntrInfo> nzIntrInfos;
          std::vector<IntrInfo> zIntrInfos;
          std::vector<IntrInfo> ivIntrInfos;
          std::vector<IntrInfo> intrInfos;

          for(int x=xWndStart, xWndIdx=0, intrX=xIntrWndStart; x < xWndEnd; x++, xWndIdx++, intrX++) // iterations in each X window...
          {
            curP = original.at<char>(y, x);
            result.at<char>(y, intrX) = curP;

            if(x == xWndStart || curP == 0) // NZ && first in window / Z -- next/prev (nextnext/prevprev)
            {
              nextGrad = getNextGradient(y, x, original, circular, gradThresh);
              prevGrad = getPrevGradient(y, x, original, circular, gradThresh);
            }
            else // NZ -- next
            {
              nextGrad = getNextGradient(y, x, original, circular, gradThresh);
              prevGrad = INVALID_GRADIENT;
            }

            if(nextGrad != INVALID_GRADIENT && prevGrad != INVALID_GRADIENT)
              leastGrad = (abs(nextGrad) < abs(prevGrad)) ? nextGrad : prevGrad;
            else if(nextGrad == INVALID_GRADIENT && prevGrad != INVALID_GRADIENT)
              leastGrad = prevGrad;
            else if(nextGrad != INVALID_GRADIENT && prevGrad == INVALID_GRADIENT)
              leastGrad = nextGrad;
            else
              leastGrad = INVALID_GRADIENT;

            IntrInfo intrInfo;
            if(leastGrad == INVALID_GRADIENT || leastGrad == -INVALID_GRADIENT) // ivIntrInfos
            {
              intrInfo.gradient = 0;
              intrInfo.wndX = xWndIdx;
              intrInfo.wndY = y;
              intrInfo.dir  = IntrInfo::RIGHT;
              intrInfo.baseVal = 0;
              ivIntrInfos.push_back(intrInfo);
            }
            else
            {
              intrInfo.gradient = leastGrad;
              intrInfo.wndX = xWndIdx;
              intrInfo.wndY = y;
              intrInfo.dir  = (abs(nextGrad) < abs(prevGrad)) ? IntrInfo::RIGHT : IntrInfo::LEFT;
              if(curP == 0) // zIntrInfos
              {
                zNextX = getRiX(x, riCols, 1, circular);
                zPrevX = getRiX(x, riCols, -1, circular);
                intrInfo.baseVal = (abs(nextGrad) < abs(prevGrad)) ? original.at<char>(y, zNextX) : original.at<char>(y, zPrevX);
                zIntrInfos.push_back(intrInfo);
              }
              else // nzIntrInfos
              {
                intrInfo.baseVal = curP;
                nzIntrInfos.push_back(intrInfo);
              }
            }
          }


          // Sort nzIntrInfos, zIntrInfos by farthest priority
          // | nzIntrInfos | zIntrInfos | ivIntrInfos |
          // |     insertions     |
          int zIntrBase = nzIntrInfos.size();
          int ivIntrBase = nzIntrInfos.size() + zIntrInfos.size();

          if(intrPriority == FARTHEST)
          {
            std::sort(nzIntrInfos.begin(), nzIntrInfos.end(), IntrInfo::compareBaseValDesc);
            if(insertions > (int)nzIntrInfos.size())
            {
              std::sort(zIntrInfos.begin(), zIntrInfos.end(), IntrInfo::compareBaseValDesc);
            }
          }

          if(intrPriority == NEAREST)
          {
            std::sort(nzIntrInfos.begin(), nzIntrInfos.end(), IntrInfo::compareBaseValAsc);
            if(insertions > (int)nzIntrInfos.size())
            {
              std::sort(zIntrInfos.begin(), zIntrInfos.end(), IntrInfo::compareBaseValAsc);
            }
          }

          for(int insertion = 0; insertion < insertions; insertion++)
          {
            IntrInfo curInfo;
            if(insertion < zIntrBase)
              curInfo = nzIntrInfos[insertion];
            else if(zIntrBase <= insertion && insertion < ivIntrBase)
              curInfo = zIntrInfos[insertion-zIntrBase];
            else
              curInfo = ivIntrInfos[insertion-ivIntrBase];
            intrInfos.push_back(curInfo);
          }
          std::sort(intrInfos.begin(), intrInfos.end(), IntrInfo::compareWndX);

          for(int insertion = 0; insertion < insertions; insertion++)
          {
              interpolateInWnd(result, intrInfos[insertion], y, xIntrWndStart, xIntrWndEnd);
          }
        }
      }
    }


    void interpolate(cv::Mat &original, cv::Mat &originalInt, cv::Mat &result, cv::Mat &resultInt,
        int sWndSize, int insertions, bool circular, int gradThresh,
        InterpolationPriority intrPriority = InterpolationPriority::FARTHEST)
    {
      if(original.cols % sWndSize != 0)
      {
        debug_print("invalid sWndSize: original.cols %d, sWndSize %d", original.cols, sWndSize);
        exit(0);
      }

      int xIter = original.cols / sWndSize;
      int riCols = original.cols;
      int intrCols = xIter * (sWndSize+insertions);
      result    = cv::Mat(original.rows, intrCols, CV_8UC1, cv::Scalar(0));
      resultInt = cv::Mat(originalInt.rows, intrCols, CV_8UC1, cv::Scalar(0));

      debug_print("xIter: %d, riCols: %d", xIter, riCols);

      for(int y = 0; y < original.rows; y++) // column iteration...
      {
        for(int xWnd = 0; xWnd < xIter; xWnd++) // X windows (row) iteration...
        {

          /* 1. Find the interpolation info */
          int xWndStart     = xWnd     * sWndSize;
          int xWndEnd       = (xWnd+1) * sWndSize;
          int xIntrWndStart = xWnd     * (sWndSize+insertions);
          int xIntrWndEnd   = (xWnd+1) * (sWndSize+insertions);

          char curP, curInt;
          int zNextX, zPrevX;

          int nextGrad, prevGrad, leastGrad;

          // Interpolation Infos within a window...
          std::vector<IntrInfo> nzIntrInfos;
          std::vector<IntrInfo> zIntrInfos;
          std::vector<IntrInfo> ivIntrInfos;
          std::vector<IntrInfo> intrInfos;

          for(int x=xWndStart, xWndIdx=0, intrX=xIntrWndStart; x < xWndEnd; x++, xWndIdx++, intrX++) // iterations in each X window...
          {
            curP = original.at<char>(y, x);
            result.at<char>(y, intrX) = curP;

            curInt = originalInt.at<char>(y, x);
            resultInt.at<char>(y, intrX) = curInt;

            if(x == xWndStart || curP == 0) // NZ && first in window / Z -- next/prev (nextnext/prevprev)
            {
              nextGrad = getNextGradient(y, x, original, circular, gradThresh);
              prevGrad = getPrevGradient(y, x, original, circular, gradThresh);
            }
            else // NZ -- next
            {
              nextGrad = getNextGradient(y, x, original, circular, gradThresh);
              prevGrad = INVALID_GRADIENT;
            }

            if(nextGrad != INVALID_GRADIENT && prevGrad != INVALID_GRADIENT)
              leastGrad = (abs(nextGrad) < abs(prevGrad)) ? nextGrad : prevGrad;
            else if(nextGrad == INVALID_GRADIENT && prevGrad != INVALID_GRADIENT)
              leastGrad = prevGrad;
            else if(nextGrad != INVALID_GRADIENT && prevGrad == INVALID_GRADIENT)
              leastGrad = nextGrad;
            else
              leastGrad = INVALID_GRADIENT;

            IntrInfo intrInfo;
            if(leastGrad == INVALID_GRADIENT || leastGrad == -INVALID_GRADIENT) // ivIntrInfos
            {
              intrInfo.gradient = 0;
              intrInfo.wndX = xWndIdx;
              intrInfo.wndY = y;
              intrInfo.dir  = IntrInfo::RIGHT;
              intrInfo.baseVal = 0;
              intrInfo.baseIntVal = 0;
              ivIntrInfos.push_back(intrInfo);
            }
            else
            {
              intrInfo.gradient = leastGrad;
              intrInfo.wndX = xWndIdx;
              intrInfo.wndY = y;
              intrInfo.dir  = (abs(nextGrad) < abs(prevGrad)) ? IntrInfo::RIGHT : IntrInfo::LEFT;
              if(curP == 0) // zIntrInfos
              {
                zNextX = getRiX(x, riCols, 1, circular);
                zPrevX = getRiX(x, riCols, -1, circular);
                intrInfo.baseVal    = (abs(nextGrad) < abs(prevGrad)) ? original.at<char>(y, zNextX) : original.at<char>(y, zPrevX);
                intrInfo.baseIntVal = (abs(nextGrad) < abs(prevGrad)) ? originalInt.at<char>(y, zNextX) : originalInt.at<char>(y, zPrevX);
                zIntrInfos.push_back(intrInfo);
              }
              else // nzIntrInfos
              {
                intrInfo.baseVal = curP;
                intrInfo.baseIntVal = curInt;
                nzIntrInfos.push_back(intrInfo);
              }
            }
          }


          // Sort nzIntrInfos, zIntrInfos by farthest priority
          // | nzIntrInfos | zIntrInfos | ivIntrInfos |
          // |     insertions     |
          int zIntrBase = nzIntrInfos.size();
          int ivIntrBase = nzIntrInfos.size() + zIntrInfos.size();

          if(intrPriority == FARTHEST)
          {
            std::sort(nzIntrInfos.begin(), nzIntrInfos.end(), IntrInfo::compareBaseValDesc);
            if(insertions > (int)nzIntrInfos.size())
            {
              std::sort(zIntrInfos.begin(), zIntrInfos.end(), IntrInfo::compareBaseValDesc);
            }
          }

          if(intrPriority == NEAREST)
          {
            std::sort(nzIntrInfos.begin(), nzIntrInfos.end(), IntrInfo::compareBaseValAsc);
            if(insertions > (int)nzIntrInfos.size())
            {
              std::sort(zIntrInfos.begin(), zIntrInfos.end(), IntrInfo::compareBaseValAsc);
            }
          }

          for(int insertion = 0; insertion < insertions; insertion++)
          {
            IntrInfo curInfo;
            if(insertion < zIntrBase)
              curInfo = nzIntrInfos[insertion];
            else if(zIntrBase <= insertion && insertion < ivIntrBase)
              curInfo = zIntrInfos[insertion-zIntrBase];
            else
              curInfo = ivIntrInfos[insertion-ivIntrBase];
            intrInfos.push_back(curInfo);
          }
          std::sort(intrInfos.begin(), intrInfos.end(), IntrInfo::compareWndX);

          for(int insertion = 0; insertion < insertions; insertion++)
          {
              interpolateInWnd(result, resultInt, intrInfos[insertion], y, xIntrWndStart, xIntrWndEnd);
          }
        }
      }
    }



    // Trial...
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

