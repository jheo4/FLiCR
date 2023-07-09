#pragma once

#include <bits/stdc++.h>
#include <pcl/common/common_headers.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <defs.h>
#include <types.h>

namespace flicr
{
class Metrics
{
  public:

  /******************************************  Point Cloud Metrics  **************************************************/
  static float getNearestDistanceFromKdtreeToPoint(const pcl::search::KdTree<pcl::PointXYZ> &tree, const pcl::PointXYZ &pt)
  {
    const int k = 1;
    std::vector<int> indices(k);
    std::vector<float> sqrDist(k);
    tree.nearestKSearch(pt, k, indices, sqrDist);

    return sqrDist[0];
  }


  static float calculateMSE(cv::Mat img1, cv::Mat img2)
  {
    int nonzero = 0;
    float mse = 0;

    for(int i = 0; i < img1.rows; i++)
    {
      for(int j = 0; j < img1.cols; j++)
      {
        if(img1.at<float>(i,j) >= 0.2)
        {
          float diff = img1.at<float>(i,j) - img2.at<float>(i,j);
          mse += pow(diff, 2);
          nonzero++;
        }
      }
    }

    mse /= nonzero;

    return mse;
  }


  static float getMseBtwPcs(types::PclPcXyz pc1, types::PclPcXyz pc2, float distThresh = std::numeric_limits<float>::max())
  {
    int outliers = 0;
    pcl::search::KdTree<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(pc1);

    float sqrdSum = std::accumulate(pc2->begin(), pc2->end(), 0.0f,
                                    [&](float currentSum, const pcl::PointXYZ& pt)
                                    {
                                      float dist = getNearestDistanceFromKdtreeToPoint(kdTree, pt);
                                      if(dist < distThresh)
                                      {
                                        return currentSum + dist;
                                      }
                                      else
                                      {
                                        outliers++;
                                        return currentSum;
                                      }
                                    }
                                   );

    //debug_print("# of outliers: %d", outliers);
    return sqrdSum / (pc2->size() - outliers);
  }


  static float calcPsnrBtwPcs(types::PclPcXyz pc1, types::PclPcXyz pc2, float pcMax)
  {
    float MSE  = getMseBtwPcs(pc1, pc2, pcMax);
    float PSNR = 10*log10(pow(pcMax, 2)/MSE);

    return PSNR;
  }


  static float calcCdBtwPcs(types::PclPcXyz pc1, types::PclPcXyz pc2)
  {
    float pc12 = getMseBtwPcs(pc1, pc2);
    float pc21 = getMseBtwPcs(pc2, pc1);

    return pc12+pc21;
  }


  static float calcPoinNumDiffBtwPcs(types::PclPcXyz pc1, types::PclPcXyz pc2)
  {
    int pc1Size = pc1->size();
    int pc2Size = pc2->size();

    return (float)(pc1Size - pc2Size) / (float)pc1Size;
  }


  /***********************************************  Image Metrics  ***************************************************/
  static double getImgPSNR(const cv::Mat& ri1, const cv::Mat& ri2, float riMax)
  {
    cv::Mat s1;
    cv::absdiff(ri1, ri2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2
    cv::Scalar s = sum(s1);         // sum elements per channel
    double sse = s.val[0]; // sum channels
    if( sse <= 1e-10) // for small values return zero
      return 0;
    else
    {
      double  mse =sse /(double)(ri1.total());
      double psnr = 10.0*log10((riMax*riMax)/mse);
      return psnr;
    }
  }


  static float getImgMSSIM( const cv::Mat& ri1, const cv::Mat& ri2)
  {
    const double C1 = 6.5025, C2 = 58.5225;
    /***************************** INITS **********************************/
    int d     = CV_32F;
    cv::Mat I1, I2;
    ri1.convertTo(I1, d);           // cannot calculate on one byte large values
    ri2.convertTo(I2, d);
    cv::Mat I2_2   = I2.mul(I2);        // I2^2
    cv::Mat I1_2   = I1.mul(I1);        // I1^2
    cv::Mat I1_I2  = I1.mul(I2);        // I1 * I2
    /*************************** END INITS **********************************/
    cv::Mat mu1, mu2;   // PRELIMINARY COMPUTING
    GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
    GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);
    cv::Mat mu1_2   =   mu1.mul(mu1);
    cv::Mat mu2_2   =   mu2.mul(mu2);
    cv::Mat mu1_mu2 =   mu1.mul(mu2);
    cv::Mat sigma1_2, sigma2_2, sigma12;
    GaussianBlur(I1_2, sigma1_2, cv::Size(11, 11), 1.5);
    sigma1_2 -= mu1_2;
    GaussianBlur(I2_2, sigma2_2, cv::Size(11, 11), 1.5);
    sigma2_2 -= mu2_2;
    GaussianBlur(I1_I2, sigma12, cv::Size(11, 11), 1.5);
    sigma12 -= mu1_mu2;
    cv::Mat t1, t2, t3;
    t1 = 2 * mu1_mu2 + C1;
    t2 = 2 * sigma12 + C2;
    t3 = t1.mul(t2);              // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))
    t1 = mu1_2 + mu2_2 + C1;
    t2 = sigma1_2 + sigma2_2 + C2;
    t1 = t1.mul(t2);               // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))
    cv::Mat ssim_map;
    divide(t3, t1, ssim_map);      // ssim_map =  t3./t1;
    cv::Scalar mssim = mean( ssim_map ); // mssim = average of ssim map
    return mssim.val[0];
  }
};
}

