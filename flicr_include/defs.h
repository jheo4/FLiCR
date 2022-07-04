#pragma once

#define PI 3.14159266
#define RAD2DEGREE(r) (r*(180.0f/PI))
#define DEGREE2RAD(r) (r*(PI/180.0f))
//#define RAD2DEGREE (180.0f/PI)

#define INVALID_GRADIENT -99999
#define INVALID_INDEX    -1

#define MAX_INTERPOLATE_WND_SIZE 32

#define KITTI_DATASET_FREQUENCY 10

// https://bit.ly/3gHtGzf
#define HDL64_THETA_PRECISION 0.4187 // Vertical Precision
#define HDL64_PI_PRECISION 0.08      // Horizontal Precision

#define HDL64_PI_PRECISION_4500 0.08
#define HDL64_PI_PRECISION_4096 0.087890625
#define HDL64_PI_PRECISION_2048 0.17578125
#define HDL64_PI_PRECISION_1024 0.3515625
#define HDL64_PI_PRECISION_512  0.703125
#define HDL64_PI_PRECISION_256  1.40625

#define HDL64_VERTICAL_DEGREE_OFFSET (-88.0f)
#define HDL64_HORIZONTAL_DEGREE_OFFSET 180.0f
#define HDL64_VERTICAL_DEGREE   26.8f
#define HDL64_HORIZONTAL_DEGREE 360.0f

#define HDL64_MIN_RANGE 0
#define HDL64_MAX_RANGE 80


#define debug_print(...) do { \
                              fprintf(stderr, "\033[1;31m[DEBUG] \033[0;32m[FUNC] %s \033[0m", __PRETTY_FUNCTION__); \
                              fprintf(stderr, __VA_ARGS__); \
                              fprintf(stderr, "\n"); \
                            } while (0)

#include <chrono>
#define getTsNow() ( (double)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() / 1000 )
#define sleepMS(a) std::this_thread::sleep_for(std::chrono::milliseconds(a));

#include <bits/stdc++.h>
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60
static void printProgress(double percentage) {
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}


#include <dirent.h>
static int countFilesInDirectory(const char *dirName)
{
  int numScans = 0;

  DIR *dir = opendir(dirName);
  if(dir == NULL)
  {
    debug_print("invalide lidarDataPath in config.yaml");
    return 0;
  }
  else
  {
    struct dirent *ent;
    while(ent = readdir(dir))
    {
      if(!strcmp(ent->d_name, ".") || !strcmp(ent->d_name, "..")) {}
      else
      {
        numScans++;
      }
    }
  }
  closedir(dir);

  return numScans;
}

