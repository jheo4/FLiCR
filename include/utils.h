#ifndef __PCC_UTILS__
#define __PCC_UTILS__

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

#endif

