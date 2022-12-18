#ifndef FAST_H
#define FAST_H

#include <vector>

namespace fast
{

using ::std::vector;

struct fast_xy
{
  short x, y;
  fast_xy(short x_, short y_) : x(x_), y(y_) {}
};

typedef unsigned char fast_byte;

/// NEON optimized version of the corner 9
void fast_corner_detect_9_neon(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, std::vector<fast_xy>& corners);     

/// SSE2 optimized version of the corner 9
void fast_corner_detect_9_sse2(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, std::vector<fast_xy>& corners);      

/// plain C++ version of the corner 9
void fast_corner_detect_9(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, std::vector<fast_xy>& corners); 

/// NEON optimized version of the corner 10
void fast_corner_detect_10_neon(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, std::vector<fast_xy>& corners);     

/// SSE2 optimized version of the corner 10
void fast_corner_detect_10_sse2(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, std::vector<fast_xy>& corners);      

/// plain C++ version of the corner 10
void fast_corner_detect_10(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, short barrier, std::vector<fast_xy>& corners); 

/// corner score 10
void fast_corner_score_10(const fast_byte* img, const int img_stride, const std::vector<fast_xy>& corners, const int threshold, std::vector<int>& scores);

/// Nonmax Suppression on a 3x3 Window
void fast_nonmax_3x3(const std::vector<fast_xy>& corners, const std::vector<int>& scores, std::vector<int>& nonmax_corners);

/*
// It's either a dark-center-bright-background (DARK = true) corner,
// or a bright-center-on-dark-background (DARK = false) corner.
// Don't mix them!
template<bool DARK = true>
struct fast_corner
{
   fast_corner()
      : x(-1), y(-1), score(-1) {}
   fast_corner(const fast_xy& corner, short score_)
      : x(corner.x), y(corner.y), score(score_) {}

   short x, y;
   short score;
};

typedef fast_corner<true> DarkCorner;
typedef fast_corner<false> BrightCorner;
typedef std::vector<DarkCorner> DarkCorners;
typedef std::vector<BrightCorner> BrightCorners;


/// Nonmax Suppression on a 5x5 window
void fast_nonmax_5x5(const fast_byte* img, int imgWidth, int imgHeight, int widthStep, const std::vector<fast_xy>& corners,
                     short barrier, DarkCorners& darkCorners, BrightCorners& brightCorners);

/// special corner score. Sum of the absolute radiometric differences between the center pixel and the pixels on the circle.
short corner_score_test(const fast_byte* img, const int *pointer_dir, short barrier, bool* type);
*/   

} // namespace Fast

#endif
