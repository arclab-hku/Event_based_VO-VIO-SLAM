#ifndef FAST_CORNER_UTILITIES_H
#define FAST_CORNER_UTILITIES_H

#if __ARM_NEON__
#include <arm_neon.h>
#elif __SSE2__
#include <emmintrin.h>
#endif

namespace fast
{

/// Check if the pointer is aligned to the specified byte granularity
template<int bytes> bool is_aligned(const void* ptr);
template<> inline bool is_aligned<8>(const void* ptr) { return ((reinterpret_cast<std::size_t>(ptr)) & 0x7) == 0; }
template<> inline bool is_aligned<16>(const void* ptr) { return ((reinterpret_cast<std::size_t>(ptr)) & 0xF) == 0; }


struct Less
{
   template <class T1, class T2> static bool eval(const T1 a, const T2 b)
   {
      return a < b;
   }
   static short prep_t(short pixel_val, short barrier)
   {
      return pixel_val - barrier;
   }
};

struct Greater
{
   template <class T1, class T2> static bool eval(const T1 a, const T2 b)
   {
      return a > b;
   }
   static short prep_t(short pixel_val, short barrier)
   {
      return pixel_val + barrier;
   }
};

#if __ARM_NEON__

// FIXXXME: there is a VCEQ instruction that compares to #0 by default,
// so we could get rid of the zero register. However, I didn't find
// any intrinsics for that so far, so we would have to use __asm__
//
// Note 1: _mm_movemask_epi8 does not exist on NEON, so we had to roll
//         our own, using a mask and a sequence of folds. Check neon_test.m
//         if you don't believe me that this does The Right Thing(tm)
// Note 2: we use CGT instead of CEQ to save one inversion ;-)
// Note 3: yes, there are still some pipeline bubbles, but due to the  data
//         dependencies we can't do anything about that (i.e. more registers
//         do not help in this particular case)
#define CHECK_BARRIER(lo, hi, other, flags)                                          \
{                                                                                    \
   const uint8x16_t cmpLo = vcgtq_u8(lo, other);                                     \
   const uint8x16_t cmpHi = vcgtq_u8(other, hi);                                     \
   const uint8x16_t maskedLo = vandq_u8(cmpLo, magic_mask);                          \
   const uint8x16_t maskedHi = vandq_u8(cmpHi, magic_mask);                          \
   const uint64x2_t foldedLo = vpaddlq_u32(vpaddlq_u16(vpaddlq_u8(maskedLo)));       \
   const uint64x2_t foldedHi = vpaddlq_u32(vpaddlq_u16(vpaddlq_u8(maskedHi)));       \
   const uint8x16_t finalLo = vreinterpretq_u8_u64(foldedLo);                        \
   const uint8x16_t finalHi = vreinterpretq_u8_u64(foldedHi);                        \
   flags = static_cast<int>(vgetq_lane_u8(finalHi, 8)) << 24 |                       \
           static_cast<int>(vgetq_lane_u8(finalHi, 0)) << 16 |                       \
           static_cast<int>(vgetq_lane_u8(finalLo, 8)) << 8  |                       \
           static_cast<int>(vgetq_lane_u8(finalLo, 0));                              \
}

template <bool Aligned> inline uint8x16_t load_ui128(const void* addr) { return vld1q_u8((const uint8_t*)addr); }

#elif __SSE2__

#define CHECK_BARRIER(lo, hi, other, flags)       \
  {                 \
  __m128i diff = _mm_subs_epu8(lo, other);      \
  __m128i diff2 = _mm_subs_epu8(other, hi);     \
  __m128i z = _mm_setzero_si128();        \
  diff = _mm_cmpeq_epi8(diff, z);         \
  diff2 = _mm_cmpeq_epi8(diff2, z);       \
  flags = ~(_mm_movemask_epi8(diff) | (_mm_movemask_epi8(diff2) << 16)); \
  }
     
  template <bool Aligned> inline __m128i load_si128(const void* addr) { return _mm_loadu_si128((const __m128i*)addr); }
  template <> inline __m128i load_si128<true>(const void* addr) { return _mm_load_si128((const __m128i*)addr); }

#endif

} // namespace Fast

#endif
