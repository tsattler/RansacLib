// Copyright (c) 2019, Torsten Sattler
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of Torsten Sattler nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// author: Torsten Sattler, torsten.sattler.de@googlemail.com

#ifndef RANSACLIB_RANSACLIB_UTILS_H_
#define RANSACLIB_RANSACLIB_UTILS_H_

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

namespace ransac_lib {
namespace utils {

// This function implements Fisher-Yates shuffling, implemented "manually"
// here following: https://lemire.me/blog/2016/10/10/a-case-study-in-the-
// performance-cost-of-abstraction-cs-stdshuffle/
void RandomShuffle(std::mt19937* rng, std::vector<int>* random_sample) {
  std::vector<int>& sample = *random_sample;
  const int kNumElements = static_cast<int>(sample.size());
  for (int i = 0; i < (kNumElements - 1); ++i) {
    std::uniform_int_distribution<int> dist(i, kNumElements - 1);
    int idx = dist(*rng);
    std::swap(sample[i], sample[idx]);
  }
}
  
void RandomShuffleAndResize(const int target_size, std::mt19937* rng,
                            std::vector<int>* random_sample) {
  RandomShuffle(rng, random_sample);
  random_sample->resize(target_size);
}

}  // namespace utils
}  // namespace ransac_lib

#endif  // RANSACLIB_RANSACLIB_UTILS_H_
