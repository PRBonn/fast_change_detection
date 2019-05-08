// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de), Cyrill Stachniss, University of Bonn
#include "fastcd/image_sequence.h"

#include <algorithm>
#include <unordered_map>
#include <utility>
#include "fastcd/regions_matcher_hist.h"

namespace fastcd {

ImageSequence::ImageSequence(int cache_size, int max_comparison)
    : cache_size_(cache_size), max_comparisons_(max_comparison) {}

void ImageSequence::AddImage(std::shared_ptr<ProcessedImage> image,
                             int kernel_size) {
  if (sequence_.size() == cache_size_) {
    sequence_.pop_front();
  }
  if (!sequence_.empty()) {
    for (int i = sequence_.size() - 1; i >= 0; i--) {
      if (sequence_[i].ImageCompared() < max_comparisons_) {
        sequence_[i].UpdateInconsistencies(
            image->Warp(sequence_[i]),kernel_size);
      }
      if (image->ImageCompared() < max_comparisons_) {
        image->UpdateInconsistencies(sequence_[i].Warp(*image),kernel_size);
      }
    }
  }
  sequence_.push_back(*image);
}

void ImageSequence::ComputeAndMatchRegions(int threshold_change_area) {
  for (size_t i = 0; i < sequence_.size(); i++) {
    sequence_[i].ComputeRegions(threshold_change_area);
  }

  RegionsMatcherHist regions_matcher;
  int label = 0;
  for (size_t i = 0; i < sequence_.size(); i++) {
    for (size_t j = i + 1; j < sequence_.size(); j++) {
      regions_matcher.Match(sequence_[i], sequence_[j], label);
      sequence_[i].UpdateLabels(regions_matcher.GetLabelsImg1());
      sequence_[j].UpdateLabels(regions_matcher.GetLabelsImg2());
      label = regions_matcher.GetMaxLabel();
    }
  }
}

}  // namespace fastcd
