// SPDX-License-Identifier: Apache-2.0
//
// kestrel/loop/place_recognizer.hpp — "have I been here before?" for keyframes.
//
// TUTORIAL NOTE
// -------------
// Theory: docs/theory/08_place_recognition_and_loop_closure.md.  References: [GalvezLopez12]
// (bags of binary words), [Sivic03] (tf-idf, inverted files), [MunozSalinas] (FBoW).
//
// A keyframe's ORB descriptors are quantized against a *vocabulary* — a tree of descriptor
// clusters learned offline — into a sparse "bag of words" vector. Two images of the same place
// share many words, so the similarity of their vectors (cosine here) ranks candidate places.
// An *inverted index* (word → keyframes containing it) makes a query touch only keyframes that
// share at least one word, instead of comparing against the whole map.
//
// The recognizer only PROPOSES candidates; LoopCloser then verifies them geometrically, because
// appearance alone is fooled by repetitive structure (identical shelves, doors, corridors).
//
// Pluggable by design: PlaceRecognizer is an interface. The default backend uses FBoW (fast SIMD
// vocabulary, permissive license); DBoW2/3 or a learned global descriptor could be added later
// behind the same interface (see docs/DEV_PLAN.md).

#pragma once

#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace kestrel {

struct PlaceCandidate {
  int id = -1;         ///< keyframe id passed to add()
  double score = 0.0;  ///< similarity in [0, 1]
};

class PlaceRecognizer {
 public:
  virtual ~PlaceRecognizer() = default;

  /// Adds a keyframe's descriptors (one binary descriptor per row, CV_8U) to the database.
  virtual void add(int id, const cv::Mat& descriptors) = 0;

  /// Best `max_results` database entries for these descriptors, highest score first.
  /// `exclude(id)` returning true skips an entry (e.g. the recent past of the same trajectory).
  virtual std::vector<PlaceCandidate> query(const cv::Mat& descriptors, int max_results,
                                            const std::function<bool(int)>& exclude) const = 0;

  /// Similarity between two stored keyframes (0 if either is unknown).
  virtual double score(int id_a, int id_b) const = 0;

  /// Number of keyframes in the database.
  virtual std::size_t size() const = 0;
};

/// FBoW-backed recognizer. Throws std::runtime_error if the vocabulary cannot be loaded.
std::unique_ptr<PlaceRecognizer> makeFbowPlaceRecognizer(const std::string& vocabulary_path);

/// Trains an FBoW vocabulary (k-means tree with branching `k` and depth `levels`) from per-image
/// ORB descriptors and saves it to `output_path`. Used by the train_vocabulary tool and tests.
void trainFbowVocabulary(const std::vector<cv::Mat>& descriptors_per_image, int k, int levels,
                         const std::string& output_path, int threads = 4);

}  // namespace kestrel
