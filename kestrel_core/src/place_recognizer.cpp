// SPDX-License-Identifier: Apache-2.0
//
// FBoW place recognition with an inverted index. See kestrel/loop/place_recognizer.hpp.

#include "kestrel/loop/place_recognizer.hpp"

#include <fbow/fbow.h>
#include <fbow/vocabulary_creator.h>

#include <algorithm>
#include <stdexcept>
#include <unordered_map>

namespace kestrel {
namespace {

class FbowPlaceRecognizer final : public PlaceRecognizer {
 public:
  explicit FbowPlaceRecognizer(const std::string& path) {
    vocabulary_.readFromFile(path);
    if (!vocabulary_.isValid()) {
      throw std::runtime_error("FBoW: cannot load vocabulary '" + path + "'");
    }
  }

  void add(int id, const cv::Mat& descriptors) override {
    if (descriptors.empty()) return;
    fbow::fBow bow = vocabulary_.transform(descriptors);
    for (const auto& [word, weight] : bow) inverted_[word].push_back(id);
    bows_[id] = std::move(bow);
  }

  std::vector<PlaceCandidate> query(const cv::Mat& descriptors, int max_results,
                                    const std::function<bool(int)>& exclude) const override {
    std::vector<PlaceCandidate> out;
    if (descriptors.empty() || bows_.empty()) return out;
    const fbow::fBow bow = vocabulary_.transform(descriptors);

    // Inverted index: score only keyframes sharing at least one word with the query.
    std::unordered_map<int, bool> touched;
    for (const auto& [word, weight] : bow) {
      const auto it = inverted_.find(word);
      if (it == inverted_.end()) continue;
      for (const int id : it->second) touched.emplace(id, true);
    }
    for (const auto& [id, unused] : touched) {
      if (exclude && exclude(id)) continue;
      out.push_back({id, fbow::fBow::score(bow, bows_.at(id))});
    }
    std::sort(out.begin(), out.end(),
              [](const PlaceCandidate& a, const PlaceCandidate& b) { return a.score > b.score; });
    if (static_cast<int>(out.size()) > max_results) out.resize(max_results);
    return out;
  }

  double score(int a, int b) const override {
    const auto ia = bows_.find(a), ib = bows_.find(b);
    if (ia == bows_.end() || ib == bows_.end()) return 0.0;
    return fbow::fBow::score(ia->second, ib->second);
  }

  std::size_t size() const override { return bows_.size(); }

 private:
  mutable fbow::Vocabulary vocabulary_;  // transform() is not const in FBoW
  std::unordered_map<int, fbow::fBow> bows_;
  std::unordered_map<uint32_t, std::vector<int>> inverted_;  // word → keyframes containing it
};

}  // namespace

std::unique_ptr<PlaceRecognizer> makeFbowPlaceRecognizer(const std::string& vocabulary_path) {
  return std::make_unique<FbowPlaceRecognizer>(vocabulary_path);
}

void trainFbowVocabulary(const std::vector<cv::Mat>& descriptors_per_image, int k, int levels,
                         const std::string& output_path, int threads) {
  fbow::Vocabulary vocabulary;
  fbow::VocabularyCreator creator;
  fbow::VocabularyCreator::Params params(static_cast<uint32_t>(k), levels,
                                         static_cast<uint32_t>(threads));
  creator.create(vocabulary, descriptors_per_image, "orb", params);
  vocabulary.saveToFile(output_path);
}

}  // namespace kestrel
