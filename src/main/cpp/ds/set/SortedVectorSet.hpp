#pragma once

#include <algorithm>
#include <ostream>
#include <vector>

#include "ds/set/basic_set.hpp"

namespace ds {
namespace set {

class SortedVectorSet : public basic_set<int> {
 private:
  std::vector<int> data_;
  typedef SortedVectorSet Self;

 public:
  //----------------------------------------------------------------------------
  //    Constructors
  //----------------------------------------------------------------------------
  SortedVectorSet() {}

  SortedVectorSet(std::vector<int> xs) {
    data_ = xs;
    std::sort(data_.begin(), data_.end());
    auto it = std::unique(data_.begin(), data_.end());
    data_.resize(std::distance(data_.begin(), it));
  }

  /* assignment */
  void operator=(Self const& other) { data_ = other.data_; }

  //----------------------------------------------------------------------------
  //    Equality
  //----------------------------------------------------------------------------
  friend bool operator==(Self const& lhs, Self const& rhs) { return lhs.data_ == rhs.data_; }
  friend bool operator!=(Self const& lhs, Self const& rhs) { return !(lhs == rhs); }
  explicit operator bool() const { return !data_.empty(); }
  friend bool operator<(Self const& lhs, Self const& rhs) { return lhs.data_ < rhs.data_; }
  friend bool operator>(Self const& lhs, Self const& rhs) { return lhs.data_ > rhs.data_; }
  friend bool operator<=(Self const& lhs, Self const& rhs) { return lhs.data_ <= rhs.data_; }
  friend bool operator>=(Self const& lhs, Self const& rhs) { return lhs.data_ >= rhs.data_; }

  //----------------------------------------------------------------------------
  //    Properties
  //----------------------------------------------------------------------------
  std::size_t size() const { return data_.size(); }
  bool empty() const { return data_.empty(); }
  int capacity() const { return -1; }
  std::vector<int> to_vector() const { return data_; }

  //----------------------------------------------------------------------------
  //    Operators
  //----------------------------------------------------------------------------
  /* insert element */
  Self& operator|=(int x) {
    auto it = std::lower_bound(data_.begin(), data_.end(), x);
    if (it == data_.end() || *it != x) data_.insert(it, x);
    return *this;
  }

  /* set union */
  Self& operator|=(Self const& rhs) {
    std::vector<int> data;
    std::set_union(data_.begin(), data_.end(), rhs.data_.begin(), rhs.data_.end(), std::back_inserter(data));
    data_ = data;
    return *this;
  }

  /* toggle */
  Self& operator^=(int x) {
    auto it = std::lower_bound(data_.begin(), data_.end(), x);
    if (it == data_.end() || *it != x) {
      data_.insert(it, x);
    } else {
      data_.erase(it);
    }
    return *this;
  }

  /* symmetric difference */
  Self& operator^=(Self const& rhs) {
    std::vector<int> data;
    std::set_symmetric_difference(data_.begin(), data_.end(), rhs.data_.begin(), rhs.data_.end(), std::back_inserter(data));
    data_ = data;
    return *this;
  }

  // do nothing
  Self& operator&=(int x) { return *this; }

  /* set intersection */
  Self& operator&=(Self const& rhs) {
    std::vector<int> data;
    std::set_intersection(data_.begin(), data_.end(), rhs.data_.begin(), rhs.data_.end(), std::back_inserter(data));
    data_ = data;
    return *this;
  }

  /* remove element */
  Self& operator-=(int x) {
    auto it = std::lower_bound(data_.begin(), data_.end(), x);
    if (it != data_.end() && *it == x) data_.erase(it);
    return *this;
  }

  /* set minus */
  Self& operator-=(Self const& rhs) {
    std::vector<int> data;
    std::set_difference(data_.begin(), data_.end(), rhs.data_.begin(), rhs.data_.end(), std::back_inserter(data));
    data_ = data;
    return *this;
  }

  //----------------------------------------------------------------------------
  //    Getters/setters
  //----------------------------------------------------------------------------
  bool operator[](int x) const {
    auto it = std::lower_bound(data_.begin(), data_.end(), x);
    return (it != data_.end() && *it == x);
  }

  void clear() { data_.clear(); }

  int front() const { return empty() ? -1 : data_.front(); }

  int pop_front() {
    if (empty()) return -1;
    int ret = data_.front();
    data_.erase(data_.begin());
    return ret;
  }

  int back() const { return empty() ? -1 : data_.back(); }

  int pop_back() {
    if (empty()) return -1;
    int ret = data_.back();
    data_.pop_back();
    return ret;
  }

  int at(std::size_t index) const {
    if (index < 0 || data_.size() <= index) throw std::invalid_argument("index out of range");
    return data_[index];
  }

  //----------------------------------------------------------------------------
  //    Support for range-based for-loop
  //----------------------------------------------------------------------------
  class ConstIterator {
   private:
    std::size_t index_;
    std::vector<int> const& data_;

   public:
    ConstIterator(std::size_t index, std::vector<int> const& data) : index_(index), data_(data) {}
    bool operator!=(ConstIterator const& other) const { return index_ != other.index_; }
    int const& operator*() const { return data_[index_]; }
    ConstIterator const& operator++() {
      ++index_;
      return *this;
    }
  };

  ConstIterator begin() const { return ConstIterator(0, data_); }
  ConstIterator end() const { return ConstIterator(data_.size(), data_); }

  //----------------------------------------------------------------------------
  //    Aliases
  //----------------------------------------------------------------------------
  // clang-format off
  friend Self operator|(Self const& lhs, int x) { Self ret(lhs); ret |= x; return ret; }
  friend Self operator|(Self const& lhs, Self const& rhs) { Self ret(lhs); ret |= rhs; return ret; }
  friend Self operator^(Self const& lhs, int x) { Self ret(lhs); ret ^= x; return ret; }
  friend Self operator^(Self const& lhs, Self const& rhs) { Self ret(lhs); ret ^= rhs; return ret; }
  friend Self operator&(Self const& lhs, int x) { Self ret(lhs); ret &= x; return ret; }
  friend Self operator&(Self const& lhs, Self const& rhs) { Self ret(lhs); ret &= rhs; return ret; }
  friend Self operator-(Self const& lhs, int x) { Self ret(lhs); ret -= x; return ret; }
  friend Self operator-(Self const& lhs, Self const& rhs) { Self ret(lhs); ret -= rhs; return ret; }
  // clang-format on

  void set(int x) { *this |= x; }
  void reset(int x) { *this -= x; }
  bool get(int x) const { return (*this)[x]; }
  static Self intersect(Self const& s, Self const& t) { return s & t; }
  static Self Union(Self const& s, Self const& t) { return s | t; }

  //----------------------------------------------------------------------------
  //    Printing
  //----------------------------------------------------------------------------
  friend std::ostream& operator<<(std::ostream& stream, Self const& s) {
    stream << "SortedVectorSet(";
    for (auto i = s.data_.begin(); i != s.data_.end(); ++i) stream << ((i == s.data_.begin()) ? "" : ", ") << *i;
    return stream << ")";
  }
};

}  // namespace set
}  // namespace ds
