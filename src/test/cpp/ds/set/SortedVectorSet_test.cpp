#include <gtest/gtest.h>

#include "ds/set/SortedVectorSet.hpp"
#include "util/Random.hpp"

using namespace std;
using namespace ds::set;

typedef std::vector<int> VI;

TEST(SortedVectorSetTest, BasicOperations) {
  auto s = SortedVectorSet();
  EXPECT_TRUE(s.empty());

  EXPECT_EQ(s.capacity(), -1);

  s.set(0);
  EXPECT_FALSE(s.empty());
  EXPECT_TRUE(s.get(0));
  EXPECT_FALSE(s.get(1));
  EXPECT_EQ(s.size(), 1);
  EXPECT_EQ(s.to_vector(), VI({0}));

  s.reset(0);
  EXPECT_TRUE(s.empty());
  EXPECT_FALSE(s.get(0));
  EXPECT_FALSE(s.get(1));
  EXPECT_EQ(s.size(), 0);
  EXPECT_EQ(s.to_vector(), VI());

  s.set(1024);
  s.set(1025);
  s.set(1023);
  s.set(1020);
  s.set(1024);
  s.set(1020);
  s.set(1019);
  EXPECT_FALSE(s.empty());
  EXPECT_TRUE(s.get(1019));
  EXPECT_TRUE(s.get(1020));
  EXPECT_FALSE(s.get(1021));
  EXPECT_FALSE(s.get(1022));
  EXPECT_TRUE(s.get(1023));
  EXPECT_TRUE(s.get(1024));
  EXPECT_TRUE(s.get(1025));
  EXPECT_EQ(s.size(), 5);

  s.reset(1024);
  EXPECT_FALSE(s.get(1024));
  EXPECT_EQ(s.size(), 4);

  s.clear();
  EXPECT_TRUE(s.empty());
  EXPECT_FALSE(s.get(1019));
  EXPECT_EQ(s.size(), 0);
}

TEST(SortedVectorSetTest, RandomInput) {
  util::Random rand(12345);

  auto b = SortedVectorSet();
  std::set<int> s;

  for (int t = 0; t < 10; ++t) {
    b.clear();
    s.clear();

    for (int i = 0; i < 100; ++i) {
      int x = rand.randint(0, 10000);
      int y = rand.randint(5000, 10000);
      b.set(x);
      EXPECT_TRUE(b.get(x));
      b.reset(y);
      EXPECT_FALSE(b.get(y));

      s.insert(x);
      s.erase(y);

      EXPECT_EQ(b.size(), s.size());
      EXPECT_EQ(b.size(), b.to_vector().size());

      auto b2 = b;
      auto s2 = s;
      EXPECT_EQ(b2.size(), s2.size());
      EXPECT_EQ(b2.size(), b2.to_vector().size());
    }
  }
}

TEST(SortedVectorSetTest, PopFront) {
  auto s = SortedVectorSet();
  s.set(10);
  s.set(20);
  s.set(5);
  EXPECT_EQ(s.front(), 5);
  EXPECT_EQ(s.pop_front(), 5);
  EXPECT_EQ(s.pop_front(), 10);
  EXPECT_EQ(s.pop_front(), 20);
  EXPECT_EQ(s.pop_front(), -1);
}

TEST(SortedVectorSetTest, PopBack) {
  auto s = SortedVectorSet();
  s.set(10);
  s.set(20);
  s.set(5);
  EXPECT_EQ(s.back(), 20);
  EXPECT_EQ(s.pop_back(), 20);
  EXPECT_EQ(s.pop_back(), 10);
  EXPECT_EQ(s.pop_back(), 5);
  EXPECT_EQ(s.pop_back(), -1);
}

TEST(SortedVectorSetTest, Equality) {
  EXPECT_EQ(SortedVectorSet({1, 2}), SortedVectorSet({2, 1}));
  EXPECT_NE(SortedVectorSet({1, 2}), SortedVectorSet({1}));
  EXPECT_EQ(SortedVectorSet({2, 2}), SortedVectorSet({2}));
  EXPECT_FALSE(SortedVectorSet());
  EXPECT_TRUE(SortedVectorSet({0}));
  EXPECT_TRUE(SortedVectorSet({1}));
  EXPECT_TRUE(SortedVectorSet({0, 1, 3}) < SortedVectorSet({0, 2}));
  EXPECT_FALSE(SortedVectorSet({0, 1, 3}) > SortedVectorSet({0, 2}));
  EXPECT_TRUE(SortedVectorSet({0, 1, 3}) <= SortedVectorSet({0, 2}));
  EXPECT_FALSE(SortedVectorSet({0, 1, 3}) >= SortedVectorSet({0, 2}));
}

TEST(SortedVectorSetTest, Print) {
  auto s = SortedVectorSet(VI({3, 1, 5}));
  stringstream ss;
  ss << s;
  EXPECT_EQ(ss.str(), "SortedVectorSet(1, 3, 5)");
}

TEST(SortedVectorSetTest, SetOperations) {
  auto s = SortedVectorSet(VI({2, 3, 4, 5, 6, 7}));
  auto t = SortedVectorSet(VI({1, 3, 5, 8}));
  EXPECT_EQ(ds::set::SortedVectorSet::intersect(s, t), SortedVectorSet(VI({3, 5})));
  EXPECT_EQ(ds::set::SortedVectorSet::Union(s, t), SortedVectorSet(VI({1, 2, 3, 4, 5, 6, 7, 8})));

  EXPECT_EQ(t & 3, t);
  EXPECT_EQ(t & 4, t);
  EXPECT_EQ(t | 3, t);
  EXPECT_EQ(t | 4, SortedVectorSet(VI({1, 3, 4, 5, 8})));
  EXPECT_EQ(t - 3, SortedVectorSet(VI({1, 5, 8})));
  EXPECT_EQ(t - 4, t);
  EXPECT_EQ(t ^ 3, SortedVectorSet(VI({1, 5, 8})));
  EXPECT_EQ(t ^ 4, SortedVectorSet(VI({1, 3, 4, 5, 8})));

  EXPECT_EQ(s & t, SortedVectorSet(VI({3, 5})));
  EXPECT_EQ(s | t, SortedVectorSet(VI({1, 2, 3, 4, 5, 6, 7, 8})));
  EXPECT_EQ(s - t, SortedVectorSet(VI({2, 4, 6, 7})));
  EXPECT_EQ(s ^ t, SortedVectorSet(VI({1, 2, 4, 6, 7, 8})));

  auto a = SortedVectorSet(VI({3}));
  auto b = SortedVectorSet(VI({2, 3, 4, 5, 6}));
  EXPECT_EQ(a | a, a);
  EXPECT_EQ(a & a, a);
  EXPECT_EQ(a ^ a, SortedVectorSet());
  EXPECT_EQ(a - a, SortedVectorSet());

  EXPECT_EQ(a | b, b | a);
  EXPECT_EQ(a ^ b, b ^ a);
  EXPECT_EQ(a & b, b & a);

  std::vector<SortedVectorSet> xs = {SortedVectorSet({1}), SortedVectorSet({2, 0})};
  EXPECT_EQ(xs[0] ^ xs[1], SortedVectorSet({0, 1, 2}));
  EXPECT_EQ(xs[1] ^ xs[0], SortedVectorSet({0, 1, 2}));
}

TEST(SortedVectorSetTest, RangeBasedFor) {
  auto xs = SortedVectorSet(VI({2, 5, 3, 2, 4, 9}));
  int sum = 0;
  for (auto x : xs) sum += x;
  EXPECT_EQ(sum, 2 + 5 + 3 + 4 + 9);
}

TEST(SortedVectorSetTest, At) {
  auto xs = SortedVectorSet(VI({2, 5, 3, 2, 4, 9}));
  EXPECT_EQ(xs.at(0), 2);
  EXPECT_EQ(xs.at(1), 3);
  EXPECT_EQ(xs.at(2), 4);
  EXPECT_EQ(xs.at(3), 5);
  EXPECT_EQ(xs.at(4), 9);
  EXPECT_THROW(xs.at(-1), std::invalid_argument);
  EXPECT_THROW(xs.at(5), std::invalid_argument);
}
