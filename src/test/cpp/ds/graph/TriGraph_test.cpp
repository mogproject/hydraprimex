#include <gtest/gtest.h>

#include "ds/graph/TriGraph.hpp"
#include "generator/random.hpp"

using namespace std;
using namespace ds::graph;

typedef vector<int> VI;
typedef vector<vector<int>> VVI;
typedef vector<pair<int, int>> VII;

TEST(TriGraphTest, WeakRedPotential) {
  TriGraph g = {{0, 1, 2, 3, 4, 5, 6, 7},
                {
                    {{0, 1}, 0},
                    {{0, 7}, 1},
                    {{1, 2}, 0},
                    {{1, 3}, 0},
                    {{1, 5}, 0},
                    {{1, 6}, 0},
                    {{1, 7}, 1},
                    {{2, 4}, 1},
                    {{3, 4}, 0},
                    {{3, 5}, 0},
                    {{4, 5}, 0},
                    {{4, 6}, 1},
                    {{6, 7}, 1},
                }};
  vector<vector<int>> actual(8, vector<int>(8));
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      if (i != j) actual[i][j] = g.weak_red_potential(i, j);
    }
  }
  EXPECT_EQ(actual, VVI({{0, 5, 2, 3, 6, 3, 2, 2},
                         {5, 0, 6, 5, 4, 5, 6, 5},
                         {2, 6, 0, 2, 4, 2, 2, 4},
                         {3, 5, 2, 0, 3, 0, 3, 5},
                         {6, 4, 4, 3, 0, 3, 5, 6},
                         {3, 5, 2, 0, 3, 0, 3, 5},
                         {2, 6, 2, 3, 5, 3, 0, 3},
                         {2, 5, 4, 5, 6, 5, 3, 0}}));

  TriGraph g2 = {{0, 1, 2, 3},
                 {
                     {{0, 1}, 0},
                     {{0, 2}, 0},
                     {{0, 3}, 0},
                     {{1, 2}, 0},
                     {{1, 3}, 0},
                     {{2, 3}, 0},
                 }};
  vector<vector<int>> actual2(4, vector<int>(4));
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (i != j) actual2[i][j] = g2.weak_red_potential(i, j);
    }
  }
  EXPECT_EQ(actual2, VVI({
                         {0, 0, 0, 0},
                         {0, 0, 0, 0},
                         {0, 0, 0, 0},
                         {0, 0, 0, 0},
                     }));
}

std::vector<std::pair<int, int>> random_contraction_sequence(int n, util::Random &rand) {
  std::vector<int> mapping;
  for (int i = 0; i < n; ++i) mapping.push_back(i);
  rand.shuffle(mapping);

  std::vector<std::pair<int, int>> ret;
  for (int i = 0; i < n - 1; ++i) {
    auto p = rand.randint(i + 1, n - 1);
    ret.push_back({mapping[p], mapping[i]});
  }

  return ret;
}

TEST(TriGraphTest, BlackComplement) {
  auto g = TriGraph({7, 2, 3, 5}, {{{7, 2}, 0}, {{7, 3}, 1}, {{2, 3}, 1}, {{3, 5}, 0}});
  auto h = g;
  g.black_complement();

  EXPECT_EQ(g.vertices(), h.vertices());
  EXPECT_EQ(g.edges(true), TriGraph::ColoredEdgeList({
                               {{0, 2}, 1},
                               {{0, 3}, 0},
                               {{1, 2}, 1},
                               {{1, 3}, 0},
                           }));
}

TEST(TriGraphTest, Contract) {
  util::Random rand(12345);

  for (auto pr : vector<double>({0.2, 0.5})) {
    for (auto n : vector<int>({5, 10, 30})) {
      for (int t = 0; t < 20; ++t) {
        // create a random graph
        auto g = generator::erdos_renyi_graph(n, pr, rand);
        // log_debug("n=%d, edges=%s", n, cstr(g.edges()));

        // create a random contraction sequence
        auto seq = random_contraction_sequence(n, rand);

        // simulate contractions
        for (auto &p : seq) {
          g.contract(p.first, p.second);
          // log_debug("contraction: j=%d, i=%d", p.first, p.second);
          g.check_consistency();
        }
      }
    }
  }
}

TEST(TriGraphTest, UndoContraction) {
  util::Random rand(1234567);

  for (auto pr : vector<double>({0.2, 0.5})) {
    for (auto n : vector<int>({5, 10, 30})) {
      for (int t = 0; t < 20; ++t) {
        // create a random graph
        auto g = generator::erdos_renyi_graph(n, pr, rand);

        // create a random contraction sequence
        auto seq = random_contraction_sequence(n, rand);

        vector<TriGraph> graphs = {g};
        vector<GraphLog> history;

        for (auto &p : seq) {
          GraphLog graph_log;
          graphs.push_back(graphs.back());
          graphs.back().contract(p.first, p.second, &graph_log);
          // graphs.back().check_consistency();
          history.push_back(graph_log);
        }

        // undo
        for (int i = n - 2; i >= 0; --i) {
          graphs[i + 1].undo(history[i]);
          graphs[i + 1].check_consistency();
          EXPECT_EQ(graphs[i], graphs[i + 1]);
        }
      }
    }
  }
}

TEST(TriGraphTest, ContractPotentialDecreased) {
  util::Random rand(12345);

  TriGraph g(util::range_to_vec(7), {
                                        {{0, 1}, 0},
                                        {{0, 2}, 0},
                                        {{0, 3}, 0},
                                        {{1, 3}, 0},
                                        {{1, 4}, 0},
                                        {{2, 6}, 1},
                                        {{3, 5}, 1},
                                        {{5, 6}, 0},
                                    });
  GraphLog graph_log;

  g.contract(0, 1, &graph_log);
  sort(graph_log.potential_decreased.begin(), graph_log.potential_decreased.end());
  EXPECT_EQ(graph_log.potential_decreased, VII({{2, 4}, {3, 5}, {3, 6}}));

  g.contract(0, 2, &graph_log);
  sort(graph_log.potential_decreased.begin(), graph_log.potential_decreased.end());
  EXPECT_EQ(graph_log.potential_decreased, VII({{0, 5}, {0, 6}, {3, 6}, {4, 6}}));
}
