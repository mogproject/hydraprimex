#include <gtest/gtest.h>

#include "ds/graph/TriGraph.hpp"
#include "gen/random.hpp"

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
        auto g = gen::erdos_renyi_graph(n, pr, rand);
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
        auto g = gen::erdos_renyi_graph(n, pr, rand);

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

TEST(TriGraphTest, UpdateCandidates) {
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

  auto cand = g.find_candidates(2);
  EXPECT_EQ(cand, VII({{0, 1}, {0, 3}, {0, 4}, {1, 3}, {1, 4}, {2, 6}, {3, 4}, {5, 6}}));

  g.contract(0, 1, &graph_log);
  cand = g.update_candidates(cand, graph_log, 2);
  sort(cand.begin(), cand.end());
  EXPECT_EQ(cand, VII({{0, 4}, {2, 4}, {2, 6}, {3, 4}, {3, 5}, {5, 6}}));

  g.contract(0, 2, &graph_log);
  cand = g.update_candidates(cand, graph_log, 2);
  sort(cand.begin(), cand.end());
  EXPECT_EQ(cand, VII({{0, 4}, {3, 4}, {3, 5}, {3, 6}, {4, 6}, {5, 6}}));

  TriGraph star(util::range_to_vec(4), {{{0, 1}, 0}, {{0, 2}, 0}, {{0, 3}, 0}});
  cand = star.find_candidates(0);
  EXPECT_EQ(cand, VII({{1, 2}, {1, 3}, {2, 3}}));

  star.contract(2, 1, &graph_log);
  cand = star.update_candidates(cand, graph_log, 0);
  EXPECT_EQ(cand, VII({{2, 3}}));

  star.contract(3, 2, &graph_log);
  cand = star.update_candidates(cand, graph_log, 0);
  EXPECT_EQ(cand, VII({{0, 3}}));

  TriGraph k2(util::range_to_vec(2), {{{0, 1}, 0}});
  cand = k2.find_candidates(0);
  EXPECT_EQ(cand, VII({{0, 1}}));
}

TEST(TriGraphTest, UpdateCandidatesWithFrozen) {
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

  auto cand = g.find_candidates(2, {5});
  EXPECT_EQ(cand, VII({{0, 1}, {0, 3}, {0, 4}, {1, 3}, {1, 4}, {2, 6}, {3, 4}}));

  g.contract(0, 1, &graph_log);
  cand = g.update_candidates(cand, graph_log, 2, {5});
  sort(cand.begin(), cand.end());
  EXPECT_EQ(cand, VII({{0, 4}, {2, 4}, {2, 6}, {3, 4}}));

  g.contract(0, 2, &graph_log);
  cand = g.update_candidates(cand, graph_log, 2, {5});
  sort(cand.begin(), cand.end());
  EXPECT_EQ(cand, VII({{0, 4}, {3, 4}, {3, 6}, {4, 6}}));
}

TEST(TriGraphTest, OuterRedPotential) {
  TriGraph g(util::range_to_vec(8), {
                                        {{0, 2}, 1},
                                        {{0, 3}, 0},
                                        {{1, 2}, 0},
                                        {{1, 3}, 1},
                                        {{2, 4}, 1},
                                        {{2, 5}, 1},
                                        {{3, 4}, 1},
                                        {{3, 5}, 1},
                                        {{6, 2}, 0},
                                        {{7, 3}, 0},
                                        {{6, 7}, 1},
                                    });
  EXPECT_EQ(g.outer_red_potential(0, 1), 0);
  EXPECT_EQ(g.outer_red_potential(6, 7), 4);
  EXPECT_EQ(g.outer_red_potential(6, 0), 4);
  EXPECT_EQ(g.outer_red_potential(6, 1), 0);
}

TEST(TriGraphTest, IsFreeContraction) {
  TriGraph g1(util::range_to_vec(2), {});
  TriGraph g2(util::range_to_vec(2), {{{0, 1}, 0}});
  TriGraph g3(util::range_to_vec(2), {{{0, 1}, 1}});

  EXPECT_TRUE(g1.is_free_contraction(0, 1));
  EXPECT_TRUE(g2.is_free_contraction(0, 1));
  EXPECT_TRUE(g3.is_free_contraction(0, 1));
}

TEST(TriGraphTest, SubGraph) {
  TriGraph g1({1, 3, 5, 7}, {
                                {{1, 3}, 0},
                                {{3, 5}, 1},
                                {{3, 7}, 1},
                                {{5, 7}, 0},
                            });
  EXPECT_EQ(g1.vertices(), VI({0, 1, 2, 3}));
  EXPECT_EQ(g1.edges(true), TriGraph::ColoredEdgeList({{{0, 1}, 0}, {{1, 2}, 1}, {{1, 3}, 1}, {{2, 3}, 0}}));

  auto s1 = g1.subgraph({1, 2, 3}, false);
  s1.check_consistency();
  EXPECT_EQ(s1.vertices(), VI({1, 2, 3}));
  EXPECT_EQ(s1.edges(true), TriGraph::ColoredEdgeList({{{1, 2}, 1}, {{1, 3}, 1}, {{2, 3}, 0}}));
  EXPECT_EQ(s1.get_label(0), 1);
  EXPECT_EQ(s1.get_label(1), 3);

  auto s2 = g1.subgraph({1, 2, 3}, true);
  s2.check_consistency();
  EXPECT_EQ(s2.vertices(), VI({0, 1, 2}));
  EXPECT_EQ(s2.edges(true), TriGraph::ColoredEdgeList({{{0, 1}, 1}, {{0, 2}, 1}, {{1, 2}, 0}}));
  EXPECT_EQ(s2.get_label(0), 3);
  EXPECT_EQ(s2.get_label(1), 5);

  TriGraph g2({0, 1, 2, 3}, {
                                {{0, 1}, 0},
                                {{2, 3}, 1},
                            });
  auto s3 = g2.subgraph({0, 1}, false);
  s3.check_consistency();
  EXPECT_EQ(s3.number_of_vertices(), 2);
  EXPECT_EQ(s3.number_of_edges(), 1);
  EXPECT_EQ(s3.vertices(), VI({0, 1}));
  EXPECT_EQ(s3.edges(true), TriGraph::ColoredEdgeList({{{0, 1}, 0}}));
}

TEST(TriGraphTest, IsConnected) {
  EXPECT_TRUE(TriGraph({}, {}).is_connected());
  EXPECT_TRUE(TriGraph({1}, {}).is_connected());
  EXPECT_TRUE(TriGraph({1, 2}, {{{1, 2}, 0}}).is_connected());
  EXPECT_TRUE(TriGraph({1, 2}, {{{1, 2}, 1}}).is_connected());
  EXPECT_FALSE(TriGraph({1, 2}, {}).is_connected());
}

TEST(TriGraphTest, ConnectedComponents) {
  EXPECT_EQ(TriGraph({}, {}).connected_components(), VVI());
  EXPECT_EQ(TriGraph({1}, {}).connected_components(), VVI({{0}}));
  EXPECT_EQ(TriGraph({1, 2}, {{{1, 2}, 0}}).connected_components(), VVI({{0, 1}}));
  EXPECT_EQ(TriGraph({1, 2}, {{{1, 2}, 1}}).connected_components(), VVI({{{0, 1}}}));
  EXPECT_EQ(TriGraph({1, 2}, {}).connected_components(), VVI({{0}, {1}}));

  TriGraph g1 = {{0, 1, 2, 3, 4},
                 {
                     {{0, 1}, 1},
                     {{1, 4}, 0},
                     {{3, 2}, 1},
                 }};
  EXPECT_EQ(g1.connected_components(), VVI({{0, 1, 4}, {2, 3}}));
}
