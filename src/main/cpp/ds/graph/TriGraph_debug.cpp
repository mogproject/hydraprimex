#include "TriGraph.hpp"

namespace ds {
namespace graph {

//============================================================================
//    Debugging
//============================================================================
/**
 * @brief Assert that the property `mu_` is consistent with the current graph.
 */
void TriGraph::check_consistency() const {
  auto h = *this;
  h.compute_pairwise_properties();

  if (h.mu_ != mu_) {
    log_debug("Found inconsistency in mu: expected=%s, actual=%s", cstr(h.mu_), cstr(mu_));
    throw std::runtime_error("error in TriGraph#check_consistency()");
  }
}

}  // namespace graph
}  // namespace ds
