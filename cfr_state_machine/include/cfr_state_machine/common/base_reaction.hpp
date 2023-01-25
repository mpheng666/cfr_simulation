#ifndef CFR_SM_BASE_REACTION_HPP_
#define CFR_SM_BASE_REACTION_HPP_

#include "cfr_state_machine/common/boost_sc.hpp"

namespace cfr_sm {

template <class... base_transitions>
struct transitions_list : mpl::list<base_transitions...> {
  template <class... derived_transitions>
  using merge = transitions_list<base_transitions..., derived_transitions...>;
};

} // namespace cfr_sm

#endif