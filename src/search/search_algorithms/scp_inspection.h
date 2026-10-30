#ifndef SEARCH_ALGORITHMS_SCP_INSPECTION_H
#define SEARCH_ALGORITHMS_SCP_INSPECTION_H

#include "../search_algorithm.h"

#include "../cost_saturation/saturated_cost_partitioning_heuristic.h"

#include <memory>
#include <string>
#include <vector>

namespace cost_saturation {
class AbstractionGenerator;
class OrderGenerator;
}

namespace scp_inspection {
/*
  Write every quantity in the definition of a saturated cost partitioning to a
  JSON file, without search.

  For abstractions a_1..a_n in an order and operator costs c, saturated cost
  partitioning starts with the remaining costs r_1 = c. The i-th abstraction
  in the order gets the goal distances h_i under r_i and keeps its saturated
  costs s_i, the least costs preserving h_i; then r_{i+1} = r_i - s_i. The
  heuristic value of a state s is the sum of h_i(a_i(s)).

  The inspected states are the initial state and, with a plan file, every state
  along the plan. The cost partitioning is computed for the state at
  `target_step` and evaluated on all of them, as the online heuristic does.
  Per abstraction the file holds its size, the abstract state and the goal
  distance under the original costs of every inspected state, and the
  greedy order's stolen costs; per position of the order the goal distances
  under the remaining costs and the saturated cost of every operator whose
  saturated cost is not zero; then the costs left over and the sum.
  Like a search with bound 0, the planner exits with
  SEARCH_UNSOLVABLE_WITHIN_BOUND.
*/
class ScpInspection : public SearchAlgorithm {
    const std::vector<std::shared_ptr<cost_saturation::AbstractionGenerator>>
        abstraction_generators;
    const cost_saturation::Saturator saturator;
    const std::shared_ptr<cost_saturation::OrderGenerator> order_generator;
    const std::vector<int> order;
    const std::string plan_file;
    const int target_step;
    const std::string output_file;

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

public:
    ScpInspection(
        const std::shared_ptr<AbstractTask> &task,
        const std::vector<std::shared_ptr<cost_saturation::AbstractionGenerator>>
            &abstraction_generators,
        cost_saturation::Saturator saturator,
        const std::shared_ptr<cost_saturation::OrderGenerator> &order_generator,
        const std::vector<int> &order, const std::string &plan_file,
        int target_step, const std::string &output_file,
        const std::string &description, utils::Verbosity verbosity);

    virtual void save_plan_if_necessary() override;
    virtual bool is_complete_within_bound() const override;
    virtual void print_statistics() const override;
};
}

#endif
