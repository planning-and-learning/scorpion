#ifndef SEARCH_ALGORITHMS_PLAN_EVALUATION_H
#define SEARCH_ALGORITHMS_PLAN_EVALUATION_H

#include "../search_algorithm.h"

#include <memory>
#include <string>

class Evaluator;

namespace plan_evaluation {
/*
  Build the heuristic once, then evaluate it on every state along a given plan.
  For each state s_i reached by the first i plan steps, print
  "Plan state i: h=<h(s_i)> h*=<cost of the remaining steps>". The plan should
  be optimal, so that the remaining cost is h*(s_i). No search is performed:
  like a search with bound 0, the planner exits with
  SEARCH_UNSOLVABLE_WITHIN_BOUND.
*/
class PlanEvaluation : public SearchAlgorithm {
    std::shared_ptr<Evaluator> evaluator;
    const std::string plan_file;

protected:
    virtual void initialize() override;
    virtual SearchStatus step() override;

public:
    PlanEvaluation(
        const std::shared_ptr<AbstractTask> &task,
        const std::shared_ptr<Evaluator> &evaluator,
        const std::string &plan_file, const std::string &description,
        utils::Verbosity verbosity);

    virtual void save_plan_if_necessary() override;
    virtual bool is_complete_within_bound() const override;
    virtual void print_statistics() const override;
};
}

#endif
