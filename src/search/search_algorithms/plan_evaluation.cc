#include "plan_evaluation.h"

#include "../evaluation_context.h"
#include "../evaluator.h"
#include "../task_utils/task_properties.h"

#include "../plugins/plugin.h"
#include "../utils/logging.h"
#include "../utils/system.h"

#include <fstream>
#include <unordered_map>

using namespace std;

namespace plan_evaluation {
static string strip_parentheses(const string &line) {
    size_t begin = line.find('(');
    size_t end = line.rfind(')');
    if (begin == string::npos || end == string::npos || end <= begin)
        return line;
    return line.substr(begin + 1, end - begin - 1);
}

PlanEvaluation::PlanEvaluation(
    const shared_ptr<AbstractTask> &task, const shared_ptr<Evaluator> &evaluator,
    const string &plan_file, const string &description,
    utils::Verbosity verbosity)
    // Bound 0: SearchAlgorithm::search() calls initialize() and never step().
    : SearchAlgorithm(
          task, NORMAL, 0, numeric_limits<double>::infinity(), description,
          verbosity),
      evaluator(evaluator),
      plan_file(plan_file) {
}

void PlanEvaluation::initialize() {
    unordered_map<string, int> operator_by_name;
    OperatorsProxy operators = task_proxy.get_operators();
    for (OperatorProxy op : operators)
        operator_by_name.emplace(op.get_name(), op.get_id());

    ifstream input(plan_file);
    if (!input) {
        cerr << "Cannot read plan file " << plan_file << endl;
        utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
    }
    vector<int> plan;
    string line;
    while (getline(input, line)) {
        if (line.empty() || line[0] == ';')
            continue;
        string name = strip_parentheses(line);
        auto it = operator_by_name.find(name);
        if (it == operator_by_name.end()) {
            cerr << "Plan step " << plan.size() << " is not an operator of the "
                 << "task: " << name << endl;
            utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
        }
        plan.push_back(it->second);
    }

    vector<int> remaining_cost(plan.size() + 1, 0);
    for (int i = static_cast<int>(plan.size()) - 1; i >= 0; --i)
        remaining_cost[i] = remaining_cost[i + 1] + operators[plan[i]].get_cost();

    State state = state_registry.get_initial_state();
    for (size_t i = 0; i <= plan.size(); ++i) {
        EvaluationContext eval_context(state, 0, false, &statistics);
        statistics.inc_evaluated_states();
        EvaluationResult result = eval_context.get_result(evaluator.get());
        if (i == 0)
            evaluator->report_value_for_initial_state(result);
        log << "Plan state " << i << ": h=";
        if (result.is_infinite())
            log << "infinity";
        else
            log << result.get_evaluator_value();
        log << " h*=" << remaining_cost[i] << endl;
        if (i == plan.size())
            break;
        OperatorProxy op = operators[plan[i]];
        if (!task_properties::is_applicable(op, state)) {
            cerr << "Plan step " << i << " is not applicable: " << op.get_name()
                 << endl;
            utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
        }
        state = state_registry.get_successor_state(state, op);
    }
}

SearchStatus PlanEvaluation::step() {
    return get_finished_search_status();
}

void PlanEvaluation::save_plan_if_necessary() {
}

bool PlanEvaluation::is_complete_within_bound() const {
    return true;
}

void PlanEvaluation::print_statistics() const {
    statistics.print_detailed_statistics();
}

class PlanEvaluationFeature
    : public plugins::TypedFeature<TaskIndependentSearchAlgorithm> {
public:
    PlanEvaluationFeature() : TypedFeature("eval_plan") {
        document_title("Plan evaluation");
        document_synopsis(
            "Evaluate a heuristic on every state along a plan, without search. "
            "Prints the heuristic value and the remaining plan cost of each "
            "state.");
        add_option<shared_ptr<TaskIndependentEvaluator>>(
            "eval", "evaluator for h-value");
        add_option<string>(
            "plan_file", "plan in Fast Downward's sas_plan format");
        add_option<string>(
            "description",
            "description used to identify search algorithm in logs",
            "\"eval_plan\"");
        utils::add_log_options_to_feature(*this);
    }

    virtual shared_ptr<TaskIndependentSearchAlgorithm> create_component(
        const plugins::Options &options) const override {
        return components::make_auto_task_independent_component<
            PlanEvaluation, SearchAlgorithm>(
            options.get<shared_ptr<TaskIndependentEvaluator>>("eval"),
            options.get<string>("plan_file"),
            options.get<string>("description"),
            utils::get_log_arguments_from_options(options));
    }
};

static plugins::FeaturePlugin<PlanEvaluationFeature> _plugin;
}
