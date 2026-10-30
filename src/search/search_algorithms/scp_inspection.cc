#include "scp_inspection.h"

#include "../cost_saturation/abstraction.h"
#include "../cost_saturation/abstraction_generator.h"
#include "../cost_saturation/cost_partitioning_heuristic.h"
#include "../cost_saturation/greedy_order_utils.h"
#include "../cost_saturation/order_generator.h"
#include "../cost_saturation/utils.h"
#include "../plugins/plugin.h"
#include "../task_utils/task_properties.h"
#include "../utils/logging.h"
#include "../utils/system.h"

#include <fstream>
#include <map>
#include <unordered_map>

using namespace std;
using cost_saturation::INF;

namespace scp_inspection {
static string strip_parentheses(const string &line) {
    size_t begin = line.find('(');
    size_t end = line.rfind(')');
    if (begin == string::npos || end == string::npos || end <= begin)
        return line;
    return line.substr(begin + 1, end - begin - 1);
}

static void exit_with_input_error(const string &message) {
    cerr << message << endl;
    utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
}

// A cost or distance as JSON: infinities are the strings "inf" and "-inf".
static string number(int value) {
    if (value == INF)
        return "\"inf\"";
    if (value == -INF)
        return "\"-inf\"";
    return to_string(value);
}

static string quoted(const string &text) {
    string result = "\"";
    for (char c : text) {
        if (c == '"' || c == '\\')
            result += '\\';
        result += c;
    }
    return result + "\"";
}

template<typename T, typename F>
static void write_list(ostream &out, const vector<T> &values, F write) {
    out << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i)
            out << ",";
        write(values[i]);
    }
    out << "]";
}

static void write_ints(ostream &out, const vector<int> &values) {
    write_list(out, values, [&out](int value) { out << number(value); });
}

// Operators whose cost is not zero, as {"operator id": cost}.
static void write_sparse_costs(ostream &out, const vector<int> &costs) {
    out << "{";
    bool first = true;
    for (size_t op = 0; op < costs.size(); ++op) {
        if (costs[op] == 0)
            continue;
        out << (first ? "" : ",") << "\"" << op << "\":" << number(costs[op]);
        first = false;
    }
    out << "}";
}

static int value_at(const vector<int> &h_values, int abstract_state_id) {
    return h_values[abstract_state_id];
}

ScpInspection::ScpInspection(
    const shared_ptr<AbstractTask> &task,
    const vector<shared_ptr<cost_saturation::AbstractionGenerator>>
        &abstraction_generators,
    cost_saturation::Saturator saturator,
    const shared_ptr<cost_saturation::OrderGenerator> &order_generator,
    const vector<int> &order, const string &plan_file, int target_step,
    const string &output_file, const string &description,
    utils::Verbosity verbosity)
    // Bound 0: SearchAlgorithm::search() calls initialize() and never step().
    : SearchAlgorithm(
          task, NORMAL, 0, numeric_limits<double>::infinity(), description,
          verbosity),
      abstraction_generators(abstraction_generators),
      saturator(saturator),
      order_generator(order_generator),
      order(order),
      plan_file(plan_file),
      target_step(target_step),
      output_file(output_file) {
}

void ScpInspection::initialize() {
    OperatorsProxy operators = task_proxy.get_operators();
    vector<int> costs = task_properties::get_operator_costs(task_proxy);

    // The inspected states: the initial state and every state along the plan.
    vector<int> plan;
    if (!plan_file.empty()) {
        unordered_map<string, int> operator_by_name;
        for (OperatorProxy op : operators)
            operator_by_name.emplace(op.get_name(), op.get_id());
        ifstream input(plan_file);
        if (!input)
            exit_with_input_error("Cannot read plan file " + plan_file);
        string line;
        while (getline(input, line)) {
            if (line.empty() || line[0] == ';')
                continue;
            auto it = operator_by_name.find(strip_parentheses(line));
            if (it == operator_by_name.end())
                exit_with_input_error(
                    "Plan step " + to_string(plan.size()) +
                    " is not an operator of the task: " + line);
            plan.push_back(it->second);
        }
    }
    vector<State> states = {state_registry.get_initial_state()};
    for (size_t i = 0; i < plan.size(); ++i) {
        OperatorProxy op = operators[plan[i]];
        if (!task_properties::is_applicable(op, states.back()))
            exit_with_input_error(
                "Plan step " + to_string(i) +
                " is not applicable: " + op.get_name());
        states.push_back(state_registry.get_successor_state(states.back(), op));
    }
    for (State &state : states)
        state.unpack();
    vector<int> remaining_plan_cost(plan.size() + 1, 0);
    for (int i = static_cast<int>(plan.size()) - 1; i >= 0; --i)
        remaining_plan_cost[i] =
            remaining_plan_cost[i + 1] + operators[plan[i]].get_cost();
    int num_states = states.size();
    if (target_step < 0 || target_step >= num_states)
        exit_with_input_error(
            "target_step " + to_string(target_step) + " is not a state of the " +
            to_string(plan.size()) + "-step plan");

    cost_saturation::Abstractions abstractions =
        cost_saturation::generate_abstractions(task, abstraction_generators);
    int num_abstractions = abstractions.size();
    vector<vector<int>> abstract_state_ids;
    for (const State &state : states)
        abstract_state_ids.push_back(
            cost_saturation::get_abstract_state_ids(abstractions, state));
    const vector<int> &target_ids = abstract_state_ids[target_step];

    // Goal distances and saturated costs under the original costs, which also
    // define the greedy order's stolen costs.
    vector<vector<int>> h_full;
    vector<vector<int>> saturated_full;
    for (const auto &abstraction : abstractions) {
        h_full.push_back(abstraction->compute_goal_distances(costs));
        saturated_full.push_back(
            abstraction->compute_saturated_costs(h_full.back()));
    }
    vector<int> surplus_costs =
        cost_saturation::compute_all_surplus_costs(costs, saturated_full);

    vector<int> used_order = order;
    if (used_order.empty()) {
        order_generator->initialize(abstractions, costs);
        used_order = order_generator->compute_order_for_state(target_ids, false);
    }
    vector<bool> in_order(num_abstractions, false);
    for (int abs : used_order) {
        if (abs < 0 || abs >= num_abstractions || in_order[abs])
            exit_with_input_error(
                "order must list distinct abstractions 0.." +
                to_string(num_abstractions - 1));
        in_order[abs] = true;
    }

    ofstream out(output_file);
    if (!out)
        exit_with_input_error("Cannot write " + output_file);
    out << "{\"target_step\":" << target_step << ",\"saturator\":"
        << (saturator == cost_saturation::Saturator::ALL     ? "\"all\""
            : saturator == cost_saturation::Saturator::PERIM ? "\"perim\""
                                                             : "\"perimstar\"")
        << ",\n\"operators\":[";
    for (OperatorProxy op : operators)
        out << (op.get_id() ? "," : "") << "{\"name\":" << quoted(op.get_name())
            << ",\"cost\":" << op.get_cost() << "}";
    out << "],\n\"states\":[";
    for (int k = 0; k < num_states; ++k) {
        out << (k ? ",\n" : "") << "{\"step\":" << k
            << ",\"remaining_plan_cost\":";
        if (plan.empty())
            out << "null";
        else
            out << remaining_plan_cost[k];
        out << ",\"values\":";
        write_ints(out, states[k].get_unpacked_values());
        out << "}";
    }
    out << "],\n\"abstractions\":[";
    for (int abs = 0; abs < num_abstractions; ++abs) {
        const auto &abstraction = *abstractions[abs];
        map<int, int> histogram;
        for (int h : h_full[abs])
            ++histogram[h];
        out << (abs ? ",\n" : "") << "{\"states\":"
            << abstraction.get_num_states()
            << ",\"goal_states\":" << abstraction.get_goal_states().size()
            << ",\"stolen_costs\":"
            << cost_saturation::compute_costs_stolen_by_heuristic(
                   saturated_full[abs], surplus_costs)
            << ",\"abstract_states\":";
        vector<int> ids, distances;
        for (int k = 0; k < num_states; ++k) {
            ids.push_back(abstract_state_ids[k][abs]);
            distances.push_back(
                value_at(h_full[abs], abstract_state_ids[k][abs]));
        }
        write_ints(out, ids);
        out << ",\"h_original_costs\":";
        write_ints(out, distances);
        out << ",\"h_original_costs_histogram\":{";
        bool first = true;
        for (const auto &[h, count] : histogram) {
            out << (first ? "" : ",")
                << (h == INF ? string("\"inf\"") : "\"" + to_string(h) + "\"")
                << ":" << count;
            first = false;
        }
        out << "}}";
    }
    out << "],\n\"order\":";
    write_ints(out, used_order);

    // The partitioning, pass by pass: perimstar is perim followed by all on
    // the costs perim leaves.
    vector<cost_saturation::Saturator> passes;
    if (saturator == cost_saturation::Saturator::PERIMSTAR)
        passes = {
            cost_saturation::Saturator::PERIM, cost_saturation::Saturator::ALL};
    else
        passes = {saturator};
    vector<int> remaining = costs;
    vector<int> h_sum(num_states, 0);
    out << ",\n\"passes\":[";
    for (size_t pass = 0; pass < passes.size(); ++pass) {
        bool perim = passes[pass] == cost_saturation::Saturator::PERIM;
        out << (pass ? ",\n" : "") << "{\"saturator\":"
            << (perim ? "\"perim\"" : "\"all\"") << ",\"positions\":[";
        for (size_t position = 0; position < used_order.size(); ++position) {
            int abs = used_order[position];
            const auto &abstraction = *abstractions[abs];
            vector<int> h_values = abstraction.compute_goal_distances(remaining);
            if (perim) {
                int h_cap = h_values[target_ids[abs]];
                for (int &h : h_values) {
                    if (h != INF)
                        h = min(h, h_cap);
                }
            }
            vector<int> saturated = abstraction.compute_saturated_costs(h_values);
            vector<int> h_at_states;
            for (int k = 0; k < num_states; ++k) {
                int h = value_at(h_values, abstract_state_ids[k][abs]);
                h_at_states.push_back(h);
                h_sum[k] = cost_saturation::left_addition(h_sum[k], h);
            }
            out << (position ? ",\n" : "\n") << "{\"abstraction\":" << abs
                << ",\"h\":";
            write_ints(out, h_at_states);
            out << ",\"saturated_costs\":";
            write_sparse_costs(out, saturated);
            out << "}";
            cost_saturation::reduce_costs(remaining, saturated);
        }
        out << "]}";
    }
    out << "],\n\"h\":";
    write_ints(out, h_sum);

    // Costs no abstraction claimed, for operators some abstraction can use.
    vector<int> leftover(costs.size(), 0);
    for (size_t op = 0; op < costs.size(); ++op) {
        for (const auto &abstraction : abstractions) {
            if (abstraction->operator_is_active(op)) {
                leftover[op] = remaining[op];
                break;
            }
        }
    }
    out << ",\n\"leftover_costs\":";
    write_sparse_costs(out, leftover);

    // Scorpion's own computation of the same partitioning, as a check.
    vector<int> check_costs = costs;
    cost_saturation::CostPartitioningHeuristic cp =
        cost_saturation::get_cp_function(saturator)(
            abstractions, used_order, check_costs, target_ids);
    vector<int> h_check;
    for (int k = 0; k < num_states; ++k)
        h_check.push_back(cp.compute_heuristic(abstract_state_ids[k]));
    out << ",\n\"h_check\":";
    write_ints(out, h_check);
    out << "}\n";
    log << "Wrote SCP inspection of " << num_states << " states and "
        << num_abstractions << " abstractions to " << output_file << endl;
}

SearchStatus ScpInspection::step() {
    return get_finished_search_status();
}

void ScpInspection::save_plan_if_necessary() {
}

bool ScpInspection::is_complete_within_bound() const {
    return true;
}

void ScpInspection::print_statistics() const {
    statistics.print_detailed_statistics();
}

class ScpInspectionFeature
    : public plugins::TypedFeature<TaskIndependentSearchAlgorithm> {
public:
    ScpInspectionFeature() : TypedFeature("inspect_scp") {
        document_title("Saturated cost partitioning inspection");
        document_synopsis(
            "Write every quantity in the definition of a saturated cost "
            "partitioning (goal distances, saturated and remaining costs per "
            "abstraction in the order, and their sum) for the states along a "
            "plan to a JSON file, without search.");
        add_list_option<shared_ptr<
            cost_saturation::TaskIndependentAbstractionGenerator>>(
            "abstractions", "abstraction generators");
        cost_saturation::add_saturator_option(*this);
        add_option<shared_ptr<cost_saturation::TaskIndependentOrderGenerator>>(
            "orders", "order generator, used when no order is given",
            "greedy_orders()");
        add_list_option<int>(
            "order",
            "abstractions in the order to saturate them, by index; empty for "
            "the order generator's order for the target state",
            "[]");
        add_option<string>(
            "plan_file", "plan in Fast Downward's sas_plan format; empty for "
            "the initial state only", "\"\"");
        add_option<int>(
            "target_step",
            "the plan state for which the cost partitioning is computed",
            "0", plugins::Bounds("0", "infinity"));
        add_option<string>(
            "output_file", "JSON output file", "\"scp_inspection.json\"");
        add_option<string>(
            "description",
            "description used to identify search algorithm in logs",
            "\"inspect_scp\"");
        utils::add_log_options_to_feature(*this);
    }

    virtual shared_ptr<TaskIndependentSearchAlgorithm> create_component(
        const plugins::Options &options) const override {
        return components::make_auto_task_independent_component<
            ScpInspection, SearchAlgorithm>(
            cost_saturation::get_abstraction_generator_list_from_options(
                options),
            options.get<cost_saturation::Saturator>("saturator"),
            options.get<shared_ptr<
                cost_saturation::TaskIndependentOrderGenerator>>("orders"),
            options.get_list<int>("order"), options.get<string>("plan_file"),
            options.get<int>("target_step"),
            options.get<string>("output_file"),
            options.get<string>("description"),
            utils::get_log_arguments_from_options(options));
    }
};

static plugins::FeaturePlugin<ScpInspectionFeature> _plugin;
}
