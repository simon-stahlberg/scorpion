#include "scp_generator_greedy.h"

#include "abstraction.h"
#include "utils.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../task_proxy.h"

#include "../utils/logging.h"

#include <algorithm>
#include <cassert>

using namespace std;

namespace cost_saturation {
SCPGeneratorGreedy::SCPGeneratorGreedy(const Options &opts)
    : SCPGenerator(opts),
      increasing_ratios(opts.get<bool>("increasing_ratios")) {
    if (max_orders != 1) {
        cerr << "SCPGeneratorGreedy currently needs max_orders = 1" << endl;
        utils::exit_with(utils::ExitCode::INPUT_ERROR);
    }
    if (diversify) {
        cerr << "SCPGeneratorGreedy currently needs diversify = false" << endl;
        utils::exit_with(utils::ExitCode::INPUT_ERROR);
    }
}

static int compute_sum(const vector<int> &vec) {
    int sum = 0;
    for (int val : vec) {
        assert(val != INF);
        if (val == -INF) {
            return -INF;
        } else {
            sum += val;
        }
    }
    return sum;
}

void SCPGeneratorGreedy::initialize(
    const TaskProxy &,
    const vector<unique_ptr<Abstraction>> &,
    const vector<StateMap> &,
    const vector<int> &) {
    // TODO: Initialize.
}

CostPartitioning SCPGeneratorGreedy::get_next_cost_partitioning(
    const TaskProxy &task_proxy,
    const vector<unique_ptr<Abstraction>> &abstractions,
    const vector<StateMap> &state_maps,
    const vector<int> &costs) {
    State initial_state = task_proxy.get_initial_state();
    assert(abstractions.size() == state_maps.size());
    int num_abstractions = abstractions.size();

    set<int> unused_abstractions;
    for (int i = 0; i < num_abstractions; ++i) {
        unused_abstractions.insert(i);
    }
    vector<int> order;

    while (!unused_abstractions.empty()) {
        double max_h_per_costs = -numeric_limits<double>::infinity();
        int min_costs = numeric_limits<int>::max();
        int best_pos = -1;
        for (int i : unused_abstractions) {
            const Abstraction &abstraction = *abstractions[i];
            const StateMap &state_map = state_maps[i];
            auto pair = abstraction.compute_goal_distances_and_saturated_costs(costs);
            vector<int> &h_values = pair.first;
            vector<int> &saturated_costs = pair.second;
            int initial_state_id = state_map(initial_state);
            double init_h = h_values[initial_state_id];
            int used_costs = compute_sum(saturated_costs);
            double h_per_costs = static_cast<double>(init_h) / max(1, used_costs);
            if (h_per_costs > max_h_per_costs ||
                (h_per_costs == max_h_per_costs && used_costs < min_costs)) {
                best_pos = i;
                max_h_per_costs = h_per_costs;
                min_costs = used_costs;
            }
            cout << i << ": " << " " << init_h << " / " << used_costs
                 << " = " << h_per_costs << endl;
        }
        assert(best_pos != -1);
        order.push_back(best_pos);
        unused_abstractions.erase(best_pos);
        cout << "Use: " << best_pos << endl;
    }
    assert(order.size() == abstractions.size());
    if (increasing_ratios) {
        reverse(order.begin(), order.end());
    }
    cout << "Order: " << order << endl;
    return compute_saturated_cost_partitioning(abstractions, order, costs);
}


static shared_ptr<SCPGenerator> _parse_greedy(OptionParser &parser) {
    parser.add_option<bool>(
        "increasing_ratios",
        "sort by increasing h/costs ratios",
        "false");
    add_common_scp_generator_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return nullptr;
    else
        return make_shared<SCPGeneratorGreedy>(opts);
}

static PluginShared<SCPGenerator> _plugin_greedy(
    "greedy", _parse_greedy);
}
