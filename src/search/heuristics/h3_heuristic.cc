#include "h3_heuristic.h"

#include "../plugins/plugin.h"
#include "../task_utils/task_properties.h"

#include <set>

namespace h3_heuristic {
H3Heuristic::H3Heuristic(const plugins::Options& opts)
    : Heuristic(opts)
    , index_offsets()
    , h1_table()
    , h2_table()
    , h3_table()
    , operators()
    , goal()
    , has_cond_effects(task_properties::has_conditional_effects(task_proxy))
{
    int num_values = 0;

    for (int var_id = 0; var_id < task->get_num_variables(); ++var_id) {
        index_offsets.emplace_back(num_values);
        num_values += task->get_variable_domain_size(var_id);
    }

    // Preallocate h1_table and h2_table to avoid dynamic allocations when computing heuristic values

    h1_table.resize(num_values);
    h2_table.resize(num_values);
    h3_table.resize(num_values);

    for (int i = 0; i < num_values; ++i) {
        h2_table[i].resize(num_values);
        h3_table[i].resize(num_values);

        for (int j = 0; j < num_values; ++j) {
            h3_table[i][j].resize(num_values);
        }
    }

    // Convert operators to the internal format

    for (const auto& op : task_proxy.get_operators()) {
        std::vector<int> precondition;

        for (const auto& cond_proxy : op.get_preconditions()) {
            const auto [var_id, val_id] = cond_proxy.get_pair();
            precondition.emplace_back(get_index(var_id, val_id));
        }

        std::sort(precondition.begin(), precondition.end());

        std::vector<int> add_effect;

        for (const auto& eff_proxy : op.get_effects()) {
            const auto [var_id, val_id] = eff_proxy.get_fact().get_pair();
            add_effect.emplace_back(get_index(var_id, val_id));
        }

        std::sort(add_effect.begin(), add_effect.end());

        std::set<int> delete_effect;

        for (const auto& eff_proxy : op.get_effects()) {
            const auto [var_id, val_id] = eff_proxy.get_fact().get_pair();

            for (int other_val = 0; other_val < task->get_variable_domain_size(var_id); ++other_val) {
                delete_effect.emplace(get_index(var_id, other_val));
            }
        }

        std::vector<int> delete_effect_complement;

        for (int var_id = 0; var_id < task->get_num_variables(); ++var_id) {
            for (int val_id = 0; val_id < task->get_variable_domain_size(var_id); ++val_id) {
                const auto index = get_index(var_id, val_id);

                if (delete_effect.count(index) == 0) {
                    delete_effect_complement.emplace_back(index);
                }
            }
        }

        std::sort(delete_effect_complement.begin(), delete_effect_complement.end());

        operators.emplace_back(precondition, add_effect, delete_effect_complement, op.get_cost());
    }

    // Convert the goal to the internal format

    for (const auto& pair_proxy : task_proxy.get_goals()) {
        const auto [var_id, val_id] = pair_proxy.get_pair();
        goal.emplace_back(get_index(var_id, val_id));
    }

    std::sort(goal.begin(), goal.end());
}

bool H3Heuristic::dead_ends_are_reliable() const
{
    return !task_properties::has_axioms(task_proxy) && !has_cond_effects;
}

int H3Heuristic::get_index(int var_id, int val_id)
{
    return index_offsets[var_id] + val_id;
}

int H3Heuristic::evaluate(const std::vector<int>& indices)
{
    int v = 0;

    for (std::size_t i = 0; i < indices.size(); ++i) {
        const auto p = indices[i];
        v = std::max(v, h1_table[p]);

        if (v == INTERNAL_DEAD_END) {
            return INTERNAL_DEAD_END;
        }

        for (std::size_t j = i + 1; j < indices.size(); ++j) {
            const auto q = indices[j];
            v = std::max(v, h2_table[p][q]);

            if (v == INTERNAL_DEAD_END) {
                return INTERNAL_DEAD_END;
            }

            for (std::size_t k = j + 1; k < indices.size(); ++k) {
                const auto r = indices[k];
                v = std::max(v, h3_table[p][q][r]);

                if (v == INTERNAL_DEAD_END) {
                    return INTERNAL_DEAD_END;
                }
            }
        }
    }

    return v;
}

int H3Heuristic::evaluate(const std::vector<int>& indices, int q)
{
    int v = h1_table[q];

    if (v == INTERNAL_DEAD_END) {
        return INTERNAL_DEAD_END;
    }

    for (std::size_t i = 0; i < indices.size(); ++i) {
        const auto p = indices[i];

        if (p < q) {
            v = std::max(v, h2_table[p][q]);
        } else if (p > q) {
            v = std::max(v, h2_table[q][p]);
        }

        if (v == INTERNAL_DEAD_END) {
            return INTERNAL_DEAD_END;
        }

        for (std::size_t j = i + 1; j < indices.size(); ++j) {
            const auto r = indices[j];

            if (r < q) {
                v = std::max(v, h3_table[p][r][q]);
            } else if ((p < q) && (q < r)) {
                v = std::max(v, h3_table[p][q][r]);
            } else if (q < p) {
                v = std::max(v, h3_table[q][p][r]);
            }

            if (v == INTERNAL_DEAD_END) {
                return INTERNAL_DEAD_END;
            }
        }
    }

    return v;
}

int H3Heuristic::evaluate(const std::vector<int>& indices, int q, int r)
{
    int v = std::max(h1_table[q], h1_table[r]);

    if (v == INTERNAL_DEAD_END) {
        return INTERNAL_DEAD_END;
    }

    if (q > r) {
        std::swap(q, r);
    }

    v = std::max(v, h2_table[q][r]);

    if (v == INTERNAL_DEAD_END) {
        return INTERNAL_DEAD_END;
    }

    for (const auto p : indices) {
        if (p < q) {
            v = std::max(v, h2_table[p][q]);
        } else if (q < p) {
            v = std::max(v, h2_table[q][p]);
        }

        if (v == INTERNAL_DEAD_END) {
            return INTERNAL_DEAD_END;
        }

        if (p < r) {
            v = std::max(v, h2_table[p][r]);
        } else if (r < p) {
            v = std::max(v, h2_table[r][p]);
        }

        if (v == INTERNAL_DEAD_END) {
            return INTERNAL_DEAD_END;
        }

        if (p < q) {
            v = std::max(v, h3_table[p][q][r]);
        }

        if ((q < p) && (p < r)) {
            v = std::max(v, h3_table[q][p][r]);
        }

        if (r < p) {
            v = std::max(v, h3_table[q][r][p]);
        }

        if (v == INTERNAL_DEAD_END) {
            return INTERNAL_DEAD_END;
        }
    }

    return v;
}

void H3Heuristic::update(std::size_t index, int value, bool& changed)
{
    if (h1_table[index] > value) {
        h1_table[index] = value;
        changed = true;
    }
}

void H3Heuristic::update(std::size_t first_index, std::size_t second_index, int value, bool& changed)
{
    if (first_index > second_index) {
        std::swap(first_index, second_index);
    }

    if (h2_table[first_index][second_index] > value) {
        h2_table[first_index][second_index] = value;
        changed = true;
    }
}

void H3Heuristic::update(std::size_t first_index, std::size_t second_index, std::size_t third_index, int value, bool& changed)
{
    if (first_index > second_index) {
        std::swap(first_index, second_index);
    }

    if (second_index > third_index) {
        std::swap(second_index, third_index);

        if (first_index > second_index) {
            std::swap(first_index, second_index);
        }
    }

    if (h3_table[first_index][second_index][third_index] > value) {
        h3_table[first_index][second_index][third_index] = value;
        changed = true;
    }
}

void H3Heuristic::fill_tables(const State& state)
{
    std::fill(h1_table.begin(), h1_table.end(), INTERNAL_DEAD_END);

    for (auto& h2_vector : h2_table) {
        std::fill(h2_vector.begin(), h2_vector.end(), INTERNAL_DEAD_END);
    }

    for (auto& h3_matrix : h3_table) {
        for (auto& h3_vector : h3_matrix) {
            std::fill(h3_vector.begin(), h3_vector.end(), INTERNAL_DEAD_END);
        }
    }

    const auto& variable_values = state.get_unpacked_values();
    const auto num_variables = static_cast<int>(variable_values.size());

    for (int first_var_id = 0; first_var_id < num_variables; ++first_var_id) {
        const auto first_val_id = variable_values[first_var_id];
        const auto first_idx = get_index(first_var_id, first_val_id);
        h1_table[first_idx] = 0;

        for (int second_var_id = first_var_id + 1; second_var_id < num_variables; ++second_var_id) {
            const auto second_val_id = variable_values[second_var_id];
            const auto second_idx = get_index(second_var_id, second_val_id);
            h2_table[first_idx][second_idx] = 0;

            for (int third_var_id = second_var_id + 1; third_var_id < num_variables; ++third_var_id) {
                const auto third_val_id = variable_values[third_var_id];
                const auto third_idx = get_index(third_var_id, third_val_id);
                h3_table[first_idx][second_idx][third_idx] = 0;
            }
        }
    }

    bool changed;

    do {
        changed = false;

        for (const auto& [precondition, add_effect, delete_effect_complement, cost] : operators) {
            const auto c1 = evaluate(precondition);

            if (c1 == INTERNAL_DEAD_END) {
                continue;
            }

            for (std::size_t i = 0; i < add_effect.size(); ++i) {
                const auto p = add_effect[i];
                update(p, c1 + cost, changed);

                for (std::size_t j = i + 1; j < add_effect.size(); ++j) {
                    const auto q = add_effect[j];
                    update(p, q, c1 + cost, changed);

                    for (std::size_t k = j + 1; k < add_effect.size(); ++k) {
                        const auto r = add_effect[k];
                        update(p, q, r, c1 + cost, changed);
                    }
                }

                for (std::size_t j = 0; j < delete_effect_complement.size(); ++j) {
                    const auto q = delete_effect_complement[j];
                    const auto c2 = std::max(c1, evaluate(precondition, q));

                    if (c2 == INTERNAL_DEAD_END) {
                        continue;
                    }

                    update(p, q, c2 + cost, changed);

                    for (std::size_t k = j + 1; k < delete_effect_complement.size(); ++k) {
                        const auto r = delete_effect_complement[k];
                        const auto c3 = std::max(c2, evaluate(precondition, q, r));

                        if (c3 == INTERNAL_DEAD_END) {
                            continue;
                        }

                        update(p, q, r, c3 + cost, changed);
                    }

                    for (std::size_t k = i + 1; k < add_effect.size(); ++k) {
                        const auto r = add_effect[k];
                        update(p, q, r, c2 + cost, changed);
                    }
                }
            }
        }
    } while (changed);
}

int H3Heuristic::compute_heuristic(const State& ancestor_state)
{
    const auto state = task_proxy.convert_ancestor_state(ancestor_state);
    fill_tables(state);
    const auto value = evaluate(goal);
    return (value == INTERNAL_DEAD_END) ? DEAD_END : value;
}

class H3HeuristicFeature : public plugins::TypedFeature<Evaluator, H3Heuristic> {
public:
    H3HeuristicFeature()
        : TypedFeature("h3")
    {
        document_title("h^3 heuristic");

        Heuristic::add_options_to_feature(*this);

        document_language_support("action costs", "supported");
        document_language_support("conditional effects", "ignored");
        document_language_support("axioms", "ignored");

        document_property(
            "admissible",
            "yes for tasks without conditional effects or axioms");
        document_property(
            "consistent",
            "yes for tasks without conditional effects or axioms");
        document_property(
            "safe",
            "yes for tasks without conditional effects or axioms");
        document_property("preferred operators", "no");
    }
};

static plugins::FeaturePlugin<H3HeuristicFeature> _plugin;
}
