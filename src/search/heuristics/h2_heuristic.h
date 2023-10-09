#ifndef HEURISTICS_H2_HEURISTIC_H
#define HEURISTICS_H2_HEURISTIC_H

#include "../heuristic.h"

#include <limits>
#include <vector>

namespace plugins {
class Options;
}

namespace h2_heuristic {

class H2Heuristic : public Heuristic {
    static constexpr int32_t INTERNAL_DEAD_END = std::numeric_limits<int>::max();
    std::vector<int> index_offsets;
    std::vector<int> h1_table;
    std::vector<std::vector<int>> h2_table;
    std::vector<std::tuple<std::vector<int>, std::vector<int>, std::vector<int>, int>> operators;
    std::vector<int> goal;
    const bool has_cond_effects;

    int get_index(int var_id, int val_id);
    int evaluate(const std::vector<int>& indices);
    int evaluate(const std::vector<int>& indices, int index);
    void update(const std::size_t index, const int value, bool& changed);
    void update(const std::size_t first_index, const std::size_t second_index, const int value, bool& changed);
    void fill_tables(const State& state);

protected:
    virtual int compute_heuristic(const State& ancestor_state) override;

public:
    explicit H2Heuristic(const plugins::Options& opts);

    virtual bool dead_ends_are_reliable() const override;
};
}

#endif
