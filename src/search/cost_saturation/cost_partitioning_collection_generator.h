#ifndef COST_SATURATION_COST_PARTITIONING_COLLECTION_GENERATOR_H
#define COST_SATURATION_COST_PARTITIONING_COLLECTION_GENERATOR_H

#include "types.h"

#include <memory>
#include <vector>

class RandomWalkSampler;
class State;
class SuccessorGenerator;
class TaskProxy;

namespace options {
class Options;
class OptionParser;
}

namespace utils {
class RandomNumberGenerator;
}

namespace cost_saturation {
class Abstraction;
class CostPartitioningGenerator;

using CPFunction = std::function<CostPartitioning(
                                     const Abstractions &, const std::vector<int> &, const std::vector<int> &)>;

class CostPartitioningCollectionGenerator {
    const std::shared_ptr<CostPartitioningGenerator> cp_generator;
    const int max_orders;
    const double max_time;
    const bool diversify;
    const std::shared_ptr<utils::RandomNumberGenerator> rng;

    std::unique_ptr<RandomWalkSampler> sampler;
    CostPartitioning scp_for_sampling;
    int init_h;

    void initialize(
        const TaskProxy &task_proxy,
        const std::vector<std::unique_ptr<Abstraction>> &abstractions,
        const std::vector<int> &costs);

public:
    explicit CostPartitioningCollectionGenerator(const options::Options &opts);
    ~CostPartitioningCollectionGenerator();

    CostPartitionings get_cost_partitionings(
        const TaskProxy &task_proxy,
        const std::vector<std::unique_ptr<Abstraction>> &abstractions,
        const std::vector<int> &costs,
        CPFunction cp_function);
};
}

#endif
