#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <iterator>
#include <tuple>

#include <ortools/constraint_solver/routing_parameters.h>

#include "IO.h"
#include "OptimizeOrderSize.h"
#include "OptimizePickupAndDeliveryDuration.h"
#include "SolveCVRPPDTW.h"

inline auto reportSolution(const std::string &Caption, const sandrokottos::Solution &Solution) noexcept {
  std::cerr << Caption << ":\t" << std::get<0>(Solution.getCost()) << "\t" << std::get<1>(Solution.getCost()) << "\t" << std::get<2>(Solution.getCost()) << std::endl;
}

int main(int ArgCount, char **ArgValues) {
  const auto StartingTime = std::chrono::steady_clock::now();

  const auto Question = sandrokottos::readQuestion(std::cin);

  const auto Problem = sandrokottos::convertToProblem(Question);

  const auto Solution = [&] {
    const auto Solution1 = [&] {
      const auto TimeLimit = StartingTime + std::chrono::milliseconds{15'000};

      auto Future1 = std::async(std::launch::async, [&] {
        const auto RoutingSearchParameters = [] {
          auto Result = operations_research::DefaultRoutingSearchParameters();

          Result.set_first_solution_strategy(operations_research::FirstSolutionStrategy::AUTOMATIC);
          Result.set_local_search_metaheuristic(operations_research::LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
          Result.set_use_full_propagation(false);

          return Result;
        }();

        const auto Solution = sandrokottos::SolveCVRPPDTW{Problem, RoutingSearchParameters}(TimeLimit);
        reportSolution("1-1", Solution);

        return Solution;
      });

      auto Future2 = std::async(std::launch::async, [&] {
        const auto RoutingSearchParameters = [] {
          auto Result = operations_research::DefaultRoutingSearchParameters();

          Result.set_first_solution_strategy(operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC);
          Result.set_local_search_metaheuristic(operations_research::LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
          Result.set_use_full_propagation(true);

          return Result;
        }();

        const auto Solution = sandrokottos::SolveCVRPPDTW{Problem, RoutingSearchParameters}(TimeLimit);
        reportSolution("1-2", Solution);

        return Solution;
      });

      const auto Solution1 = Future1.get();
      const auto Solution2 = Future2.get();

      return Solution1.getCost() < Solution2.getCost() ? Solution1 : Solution2;
    }();
    reportSolution("1", Solution1);

    if (std::accumulate(std::begin(Solution1.getRoutes()), std::end(Solution1.getRoutes()), 0, [](const auto &Acc, const auto &Route) { return Acc + static_cast<int>(std::size(Route)); }) == Problem.getOrderSize() * 2) {
      const auto Solution2 = sandrokottos::OptimizePickupAndDeliveryDuration{Problem}(Solution1, StartingTime + std::chrono::milliseconds(19'500));
      reportSolution("2", Solution2);

      return Solution2;
    } else {
      const auto Solution3 = sandrokottos::OptimizeOrderSize{Problem}(Solution1, StartingTime + std::chrono::milliseconds(19'500));
      reportSolution("3", Solution3);

      return Solution3;
    }
  }();

  sandrokottos::writeAnswer(std::cout, sandrokottos::convertToAnswer(Question, Problem, Solution));

  return 0;
}
