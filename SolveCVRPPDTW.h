#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <ranges>
#include <vector>

#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_enums.pb.h>
#include <ortools/constraint_solver/routing_index_manager.h>
#include <ortools/constraint_solver/routing_parameters.h>

#include "Model.h"

namespace sandrokottos {

// OR-Toolsを使用して、Constrained Vehicle Routing Problem with Pickup and Delivery with Time Windowsを解きます。

class SolveCVRPPDTW final {
  const sandrokottos::Problem &Problem;
  const operations_research::RoutingSearchParameters &RoutingSearchParameters;

public:
  explicit SolveCVRPPDTW(const sandrokottos::Problem &Problem, const operations_research::RoutingSearchParameters &RoutingSearchParameters) noexcept
      : Problem{Problem}, RoutingSearchParameters{RoutingSearchParameters} {}

  auto operator()(const std::chrono::steady_clock::time_point &TimeLimit) const noexcept {
    auto RoutingManager = operations_research::RoutingIndexManager{Problem.getOrderSize() * 2 + 1,
                                                                   Problem.getRobotSize(),
                                                                   operations_research::RoutingIndexManager::NodeIndex{Problem.getOrderSize() * 2}};

    auto RoutingModel = operations_research::RoutingModel{RoutingManager};

    // 総走行距離をコストに設定します。

    RoutingModel.SetArcCostEvaluatorOfAllVehicles(RoutingModel.RegisterTransitCallback([&](const auto &FromIndex, const auto &ToIndex) {
      const auto FromNode = RoutingManager.IndexToNode(FromIndex).value();
      const auto ToNode = RoutingManager.IndexToNode(ToIndex).value();

      if (FromNode == Problem.getOrderSize() * 2 || ToNode == Problem.getOrderSize() * 2) {
        return 0;
      }

      return Problem.getDistanceMatrix()[FromNode][ToNode];
    }));

    // ノードを訪問しない場合のペナルティを設定します。

    for (const auto &I : std::views::iota(0, Problem.getOrderSize() * 2)) {
      RoutingModel.AddDisjunction({RoutingManager.NodeToIndex(operations_research::RoutingIndexManager::NodeIndex{I})}, 300);
    }

    // 荷物の量がキャパシティを超えてはなりません。

    RoutingModel.AddDimensionWithVehicleCapacity(
        RoutingModel.RegisterUnaryTransitCallback([&](const auto &Index) {
          const auto Node = RoutingManager.IndexToNode(Index).value();

          if (Node == Problem.getOrderSize() * 2) {
            return 0;
          }

          return Node % 2 == 0 ? 1 : -1;
        }),
        0,
        std::vector<std::int64_t>(std::begin(Problem.getCapacities()), std::end(Problem.getCapacities())),
        true,
        "Capacity");

    // 注文単位で、積込みの後に配送をしなければなりません。

    for (const auto &I : std::views::iota(0, Problem.getOrderSize())) {
      RoutingModel.AddPickupAndDelivery(RoutingManager.NodeToIndex(operations_research::RoutingIndexManager::NodeIndex{I * 2 + 0}),
                                        RoutingManager.NodeToIndex(operations_research::RoutingIndexManager::NodeIndex{I * 2 + 1}));
    }

    // 13:00の2分前までに、配送しなければなりません。

    RoutingModel.AddDimension(
        RoutingModel.RegisterTransitCallback([&](const auto &FromIndex, const auto &ToIndex) {
          const auto FromNode = RoutingManager.IndexToNode(FromIndex).value();
          const auto ToNode = RoutingManager.IndexToNode(ToIndex).value();

          if (FromNode == Problem.getOrderSize() * 2 || ToNode == Problem.getOrderSize() * 2) {
            return 0;
          }

          return Problem.getDurationMatrix()[FromNode][ToNode];
        }),
        150 - 2,
        150 - 2,
        false,
        "Time");

    // 時刻はこの先の処理で必要となるので、変数に対比しておきます。

    const auto &TimeDimension = RoutingModel.GetDimensionOrDie("Time");

    // 希望配送時刻に配送しなければなりません。

    for (const auto &I : std::views::iota(0, Problem.getOrderSize())) {
      auto DeriveryTime = TimeDimension.CumulVar(RoutingManager.NodeToIndex(operations_research::RoutingIndexManager::NodeIndex{I * 2 + 1}));

      DeriveryTime->SetRange(std::max(std::get<0>(Problem.getTimeWindows()[I]), 30), std::min(std::get<1>(Problem.getTimeWindows()[I]), 150 - 2));
    }

    /* 時刻は制約プログラミングで設定するので、ここでは何もしません。
    // 積込みはできるだけ遅く、配送はできるだけ早くします。

    for (const auto &I : std::views::iota(0, Problem.getOrderSize())) {
      RoutingModel.AddVariableMaximizedByFinalizer(TimeDimension.CumulVar(RoutingManager.NodeToIndex(operations_research::RoutingIndexManager::NodeIndex{I * 2 + 0})));
      RoutingModel.AddVariableMinimizedByFinalizer(TimeDimension.CumulVar(RoutingManager.NodeToIndex(operations_research::RoutingIndexManager::NodeIndex{I * 2 + 1})));
    }
    */

    // 問題を解きます。

    const auto RoutingSolution = RoutingModel.SolveWithParameters([&] {
      auto Result = RoutingSearchParameters;

      const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(TimeLimit - std::chrono::steady_clock::now()).count();

      Result.mutable_time_limit()->set_seconds(static_cast<int>(duration / 1'000'000'000));
      Result.mutable_time_limit()->set_nanos(static_cast<int>(duration % 1'000'000'000));

      return Result;
    }());

    // ソリューションを作成してリターンします。

    return [&] {
      const auto Routes = [&] {
        auto Result = std::vector<Route>{};

        for (const auto &I : std::views::iota(0, Problem.getRobotSize())) {
          Result.emplace_back([&] {
            auto Result = Route{};

            auto Index = RoutingModel.Start(I);

            if (!RoutingModel.IsEnd(Index)) {
              for (Index = RoutingSolution->Value(RoutingModel.NextVar(Index)); !RoutingModel.IsEnd(Index); Index = RoutingSolution->Value(RoutingModel.NextVar(Index))) {
                Result.emplace_back(RoutingManager.IndexToNode(Index).value());
              }
            }

            return Result;
          }());
        }

        return Result;
      }();

      const auto Timetables = [&] {
        auto Result = std::vector<Timetable>{};

        std::ranges::copy(
            Routes | std::views::transform([&](const auto &Route) {
              return CreateStrictTimetable{Problem}(Route);
            }),
            std::back_inserter(Result));

        return Result;
      }();

      return Solution{Routes, Timetables, CalculateCost{Problem}(Routes, Timetables)};
    }();
  }
};

} // namespace sandrokottos
