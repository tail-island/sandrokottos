#pragma once

#include <algorithm>
#include <chrono>
#include <iterator>
#include <limits>
#include <ranges>
#include <tuple>
#include <vector>

#include "Model.h"

namespace sandrokottos {

// 部分点をかき集めて、配送できた件数を最大化します。

class OptimizeOrderSize final {
  const sandrokottos::Problem &Problem;

  auto getNewRoute(const Route &Route, int Order, int PIndex, int DIndex) const noexcept {
    // auto Result = Route;

    // Result.emplace(std::begin(Result) + PIndex, Order * 2 + 0);
    // Result.emplace(std::begin(Result) + DIndex, Order * 2 + 1);

    // return Result;

    auto Result = sandrokottos::Route{};

    auto it = std::begin(Route);

    for (auto I = 0; I < PIndex; ++I, ++it) {
      Result.emplace_back(*it);
    }

    Result.emplace_back(Order * 2 + 0);

    for (auto I = PIndex + 1; I < DIndex; ++I, ++it) {
      Result.emplace_back(*it);
    }

    Result.emplace_back(Order * 2 + 1);

    for (; it != std::end(Route); ++it) {
      Result.emplace_back(*it);
    }

    return Result;
  }

  auto getNewTimetable(const Route &Route, int RIndex) const noexcept {
    auto Result = Timetable{};

    auto Minute = 0;
    auto LuggageSize = 0;

    for (const auto &I : std::views::iota(0, static_cast<int>(std::size(Route)))) {
      if (I > 0) {
        Minute += Problem.getDurationMatrix()[Route[I - 1]][Route[I]];
      }

      Result.emplace_back(Minute);

      if (Route[I] % 2 == 0) {
        if (++LuggageSize > Problem.getCapacities()[RIndex]) { // キャパシティーを超えて積み込むことはできません。
          return Timetable{};
        }
      } else {
        Minute = std::max(Minute, 30);

        if (Minute > 150 - 2) { // 13:00の2分前までに配送しなければなりません。
          return Timetable{};
        }

        LuggageSize--;
      }
    }

    return Result;
  }

  auto getCost(const Route &Route, const Timetable &Timetable) const noexcept {
    auto Score1 = 0;
    auto Score2 = 0;
    auto Score3 = 0;

    auto LuggageSize = 0;

    for (const auto &I : std::views::iota(0, static_cast<int>(std::size(Route)))) {
      if (I > 0) {
        Score2 += (Timetable[I] - Timetable[I - 1]) * LuggageSize;
        Score3 += Problem.getDistanceMatrix()[Route[I - 1]][Route[I]];
      }

      if (Route[I] % 2 == 0) {
        LuggageSize++;

        Score2 -= 2; // 次のループで積み込み時間＋移動時間が足されるので、事前に積み込み時間分を減らしておきます。
      } else {
        LuggageSize--;

        const auto &[Lower, Upper] = Problem.getTimeWindows()[Route[I] / 2];
        Score1 += Lower <= Timetable[I] && Timetable[I] <= Upper ? 100 : std::max(80 - std::max(Lower - Timetable[I], Timetable[I] - Upper), 20);
      }
    }

    return std::make_tuple(-Score1, Score2, Score3);
  }

public:
  OptimizeOrderSize(const sandrokottos::Problem &Problem) noexcept : Problem{Problem} {}

  auto operator()(const Solution &Solution, const std::chrono::steady_clock::time_point &TimeLimit) noexcept {
    auto Routes = Solution.getRoutes();
    auto Timetables = Solution.getTimetables();

    auto Orders = [&] {
      auto Result = std::vector<int>{};

      std::ranges::copy(
          std::views::iota(0, Problem.getOrderSize()),
          std::back_inserter(Result));

      for (const auto &Route : Routes) {
        for (const auto &Node : Route) {
          if (Node % 2 != 0) {
            continue;
          }

          std::erase(Result, Node / 2);
        }
      }

      return Result;
    }();

    while (!Orders.empty() && std::chrono::steady_clock::now() <= TimeLimit) {
      const auto [Order, I, Route, Timetable] = [&] {
        auto Result = std::make_tuple(-1, 0, sandrokottos::Route{}, sandrokottos::Timetable{});

        auto BestDelta = std::make_tuple(0, 0, 0);

        for (const auto &Order : Orders) {
          if (!(std::chrono::steady_clock::now() <= TimeLimit)) {
            return Result;
          }

          for (const auto &RIndex : std::views::iota(0, static_cast<int>(std::size(Routes)))) {
            const auto Cost = getCost(Routes[RIndex], Timetables[RIndex]);

            for (const auto &PIndex : std::views::iota(0, static_cast<int>(std::size(Routes[RIndex]) + 1))) {
              for (const auto &DIndex : std::views::iota(PIndex + 1, static_cast<int>(std::size(Routes[RIndex]) + 2))) {
                const auto NewRoute = getNewRoute(Routes[RIndex], Order, PIndex, DIndex);
                const auto NewTimetable = getNewTimetable(NewRoute, RIndex);

                if (NewTimetable.empty()) {
                  continue;
                }

                const auto NewCost = getCost(NewRoute, NewTimetable);
                const auto Delta = std::make_tuple(std::get<0>(NewCost) - std::get<0>(Cost), std::get<1>(NewCost) - std::get<1>(Cost), std::get<2>(NewCost) - std::get<2>(Cost));

                if (Delta < BestDelta) {
                  BestDelta = Delta;
                  Result = std::make_tuple(Order, RIndex, NewRoute, NewTimetable);
                }
              }
            }
          }
        }

        return Result;
      }();

      if (Order < 0) {
        break;
      }

      std::erase(Orders, Order);

      Routes[I] = Route;
      Timetables[I] = Timetable;
    }

    Timetables = [&] {
      auto Result = std::vector<Timetable>{};

      std::ranges::copy(
          Routes | std::views::transform([&](const auto &Route) {
            return CreateRelaxedTimetable{Problem}(Route);
          }),
          std::back_inserter(Result));

      return Result;
    }();

    return sandrokottos::Solution(Routes, Timetables, CalculateCost{Problem}(Routes, Timetables));
  }
};

} // namespace sandrokottos
