#pragma once

#include <algorithm>
#include <iterator>
#include <random>
#include <ranges>
#include <vector>

#include <boost/container/small_vector.hpp>

#include "Model.h"

namespace sandrokottos {

// 総走行距離を犠牲にして、積み込み〜配送の総時間を局所探索法で最小化します。

class OptimizePickupAndDeliveryDuration final {
  const sandrokottos::Problem &Problem;
  std::minstd_rand RandomNumberGenerator;

  auto getNeighborRoute(const Route &Route) noexcept {
    const auto Orders = [&] {
      auto Result = boost::container::small_vector<int, 16>{};

      std::ranges::copy(
          Route | std::views::filter([&](const auto &Node) {
            return Node % 2 == 0;
          }) | std::views::transform([&](const auto &Node) {
            return Node / 2;
          }),
          std::back_inserter(Result));

      return Result;
    }();

    if (Orders.empty()) {
      return sandrokottos::Route{};
    }

    const auto Order = Orders[std::uniform_int_distribution<>{0, static_cast<int>(std::size(Orders) - 1)}(RandomNumberGenerator)];

    auto Result = sandrokottos::Route{};

    std::ranges::copy(
        Route | std::views::filter([&](const auto &Node) {
          return Node / 2 != Order;
        }),
        std::back_inserter(Result));

    const auto PIndex = std::uniform_int_distribution<>{0, static_cast<int>(std::size(Result))}(RandomNumberGenerator);
    const auto DIndex = std::uniform_int_distribution<>{PIndex + 1, static_cast<int>(std::size(Result) + 1)}(RandomNumberGenerator);

    Result.emplace(std::begin(Result) + PIndex, Order * 2 + 0);
    Result.emplace(std::begin(Result) + DIndex, Order * 2 + 1);

    return Result;
  }

  auto isValidRoute(const int Capacity, const Route &Route) noexcept {
    auto Minute = 0;
    auto LuggageSize = 0;

    for (const auto &I : std::views::iota(0, static_cast<int>(std::size(Route)))) {
      if (I > 0) {
        Minute += Problem.getDurationMatrix()[Route[I - 1]][Route[I]];
      }

      if (Route[I] % 2 == 0) {
        if (++LuggageSize > Capacity) { // キャパシティーを超えて積み込むことはできません。
          return false;
        }
      } else {
        Minute = std::max(Minute, std::max(30, std::get<0>(Problem.getTimeWindows()[Route[I] / 2])));

        if (Minute > std::get<1>(Problem.getTimeWindows()[Route[I] / 2])) { // 希望配送時間を超えてはなりません。
          return false;
        }

        if (Minute > 150 - 2) { // 13:00の2分前までに配送しなければなりません。
          return false;
        }

        LuggageSize--;
      }
    }

    return true;
  }

  auto getCost(const Route &Route, const Timetable &Timetable) noexcept {
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
      }
    }

    return std::make_tuple(Score2, Score3);
  }

public:
  explicit OptimizePickupAndDeliveryDuration(const sandrokottos::Problem &Problem) noexcept : Problem{Problem}, RandomNumberGenerator{0} {}

  auto operator()(const Solution &Solution, const std::chrono::steady_clock::time_point &TimeLimit) noexcept {
    auto Routes = Solution.getRoutes();
    auto Timetables = Solution.getTimetables();

    while (std::chrono::steady_clock::now() <= TimeLimit) {
      const auto I = std::uniform_int_distribution<>{0, static_cast<int>(std::size(Routes) - 1)}(RandomNumberGenerator);

      const auto Route = getNeighborRoute(Routes[I]);

      if (Route.empty()) {
        continue;
      }

      if (!isValidRoute(Problem.getCapacities()[I], Route)) {
        continue;
      }

      const auto Timetable = CreateStrictTimetable{Problem}(Route);

      if (Timetable.empty()) {
        continue;
      }

      if (getCost(Route, Timetable) > getCost(Routes[I], Timetables[I])) {
        continue;
      }

      Routes[I] = Route;
      Timetables[I] = Timetable;
    }

    return sandrokottos::Solution(Routes, Timetables, CalculateCost{Problem}(Routes, Timetables));
  }
};

} // namespace sandrokottos
