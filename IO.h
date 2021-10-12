#pragma once

#include <algorithm>
#include <cmath>
#include <istream>
#include <iterator>
#include <ostream>
#include <ranges>
#include <tuple>
#include <vector>

#include <nlohmann/json.hpp>

#include "Model.h"

namespace sandrokottos {

inline auto readQuestion(std::istream &Stream) noexcept {
  auto Result = nlohmann::json{};

  Stream >> Result;

  return Result;
}

inline auto writeAnswer(std::ostream &Stream, const nlohmann::json &JSON) noexcept {
  Stream << JSON << std::endl;
}

inline auto getMinute(int OClock) noexcept {
  return OClock / 100 * 60 + OClock % 100 - 630;
}

inline auto getOClock(int Minute) noexcept {
  return (Minute + 30) / 60 * 100 + (Minute + 30) % 60 + 1000;
}

inline auto convertToProblem(const nlohmann::json &Question) noexcept {
  const auto RobotSize = static_cast<int>(std::size(Question["robots"]));

  const auto OrderSize = std::min(static_cast<int>(std::size(Question["orders"])), 2'000);

  const auto Capacities = [&] {
    auto Result = std::vector<int>{};

    std::ranges::copy(
        std::views::iota(0, RobotSize) | std::views::transform([&](const auto &I) {
          return Question["robots"][I]["capacity"];
        }),
        std::back_inserter(Result));

    return Result;
  }();

  const auto TimeWindows = [&] {
    auto Result = std::vector<std::tuple<int, int>>{};

    std::ranges::copy(
        std::views::iota(0, OrderSize) | std::views::transform([&](const auto &I) {
          return std::make_tuple(getMinute(Question["orders"][I]["start_time"]), getMinute(Question["orders"][I]["end_time"]) - 2);
        }),
        std::back_inserter(Result));

    return Result;
  }();

  const auto DistanceMatrix = [&] {
    const auto Locations = [&] {
      // c++23がリリースされたら、flat_mapで書き換える。

      const auto LocationPairs = [&] {
        auto Result = std::vector<std::vector<std::tuple<int, int>>>{};

        std::ranges::copy(
            std::views::iota(0, OrderSize) | std::views::transform([&](const auto &I) {
              return std::vector<std::tuple<int, int>>{{Question["orders"][I]["r_address"][0], Question["orders"][I]["r_address"][1]},
                                                       {Question["orders"][I]["u_address"][0], Question["orders"][I]["u_address"][1]}};
            }),
            std::back_inserter(Result));

        return Result;
      }();

      auto Result = std::vector<std::tuple<int, int>>{};

      std::ranges::copy(LocationPairs | std::views::join, std::back_inserter(Result));

      return Result;
    }();

    auto Result = std::vector<std::vector<int>>{};

    std::ranges::copy(
        Locations | std::views::transform([&](const auto &Location1) {
          auto Result = std::vector<int>{};

          std::ranges::copy(
              Locations | std::views::transform([&](const auto &Location2) {
                return std::abs(std::get<0>(Location1) - std::get<0>(Location2)) + std::abs(std::get<1>(Location1) - std::get<1>(Location2));
              }),
              std::back_inserter(Result));

          return Result;
        }),
        std::back_inserter(Result));

    return Result;
  }();

  const auto DurationMatrix = [&] {
    auto Result = std::vector<std::vector<int>>{};

    std::ranges::copy(
        DistanceMatrix | std::views::transform([&](const auto &Distances) {
          auto Result = std::vector<int>{};

          std::ranges::copy(
              Distances | std::views::transform([](const auto &Distance) {
                return (Distance + (5 - 1)) / 5 + 2;
              }),
              std::back_inserter(Result));

          return Result;
        }),
        std::back_inserter(Result));

    return Result;
  }();

  return Problem{RobotSize, OrderSize, Capacities, TimeWindows, DistanceMatrix, DurationMatrix};
}

inline auto convertToAnswer(const nlohmann::json &Question, const Problem &Problem, const Solution &Solution) noexcept {
  auto Result = nlohmann::json{};

  Result["plans"] = [&] {
    auto Result = nlohmann::json::array();

    std::ranges::copy(
        std::views::iota(0, static_cast<int>(std::size(Solution.getRoutes()))) | std::views::transform([&](const auto &I) {
          return nlohmann::json::object(
              {{"robot", Question["robots"][I]["id"]},
               {"detail_plans", [&] {
                  auto Result = nlohmann::json::array();

                  std::ranges::copy(
                      std::views::iota(0, static_cast<int>(std::size(Solution.getRoutes()[I]))) | std::views::transform([&](const auto &J) {
                        return nlohmann::json::object(
                            {{"id", J},
                             {"order_id", Question["orders"][Solution.getRoutes()[I][J] / 2]["id"]},
                             {"action", Solution.getRoutes()[I][J] % 2 == 0 ? "load" : "deliver"},
                             {"start_time", getOClock(Solution.getTimetables()[I][J])}});
                      }),
                      std::back_inserter(Result));

                  return Result;
                }()}});
        }),
        std::back_inserter(Result));

    return Result;
  }();

  return Result;
}

} // namespace sandrokottos
