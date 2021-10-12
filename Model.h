#pragma once

#include <algorithm>
#include <iostream>
#include <iterator>
#include <ranges>
#include <tuple>
#include <vector>

#include <boost/container/small_vector.hpp>
#include <ortools/sat/cp_model.h>

namespace sandrokottos {

using Route = boost::container::small_vector<int, 32>;
using Timetable = boost::container::small_vector<int, 32>;

class Problem final {
  int RobotSize;
  int OrderSize;
  std::vector<int> Capacities;
  std::vector<std::tuple<int, int>> TimeWindows;
  std::vector<std::vector<int>> DistanceMatrix;
  std::vector<std::vector<int>> DurationMatrix;

public:
  explicit Problem(int RobotSize, int OrderSize, const std::vector<int> &Capacities, const std::vector<std::tuple<int, int>> &TimeWindows, const std::vector<std::vector<int>> &DistanceMatrix, const std::vector<std::vector<int>> &DurationMatrix) noexcept
      : RobotSize{RobotSize}, OrderSize{OrderSize}, Capacities{Capacities}, TimeWindows{TimeWindows}, DistanceMatrix{DistanceMatrix}, DurationMatrix{DurationMatrix} {}

  auto getRobotSize() const noexcept {
    return RobotSize;
  }

  auto getOrderSize() const noexcept {
    return OrderSize;
  }

  const auto &getCapacities() const noexcept {
    return Capacities;
  }

  const auto &getTimeWindows() const noexcept {
    return TimeWindows;
  }

  const auto &getDistanceMatrix() const noexcept {
    return DistanceMatrix;
  }

  const auto &getDurationMatrix() const noexcept {
    return DurationMatrix;
  }
};

class Solution final {
  std::vector<Route> Routes;
  std::vector<Timetable> Timetables;
  std::tuple<int, int, int> Cost;

public:
  explicit Solution(const std::vector<Route> &Routes, const std::vector<Timetable> &Timetables, std::tuple<int, int, int> Cost) noexcept : Routes{Routes}, Timetables{Timetables}, Cost{Cost} {}

  Solution() {}

  const auto &getRoutes() const noexcept {
    return Routes;
  }

  const auto &getTimetables() const noexcept {
    return Timetables;
  }

  const auto &getCost() const noexcept {
    return Cost;
  }
};

class CreateStrictTimetable final {
  const sandrokottos::Problem &Problem;

public:
  explicit CreateStrictTimetable(const sandrokottos::Problem &Problem) noexcept : Problem{Problem} {}

  auto operator()(const Route &Route) const noexcept {
    if (Route.empty()) {
      return Timetable{};
    }

    auto ModelBuilder = operations_research::sat::CpModelBuilder{};

    // 積み込みや配送の時刻を表現する変数を作成します。

    const auto Minutes = [&] {
      auto Result = std::vector<operations_research::sat::IntVar>{};

      std::ranges::copy(
          Route | std::views::transform([&](const auto &_) {
            return ModelBuilder.NewIntVar({0, 150 - 2});
          }),
          std::back_inserter(Result));

      return Result;
    }();

    // 積み込みや配送の時刻を計算しながら、配送時刻の制約を追加して、コストを作成します。

    const auto TotalCostExpr = [&] {
      auto Result = operations_research::sat::LinearExpr{0};

      auto MinuteExpr = operations_research::sat::LinearExpr{0};

      for (const auto &I : std::views::iota(0, static_cast<int>(std::size(Route)))) {
        // 時刻に移動時間を追加します。

        if (I > 0) {
          MinuteExpr.AddConstant(Problem.getDurationMatrix()[Route[I - 1]][Route[I]]);
        }

        // 時刻に待ち時間を追加します。

        MinuteExpr.AddVar(ModelBuilder.NewIntVar({0, 150 - 2}));

        // 時刻を変数に設定します。

        ModelBuilder.AddEquality(Minutes[I], MinuteExpr);

        // 積み込みの場合は、制約やコスト変数とは無関係なのでコンティニューします。

        if (Route[I] % 2 == 0) {
          continue;
        }

        // 配送時刻の制約を追加します。

        ModelBuilder.AddGreaterOrEqual(Minutes[I], 30);
        ModelBuilder.AddLinearConstraint(Minutes[I], {std::get<0>(Problem.getTimeWindows()[Route[I] / 2]), std::get<1>(Problem.getTimeWindows()[Route[I] / 2])});

        // 積み込み〜配送までの時間を変数に設定します。

        const auto Cost = ModelBuilder.NewIntVar({0, 150 - 2});
        ModelBuilder.AddEquality(
            Cost,
            [&] {
              auto Result = MinuteExpr;

              Result.AddTerm(Minutes[std::distance(std::begin(Route), std::find(std::begin(Route), std::end(Route), Route[I] / 2 * 2))], -1);

              return Result;
            }());

        // 積み込み〜配送までの時間をコストに追加します。

        Result.AddVar(Cost);
      }

      return Result;
    }();

    // コストを最小化するように指示します。

    ModelBuilder.Minimize(TotalCostExpr);

    // 問題を解きます。

    const auto Solution = operations_research::sat::Solve(ModelBuilder.Build());

    if (Solution.status() == operations_research::sat::CpSolverStatus::INFEASIBLE) {
      std::cerr << "SAT FAILED..." << std::endl;
      return Timetable{};
    }

    // タイムテーブルを作成してリターンします。

    return [&] {
      auto Result = Timetable{};

      std::ranges::copy(
          Minutes | std::views::transform([&](const auto &Minute) {
            return static_cast<int>(operations_research::sat::SolutionIntegerValue(Solution, Minute));
          }),
          std::back_inserter(Result));

      return Result;
    }();
  }
};

class CreateRelaxedTimetable final {
  const sandrokottos::Problem &Problem;

public:
  explicit CreateRelaxedTimetable(const sandrokottos::Problem &Problem) noexcept : Problem{Problem} {}

  auto operator()(const Route &Route) const noexcept {
    if (Route.empty()) {
      return Timetable{};
    }

    auto ModelBuilder = operations_research::sat::CpModelBuilder{};

    // 積み込みや配送の時刻を表現する変数を作成します。

    const auto Minutes = [&] {
      auto Result = std::vector<operations_research::sat::IntVar>{};

      std::ranges::copy(
          Route | std::views::transform([&](const auto &_) {
            return ModelBuilder.NewIntVar({0, 150 - 2});
          }),
          std::back_inserter(Result));

      return Result;
    }();

    // 希望配送時間に配送したことを表現する変数を作成します。

    const auto OnTimes = [&] {
      auto Result = std::vector<operations_research::sat::BoolVar>{};

      std::ranges::copy(
          Route | std::views::transform([&](const auto &_) {
            return ModelBuilder.NewBoolVar();
          }),
          std::back_inserter(Result));

      return Result;
    }();

    // 積み込みや配送の時刻を計算しながら、配送時刻の制約を追加して、コストを作成します。

    const auto TotalCostExpr = [&] {
      auto Result = operations_research::sat::LinearExpr{0};

      auto MinuteExpr = operations_research::sat::LinearExpr{0};

      for (const auto &I : std::views::iota(0, static_cast<int>(std::size(Route)))) {
        // 時刻に移動時間を追加します。

        if (I > 0) {
          MinuteExpr.AddConstant(Problem.getDurationMatrix()[Route[I - 1]][Route[I]]);
        }

        // 時刻に待ち時間を追加します。

        MinuteExpr.AddVar(ModelBuilder.NewIntVar({0, 150 - 2}));

        // 時刻を変数に設定します。

        ModelBuilder.AddEquality(Minutes[I], MinuteExpr);

        // 積み込みの場合は、制約やコスト変数とは無関係なのでコンティニューします。

        if (Route[I] % 2 == 0) {
          continue;
        }

        // 配送時刻の制約を追加します。

        ModelBuilder.AddGreaterOrEqual(Minutes[I], 30);
        ModelBuilder.AddLinearConstraint(Minutes[I], {std::get<0>(Problem.getTimeWindows()[Route[I] / 2]), std::get<1>(Problem.getTimeWindows()[Route[I] / 2])}).OnlyEnforceIf(OnTimes[I]);

        // 配送件数（希望配送時間通りなら0、そうでなければ20～80）を変数に設定します。

        const auto Cost1 = ModelBuilder.NewIntVar({0, 80});
        ModelBuilder.AddEquality(
            Cost1,
            [&] {
              const auto LowerDifference = ModelBuilder.NewIntVar({-150 + 2, 150 - 2});
              ModelBuilder.AddEquality(
                  LowerDifference,
                  [&] {
                    auto Result = operations_research::sat::LinearExpr(std::get<0>(Problem.getTimeWindows()[Route[I] / 2]));

                    Result.AddTerm(Minutes[I], -1);

                    return Result;
                  }());

              const auto UpperDifference = ModelBuilder.NewIntVar({-150 + 2, 150 - 2});
              ModelBuilder.AddEquality(
                  UpperDifference,
                  [&] {
                    auto Result = operations_research::sat::LinearExpr(-std::get<1>(Problem.getTimeWindows()[Route[I] / 2]));

                    Result.AddTerm(Minutes[I], 1);

                    return Result;
                  }());

              const auto Difference = ModelBuilder.NewIntVar({0, 150 - 2});
              ModelBuilder.AddMaxEquality(Difference, {LowerDifference, UpperDifference, ModelBuilder.NewConstant(0)});

              const auto Penalty = ModelBuilder.NewIntVar({0, 60});
              ModelBuilder.AddMinEquality(Penalty, {Difference, ModelBuilder.NewConstant(60)});

              return [&] {
                auto Result = operations_research::sat::LinearExpr(20);

                Result.AddTerm(Penalty, 1);
                Result.AddTerm(OnTimes[I], -20);

                return Result;
              }();
            }());

        // 積み込み〜配送までの時間を変数に設定します。

        const auto Cost2 = ModelBuilder.NewIntVar({0, 150 - 2});
        ModelBuilder.AddEquality(
            Cost2,
            [&] {
              auto Result = MinuteExpr;

              Result.AddTerm(Minutes[std::distance(std::begin(Route), std::find(std::begin(Route), std::end(Route), Route[I] / 2 * 2))], -1);

              return Result;
            }());

        // 積み込み〜配送までの時間をコストに追加します。

        Result.AddTerm(Cost1, (150 - 2) * static_cast<int>(std::size(Route) / 2));
        Result.AddTerm(Cost2, 1);
      }

      return Result;
    }();

    // コストを最小化するように指示します。

    ModelBuilder.Minimize(TotalCostExpr);

    // 問題を解きます。

    const auto Solution = operations_research::sat::Solve(ModelBuilder.Build());

    if (Solution.status() == operations_research::sat::CpSolverStatus::INFEASIBLE) {
      std::cerr << "SAT FAILED..." << std::endl;
      return Timetable{};
    }

    // タイムテーブルを作成してリターンします。

    return [&] {
      auto Result = Timetable{};

      std::ranges::copy(
          Minutes | std::views::transform([&](const auto &Minute) {
            return static_cast<int>(operations_research::sat::SolutionIntegerValue(Solution, Minute));
          }),
          std::back_inserter(Result));

      return Result;
    }();
  }
};

class CalculateCost final {
  const sandrokottos::Problem &Problem;

public:
  CalculateCost(const sandrokottos::Problem &Problem) noexcept : Problem{Problem} {}

  auto operator()(const std::vector<Route> &Routes, const std::vector<Timetable> &Timetables) const noexcept {
    auto Score1 = 0;
    auto Score2 = 0;
    auto Score3 = 0;

    for (const auto &I : std::views::iota(0, Problem.getRobotSize())) {
      auto LuggageSize = 0;

      for (const auto &J : std::views::iota(0, static_cast<int>(std::size(Routes[I])))) {
        if (J > 0) {
          Score2 += (Timetables[I][J] - Timetables[I][J - 1]) * LuggageSize;
          Score3 += Problem.getDistanceMatrix()[Routes[I][J - 1]][Routes[I][J]];
        }

        if (Routes[I][J] % 2 == 0) {
          LuggageSize++;

          Score2 -= 2; // 次のループで積み込み時間＋移動時間が足されるので、事前に積み込み時間分を減らしておきます。
        } else {
          LuggageSize--;

          const auto &[Lower, Upper] = Problem.getTimeWindows()[Routes[I][J] / 2];
          Score1 += Lower <= Timetables[I][J] && Timetables[I][J] <= Upper ? 100 : std::max(80 - std::max(Lower - Timetables[I][J], Timetables[I][J] - Upper), 20);
        }
      }
    }

    return std::make_tuple(-Score1, Score2, Score3);
  }
};

} // namespace sandrokottos
