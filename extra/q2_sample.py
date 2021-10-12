import json
import numpy as np
import sys

from funcy import complement, drop, iterate, mapcat, pairwise, take, takewhile
from operator import itemgetter
from ortools.constraint_solver import pywrapcp, routing_enums_pb2


def _to_minute(oclock):
    return oclock // 100 * 60 + oclock % 100 - 630  # 10:30を0にします。


def _to_oclock(minute):
    return (minute + 30) // 60 * 100 + (minute + 30) % 60 + 1000  # 10:30を0にすると11:00が30になって、1060になってしまいます。だから、+30して10:00を基準にしました。


def create_problem(question):
    def create_distance_matrix():
        locations = np.array(tuple(mapcat(lambda order: (order['r_address'], order['u_address']), take(order_count, question['orders']))), dtype=np.int32)
        return np.sum(np.abs(locations[:, None, :] - locations[None, :, :]), axis=-1)  # 距離の行列を作成します。NumPy便利。

    robot_count = len(question['robots'])
    order_count = min(len(question['orders']), 2000)
    capacities = tuple(map(itemgetter('capacity'), question['robots']))
    time_windows = tuple(map(lambda order: (_to_minute(order['start_time']), _to_minute(order['end_time']) - 2), take(order_count, question['orders'])))
    distance_matrix = create_distance_matrix()
    duration_matrix = (distance_matrix + (5 - 1)) // 5 + 2

    return {'robot_count': robot_count, 'order_count': order_count, 'capacities': capacities, 'time_windows': time_windows, 'distance_matrix': distance_matrix, 'duration_matrix': duration_matrix}


def solve(problem):
    def get_distance_matrix():
        def get_distance(from_node, to_node):
            if from_node == problem['order_count'] * 2 or to_node == problem['order_count'] * 2:  # 最後のノードはステーションなので、距離は0になります。
                return 0

            return problem['distance_matrix'][from_node][to_node]

        return list(map(lambda from_node: tuple(map(lambda to_node: get_distance(from_node, to_node), range(problem['order_count'] * 2 + 1))), range(problem['order_count'] * 2 + 1)))

    def get_duration_matrix():
        def get_duration(from_node, to_node):
            if from_node == problem['order_count'] * 2 or to_node == problem['order_count'] * 2:  # 最後のノードはステーションなので、移動時間は0になります。
                return 0

            return problem['duration_matrix'][from_node][to_node]

        return list(map(lambda from_node: tuple(map(lambda to_node: get_duration(from_node, to_node), range(problem['order_count'] * 2 + 1))), range(problem['order_count'] * 2 + 1)))

    def get_luggage_delta_vector():
        def get_luggage_delta(node):
            if node == problem['order_count'] * 2:
                return 0

            return 1 if node % 2 == 0 else -1

        return tuple(map(lambda node: get_luggage_delta(node), range(problem['order_count'] * 2 + 1)))

    routing_manager = pywrapcp.RoutingIndexManager(problem['order_count'] * 2 + 1, problem['robot_count'], problem['order_count'] * 2)  # ノードの最後にステーションを足しておきます。
    routing_model = pywrapcp.RoutingModel(routing_manager)

    # コストは、総走行距離にしてみました。

    routing_model.SetArcCostEvaluatorOfAllVehicles(routing_model.RegisterTransitMatrix(get_distance_matrix()))  # OR-ToolsのガイドだとRegisterTransitCallbackを使用していますけど、距離が必要になるたびに遅いPythonを呼ばれるのは嫌なので、RegisterTransitMatrixを使用しました。

    # ノードを訪問しない場合のペナルティを設定します。

    for i in range(problem['order_count'] * 2):
        routing_model.AddDisjunction((routing_manager.NodeToIndex(i),), 300)  # コストは距離なので、ノード間の最長距離をペナルティに設定します。

    # 荷物の量がキャパシティを超えてはなりません。

    routing_model.AddDimensionWithVehicleCapacity(routing_model.RegisterUnaryTransitVector(get_luggage_delta_vector()), 0, problem['capacities'], True, 'capacity')

    # 注文単位で、積み込みの後に配送をしなければなりません。

    for i in range(problem['order_count']):
        routing_model.AddPickupAndDelivery(routing_manager.NodeToIndex(i * 2), routing_manager.NodeToIndex(i * 2 + 1))

    # 13:00の2分前までに、配送しなければなりません。

    routing_model.AddDimension(routing_model.RegisterTransitMatrix(get_duration_matrix()), 150 - 2, 150 - 2, False, 'time')

    # 時刻はこの先の処理で必要となるので、変数に保存しておきます。

    time_dimension = routing_model.GetDimensionOrDie('time')

    # 希望配送時間に配送しなければなりません。SetCumulVarSoftLowerBoundを使用すれば時間外配送も許可できるみたいなのだけど、係数調整が大変なのと、試してみた範囲ではスコアが下がっちゃったので、とりあえずは無視で。

    for i, (time_window_lower, time_window_upper) in enumerate(problem['time_windows']):
        time_dimension.CumulVar(routing_manager.NodeToIndex(i * 2 + 1)).SetRange(max(time_window_lower, 30), min(time_window_upper, 150 - 2))  # 配送が可能なのは、11:00〜12:58まで。希望配送時間の最遅が13:00でも、12:58までに配送しないと完了が13:00を超えるのでエラーになってしまう。最早は11:00前だとエラーだし、AddDimensionするときに制約をかけているから、minとmaxがなくても大丈夫なんだけど。。。

    # 配送はできるだけ早く、積み込みはできるだけ遅くします。

    for i in range(problem['order_count']):
        routing_model.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing_manager.NodeToIndex(i * 2 + 1)))
        routing_model.AddVariableMaximizedByFinalizer(time_dimension.CumulVar(routing_manager.NodeToIndex(i * 2)))

    # 問題を解きます。

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.use_full_propagation = True
    search_parameters.time_limit.seconds = 19
    search_parameters.time_limit.nanos = 500_000_000

    routing_solution = routing_model.SolveWithParameters(search_parameters)

    if not routing_solution:
        return None

    # ソリューションを作成してリターンします。

    def create_route_and_timetable(robot):
        indexes = tuple(drop(1, takewhile(complement(routing_model.IsEnd), iterate(lambda index: routing_solution.Value(routing_model.NextVar(index)), routing_model.Start(robot)))))

        route = tuple(map(lambda index: routing_manager.IndexToNode(index), indexes))
        timetable = tuple(map(lambda index: routing_solution.Min(time_dimension.CumulVar(index)), indexes))

        return route, timetable

    routes, timetables = zip(*map(create_route_and_timetable, range(problem['robot_count'])))

    return {'routes': routes, 'timetables': timetables}


def calculate_cost(problem, solution):
    score_1 = 0
    score_2 = 0
    score_3 = 0

    for route, timetable in zip(solution['routes'], solution['timetables']):
        picked = set()

        for (prev_node_index, node_index), (prev_minute, minute) in zip(pairwise((None,) + tuple(route)), pairwise((None,) + tuple(timetable))):
            order_index = node_index // 2

            if prev_minute is not None:
                score_2 += (minute - prev_minute) * len(picked) - (2 if node_index % 2 == 0 else 0)

            if prev_node_index is not None:
                score_3 += problem['distance_matrix'][prev_node_index][node_index]

            if node_index % 2 == 0:
                picked.add(order_index)
            else:
                picked.remove(order_index)

                lower, upper = problem['time_windows'][order_index]
                score_1 += 100 if lower <= minute <= upper else max(80 - max(lower - minute, minute - upper), 20)

    return (-score_1, score_2, score_3)


def create_answer(question, solution):
    def create_plans():
        def create_detail_plans(route, timetable):
            for i, (node, minute) in enumerate(zip(route, timetable)):
                yield {
                    'id': i,
                    'order_id': question['orders'][node // 2]['id'],
                    'action': 'load' if node % 2 == 0 else 'deliver',
                    'start_time': _to_oclock(minute)
                }

        for i, (route, timetable) in enumerate(zip(solution['routes'], solution['timetables'])):
            yield {'robot': question['robots'][i]['id'], 'detail_plans': tuple(create_detail_plans(route, timetable))}

    return {'plans': tuple(create_plans())}


def main():
    question = json.load(sys.stdin)

    problem = create_problem(question)
    solution = solve(problem)

    if not solution:
        sys.exit(1)

    cost = calculate_cost(problem, solution)
    print(f'sample\t{cost[0]}\t{cost[1]}\t{cost[2]}', file=sys.stderr)

    answer = create_answer(question, solution)
    json.dump(answer, sys.stdout)


if __name__ == '__main__':
    main()
