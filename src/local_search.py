import math
import random

from src.drone_scheduler import create_drone_schedule
from src.models import Solution
from src.evaluator import evaluate_solution


def route_distance(route, time_matrix):
    total = 0.0
    for i in range(len(route) - 1):
        total += time_matrix[route[i]][route[i + 1]]
    return total


def random_destruction(truck_route):
    """
    Random destruction:
    removes a random segment and reinserts it in reversed order.
    """
    if len(truck_route) <= 4:
        return truck_route[:]

    new_route = truck_route[:]
    i, j = sorted(random.sample(range(1, len(new_route) - 1), 2))

    segment = new_route[i:j + 1]
    remaining = new_route[:i] + new_route[j + 1:]

    insert_pos = random.randint(1, len(remaining) - 1)
    new_route = remaining[:insert_pos] + list(reversed(segment)) + remaining[insert_pos:]

    return new_route


def worst_position_destruction(truck_route, truck_time):
    """
    Worst-position destruction:
    finds the node with the highest marginal cost and relocates it.
    """
    if len(truck_route) <= 4:
        return truck_route[:]

    worst_index = 1
    worst_cost = -math.inf

    for i in range(1, len(truck_route) - 1):
        prev_node = truck_route[i - 1]
        node = truck_route[i]
        next_node = truck_route[i + 1]

        marginal_cost = (
            truck_time[prev_node][node]
            + truck_time[node][next_node]
            - truck_time[prev_node][next_node]
        )

        if marginal_cost > worst_cost:
            worst_cost = marginal_cost
            worst_index = i

    new_route = truck_route[:]
    removed_node = new_route.pop(worst_index)

    best_route = None
    best_cost = math.inf

    for pos in range(1, len(new_route)):
        candidate = new_route[:pos] + [removed_node] + new_route[pos:]
        candidate_cost = route_distance(candidate, truck_time)

        if candidate_cost < best_cost:
            best_cost = candidate_cost
            best_route = candidate

    return best_route


def zone_based_destruction(truck_route, truck_time):
    """
    Zone-based destruction:
    selects nodes that are close to a randomly chosen center node.
    Since coordinates may not be available, truck_time matrix is used as proximity.
    """
    if len(truck_route) <= 5:
        return truck_route[:]

    middle_nodes = truck_route[1:-1]
    center_node = random.choice(middle_nodes)

    distances = []

    for node in middle_nodes:
        if node == center_node:
            continue

        proximity = truck_time[center_node][node]
        distances.append((proximity, node))

    distances.sort()

    selected_nodes = [center_node]
    selected_nodes += [
        node for _, node in distances[:min(2, len(distances))]
    ]

    remaining = [node for node in truck_route if node not in selected_nodes]

    insert_pos = random.randint(1, len(remaining) - 1)
    new_route = remaining[:insert_pos] + selected_nodes + remaining[insert_pos:]

    return new_route

def adaptive_operator_selection(operator_weights):
    """
    Roulette-wheel selection based on operator weights.
    """
    total_weight = sum(operator_weights.values())
    pick = random.uniform(0, total_weight)

    cumulative = 0.0
    for operator_name, weight in operator_weights.items():
        cumulative += weight
        if pick <= cumulative:
            return operator_name

    return random.choice(list(operator_weights.keys()))


def normalize_operator_weights(operator_scores, operator_counts):
    """
    Updates operator weights using average success scores.
    """
    new_weights = {}

    for operator_name in operator_scores:
        if operator_counts[operator_name] > 0:
            new_weights[operator_name] = (
                operator_scores[operator_name] / operator_counts[operator_name]
            )
        else:
            new_weights[operator_name] = 1.0

    total = sum(new_weights.values())

    if total == 0:
        return {name: 1 / len(new_weights) for name in new_weights}

    return {name: value / total for name, value in new_weights.items()}


def apply_destruction_operator(
    truck_route,
    truck_time,
    all_nodes,
    operator_name
):
    if operator_name == "random":
        return random_destruction(truck_route)

    if operator_name == "worst_position":
        return worst_position_destruction(truck_route, truck_time)

    if operator_name == "zone_based":
        return zone_based_destruction(truck_route, truck_time)

    return random_destruction(truck_route)


def or_opt_drone_subroutes(drone_assignments, drone_time, battery_capacity):
    """
    Or-Opt local search for drone sub-routes.
    It relocates segments of length 1, 2, or 3 inside each drone path.
    """

    improved_assignments = {}

    for edge, path in drone_assignments.items():
        start_node, end_node = edge
        best_path = path[:]

        def route_cost(route):
            if len(route) == 0:
                return drone_time[start_node][end_node]

            total = drone_time[start_node][route[0]]

            for i in range(len(route) - 1):
                total += drone_time[route[i]][route[i + 1]]

            total += drone_time[route[-1]][end_node]
            return total

        best_cost = route_cost(best_path)
        improved = True

        while improved:
            improved = False

            for segment_length in [1, 2, 3]:
                if len(best_path) < segment_length:
                    continue

                for i in range(len(best_path) - segment_length + 1):
                    segment = best_path[i:i + segment_length]
                    remaining = best_path[:i] + best_path[i + segment_length:]

                    for j in range(len(remaining) + 1):
                        candidate = remaining[:j] + segment + remaining[j:]
                        candidate_cost = route_cost(candidate)

                        if (
                            candidate_cost < best_cost
                            and candidate_cost <= battery_capacity
                        ):
                            best_path = candidate
                            best_cost = candidate_cost
                            improved = True
                            break

                    if improved:
                        break

                if improved:
                    break

        improved_assignments[edge] = best_path

    return improved_assignments


def improve_solution_once(
    current_solution,
    all_nodes,
    truck_time,
    drone_time,
    battery_capacity,
    operator_name="random"
):
    """
    Generates one candidate solution using the selected destruction operator.
    Then rebuilds drone assignments and improves drone sub-routes with Or-Opt.
    """
    candidate_truck_route = apply_destruction_operator(
        truck_route=current_solution.truck_route,
        truck_time=truck_time,
        all_nodes=all_nodes,
        operator_name=operator_name
    )

    candidate_truck_route, candidate_drone_assignments = create_drone_schedule(
        candidate_truck_route,
        all_nodes,
        drone_time,
        battery_capacity
    )

    candidate_drone_assignments = or_opt_drone_subroutes(
        candidate_drone_assignments,
        drone_time,
        battery_capacity
    )

    candidate_solution = Solution(
        truck_route=candidate_truck_route,
        drone_assignments=candidate_drone_assignments
    )

    return evaluate_solution(
        candidate_solution,
        truck_time,
        drone_time,
        battery_capacity
    )