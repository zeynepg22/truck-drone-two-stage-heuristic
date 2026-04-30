import random
from copy import deepcopy

from src.drone_scheduler import create_drone_schedule
from src.models import Solution
from src.evaluator import evaluate_solution


def swap_two_truck_nodes(truck_route):
    """
    Creates a neighbor solution by swapping two middle truck nodes.
    Origin and destination are not changed.
    """
    if len(truck_route) <= 4:
        return truck_route[:]

    new_route = truck_route[:]

    i, j = random.sample(range(1, len(new_route) - 1), 2)
    new_route[i], new_route[j] = new_route[j], new_route[i]

    return new_route


def reverse_truck_segment(truck_route):
    """
    Creates a neighbor by reversing a segment of the truck route.
    Origin and destination stay fixed.
    """
    if len(truck_route) <= 4:
        return truck_route[:]

    new_route = truck_route[:]

    i, j = sorted(random.sample(range(1, len(new_route) - 1), 2))
    new_route[i:j + 1] = reversed(new_route[i:j + 1])

    return new_route


def insert_truck_node(truck_route):
    """
    Creates a neighbor by moving one middle truck node to another position.
    """
    if len(truck_route) <= 4:
        return truck_route[:]

    new_route = truck_route[:]

    i, j = random.sample(range(1, len(new_route) - 1), 2)
    node = new_route.pop(i)
    new_route.insert(j, node)

    return new_route


def generate_neighbor_route(truck_route):
    """
    Randomly selects one local search move.
    """
    operator = random.choice([
        swap_two_truck_nodes,
        reverse_truck_segment,
        insert_truck_node
    ])

    return operator(truck_route)


def improve_solution_once(
    current_solution,
    all_nodes,
    truck_time,
    drone_time,
    battery_capacity
):
    """
    Generates one neighbor solution and evaluates it.

    The truck route is slightly changed.
    Then drone assignments are rebuilt according to the new truck route.
    """
    candidate_truck_route = generate_neighbor_route(
        current_solution.truck_route
    )

    candidate_truck_route, candidate_drone_assignments = create_drone_schedule(
        candidate_truck_route,
        all_nodes,
        drone_time,
        battery_capacity
    )

    candidate_solution = Solution(
        truck_route=candidate_truck_route,
        drone_assignments=candidate_drone_assignments
    )

    candidate_solution = evaluate_solution(
        candidate_solution,
        truck_time,
        drone_time,
        battery_capacity
    )

    return candidate_solution