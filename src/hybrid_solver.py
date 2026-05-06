import math
import random
import time

from src.initial_solution import create_initial_truck_route
from src.drone_scheduler import create_drone_schedule
from src.models import Solution
from src.evaluator import evaluate_solution
from src.local_search import (
    adaptive_operator_selection,
    improve_solution_once,
    normalize_operator_weights,
    or_opt_drone_subroutes
)


def accept_solution(current_solution, candidate_solution, temperature):
    """
    Simulated Annealing acceptance rule.
    """
    if candidate_solution.makespan < current_solution.makespan:
        return True

    if temperature <= 0:
        return False

    delta = candidate_solution.makespan - current_solution.makespan
    probability = math.exp(-delta / temperature)

    return random.random() < probability


def solve_two_stage_hybrid(
    truck_time,
    drone_time,
    all_nodes,
    start_node,
    end_node,
    battery_capacity,
    max_iterations=200,
    initial_temperature=10.0,
    cooling_rate=0.995,
    keep_every=2
):
    """
    Two-Stage Hybrid Solver with:
    - 3 destruction operators
    - adaptive operator selection
    - Or-Opt local search
    - Simulated Annealing acceptance
    - adaptive cooling
    """
    start_time = time.time()

    truck_route, full_truck_route = create_initial_truck_route(
        truck_time,
        start_node,
        end_node,
        keep_every=keep_every
    )

    truck_route, drone_assignments = create_drone_schedule(
        truck_route,
        all_nodes,
        drone_time,
        battery_capacity
    )

    drone_assignments = or_opt_drone_subroutes(
        drone_assignments,
        drone_time,
        battery_capacity
    )

    current_solution = Solution(
        truck_route=truck_route,
        drone_assignments=drone_assignments
    )

    current_solution = evaluate_solution(
        current_solution,
        truck_time,
        drone_time,
        battery_capacity
    )

    best_solution = current_solution
    temperature = initial_temperature

    operator_weights = {
        "random": 1 / 3,
        "worst_position": 1 / 3,
        "zone_based": 1 / 3
    }

    operator_scores = {
        "random": 0.0,
        "worst_position": 0.0,
        "zone_based": 0.0
    }

    operator_counts = {
        "random": 0,
        "worst_position": 0,
        "zone_based": 0
    }

    history = []
    no_improvement_count = 0

    for iteration in range(max_iterations):
        selected_operator = adaptive_operator_selection(operator_weights)

        candidate_solution = improve_solution_once(
            current_solution=current_solution,
            all_nodes=all_nodes,
            truck_time=truck_time,
            drone_time=drone_time,
            battery_capacity=battery_capacity,
            operator_name=selected_operator
        )

        operator_counts[selected_operator] += 1

        if candidate_solution.makespan < current_solution.makespan:
            operator_scores[selected_operator] += 2.0
        elif candidate_solution.makespan < best_solution.makespan:
            operator_scores[selected_operator] += 3.0
        elif candidate_solution.feasible:
            operator_scores[selected_operator] += 0.5

        if accept_solution(current_solution, candidate_solution, temperature):
            current_solution = candidate_solution

        if (
            current_solution.feasible
            and current_solution.makespan < best_solution.makespan
        ):
            best_solution = current_solution
            no_improvement_count = 0
        else:
            no_improvement_count += 1

        if iteration > 0 and iteration % 50 == 0:
            operator_weights = normalize_operator_weights(
                operator_scores,
                operator_counts
            )

            operator_scores = {
                "random": 0.0,
                "worst_position": 0.0,
                "zone_based": 0.0
            }

            operator_counts = {
                "random": 0,
                "worst_position": 0,
                "zone_based": 0
            }

        history.append({
            "iteration": iteration,
            "current_makespan": current_solution.makespan,
            "best_makespan": best_solution.makespan,
            "temperature": temperature,
            "selected_operator": selected_operator,
            "weight_random": operator_weights["random"],
            "weight_worst_position": operator_weights["worst_position"],
            "weight_zone_based": operator_weights["zone_based"]
        })

        if no_improvement_count > 50:
            adaptive_cooling_rate = min(cooling_rate + 0.002, 0.9995)
        else:
            adaptive_cooling_rate = cooling_rate

        temperature *= adaptive_cooling_rate

    runtime = time.time() - start_time

    return {
        "best_solution": best_solution,
        "initial_full_truck_route": full_truck_route,
        "history": history,
        "runtime": runtime
    }