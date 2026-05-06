import math
import random
import time

from src.initial_solution import create_initial_truck_route
from src.drone_scheduler import create_drone_schedule
from src.models import Solution
from src.evaluator import evaluate_solution
from src.local_search import improve_solution_once, or_opt_drone_subroutes


def accept_solution(current_solution, candidate_solution, temperature):
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
    history = []
    no_improvement_count = 0

    for iteration in range(max_iterations):
        candidate_solution = improve_solution_once(
            current_solution=current_solution,
            all_nodes=all_nodes,
            truck_time=truck_time,
            drone_time=drone_time,
            battery_capacity=battery_capacity
        )

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

        history.append({
            "iteration": iteration,
            "current_makespan": current_solution.makespan,
            "best_makespan": best_solution.makespan,
            "temperature": temperature
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