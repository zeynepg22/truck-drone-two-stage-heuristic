from src.drone_scheduler import drone_subroute_cost


def compute_truck_edge_time(start_node, end_node, truck_time):
    """
    Computes truck travel time between two synchronization nodes.
    """
    return truck_time[start_node][end_node]


def compute_drone_edge_time(start_node, drone_nodes, end_node, drone_time):
    """
    Computes drone travel time for one drone sub-route:
    start_node -> drone_nodes -> end_node
    """
    return drone_subroute_cost(
        start_node,
        drone_nodes,
        end_node,
        drone_time
    )


def evaluate_solution(solution, truck_time, drone_time, battery_capacity):
    """
    Evaluates a complete truck-drone solution.

    For every truck edge (A, B):
        - Truck travels directly from A to B.
        - Drone starts from A, visits assigned drone nodes, and lands at B.
        - Both vehicles synchronize at B.
        - The duration of this stage is max(truck_time, drone_time).

    Makespan:
        Sum of synchronized stage durations.

    Feasibility:
        Each drone sub-route must have drone_time <= battery_capacity.
    """
    total_makespan = 0
    feasible = True
    battery_violations = []
    edge_times = []

    truck_route = solution.truck_route
    drone_assignments = solution.drone_assignments

    for i in range(len(truck_route) - 1):
        start_node = truck_route[i]
        end_node = truck_route[i + 1]
        edge = (start_node, end_node)

        drone_nodes = drone_assignments.get(edge, [])

        truck_duration = compute_truck_edge_time(
            start_node,
            end_node,
            truck_time
        )

        drone_duration = compute_drone_edge_time(
            start_node,
            drone_nodes,
            end_node,
            drone_time
        )

        stage_duration = max(truck_duration, drone_duration)
        total_makespan += stage_duration

        if drone_duration > battery_capacity:
            feasible = False
            battery_violations.append({
                "edge": edge,
                "drone_nodes": drone_nodes,
                "drone_duration": drone_duration,
                "battery_capacity": battery_capacity,
                "excess": drone_duration - battery_capacity
            })

        edge_times.append({
            "edge": edge,
            "truck_duration": truck_duration,
            "drone_duration": drone_duration,
            "stage_duration": stage_duration,
            "drone_nodes": drone_nodes
        })

    solution.makespan = total_makespan
    solution.feasible = feasible
    solution.battery_violations = battery_violations
    solution.edge_times = edge_times

    return solution


def print_solution_report(solution):
    """
    Prints a readable report for debugging and presentation.
    """
    print("\n================ SOLUTION REPORT ================")

    print("\nTruck / Synchronization Route:")
    print(solution.truck_route)

    print("\nEdge Details:")
    for item in solution.edge_times:
        edge = item["edge"]
        drone_nodes = item["drone_nodes"]
        truck_duration = item["truck_duration"]
        drone_duration = item["drone_duration"]
        stage_duration = item["stage_duration"]

        print(
            f"{edge} | Drone nodes: {drone_nodes} | "
            f"Truck: {truck_duration:.3f} | "
            f"Drone: {drone_duration:.3f} | "
            f"Stage: {stage_duration:.3f}"
        )

    print("\nMakespan:")
    print(f"{solution.makespan:.3f}")

    print("\nFeasible:")
    print(solution.feasible)

    if solution.battery_violations:
        print("\nBattery Violations:")
        for violation in solution.battery_violations:
            print(violation)

    print("=================================================\n")