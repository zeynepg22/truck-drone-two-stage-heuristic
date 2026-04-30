from src.config import TRUCK_SPEED, DRONE_SPEED, BATTERY_CAPACITY
from src.data_loader import load_nodes
from src.distance_matrix import compute_distance_matrix, compute_time_matrix
from src.initial_solution import create_initial_truck_route
from src.drone_scheduler import create_drone_schedule
from src.models import Solution
from src.evaluator import evaluate_solution, print_solution_report


def main():
    nodes_df = load_nodes("data/nodes.csv")

    distance_matrix = compute_distance_matrix(nodes_df)
    truck_time = compute_time_matrix(distance_matrix, TRUCK_SPEED)
    drone_time = compute_time_matrix(distance_matrix, DRONE_SPEED)

    start_node = int(nodes_df[nodes_df["type"] == "origin"]["node_id"].iloc[0])
    end_node = int(nodes_df[nodes_df["type"] == "destination"]["node_id"].iloc[0])
    all_nodes = nodes_df["node_id"].tolist()

    truck_route, full_truck_route = create_initial_truck_route(
        truck_time,
        start_node,
        end_node,
        keep_every=2
    )

    truck_route, drone_assignments = create_drone_schedule(
        truck_route,
        all_nodes,
        drone_time,
        BATTERY_CAPACITY
    )

    solution = Solution(
        truck_route=truck_route,
        drone_assignments=drone_assignments
    )

    solution = evaluate_solution(
        solution,
        truck_time,
        drone_time,
        BATTERY_CAPACITY
    )

    print("\nFull Truck Route Before Reduction:")
    print(full_truck_route)

    print_solution_report(solution)


if __name__ == "__main__":
    main()