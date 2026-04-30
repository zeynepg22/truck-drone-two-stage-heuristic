import csv

from src.config import TRUCK_SPEED, DRONE_SPEED, BATTERY_CAPACITY
from src.data_loader import load_nodes
from src.distance_matrix import compute_distance_matrix, compute_time_matrix
from src.hybrid_solver import solve_two_stage_hybrid
from src.evaluator import print_solution_report
from src.visualization import plot_solution, plot_history


def save_results_csv(result, output_path="results/results.csv"):
    """
    Saves final summary result to CSV.
    """
    best_solution = result["best_solution"]

    with open(output_path, mode="w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow([
            "method",
            "makespan",
            "feasible",
            "runtime_seconds",
            "truck_route",
            "drone_assignments"
        ])

        writer.writerow([
            "Two-Stage Hybrid Heuristic",
            round(best_solution.makespan, 4),
            best_solution.feasible,
            round(result["runtime"], 4),
            best_solution.truck_route,
            best_solution.drone_assignments
        ])


def main():
    nodes_df = load_nodes("data/nodes.csv")

    distance_matrix = compute_distance_matrix(nodes_df)
    truck_time = compute_time_matrix(distance_matrix, TRUCK_SPEED)
    drone_time = compute_time_matrix(distance_matrix, DRONE_SPEED)

    start_node = int(nodes_df[nodes_df["type"] == "origin"]["node_id"].iloc[0])
    end_node = int(nodes_df[nodes_df["type"] == "destination"]["node_id"].iloc[0])
    all_nodes = nodes_df["node_id"].tolist()

    result = solve_two_stage_hybrid(
        truck_time=truck_time,
        drone_time=drone_time,
        all_nodes=all_nodes,
        start_node=start_node,
        end_node=end_node,
        battery_capacity=BATTERY_CAPACITY,
        max_iterations=300,
        initial_temperature=10.0,
        cooling_rate=0.995,
        keep_every=2
    )

    best_solution = result["best_solution"]

    print("\nInitial Full Truck Route:")
    print(result["initial_full_truck_route"])

    print_solution_report(best_solution)

    save_results_csv(result)

    plot_solution(nodes_df, best_solution)
    plot_history(result["history"])

    print("Results saved to results/results.csv")
    print("Solution plot saved to results/solution_plot.png")
    print("Convergence plot saved to results/convergence_plot.png")


if __name__ == "__main__":
    main()