import matplotlib.pyplot as plt


def plot_solution(nodes_df, solution, output_path="results/solution_plot.png"):
    """
    Plots truck route and drone sub-routes.

    Truck route is drawn as connected line.
    Drone sub-routes are drawn between synchronization nodes and drone customers.
    """
    plt.figure(figsize=(10, 7))

    x_values = nodes_df["x"].tolist()
    y_values = nodes_df["y"].tolist()
    labels = nodes_df["node_id"].tolist()

    plt.scatter(x_values, y_values)

    for _, row in nodes_df.iterrows():
        plt.text(row["x"] + 0.5, row["y"] + 0.5, str(row["node_id"]))

    # Plot truck route
    truck_route = solution.truck_route

    for i in range(len(truck_route) - 1):
        a = truck_route[i]
        b = truck_route[i + 1]

        ax = nodes_df.loc[nodes_df["node_id"] == a, "x"].iloc[0]
        ay = nodes_df.loc[nodes_df["node_id"] == a, "y"].iloc[0]
        bx = nodes_df.loc[nodes_df["node_id"] == b, "x"].iloc[0]
        by = nodes_df.loc[nodes_df["node_id"] == b, "y"].iloc[0]

        plt.plot([ax, bx], [ay, by], linewidth=2)

    # Plot drone routes
    for edge, drone_nodes in solution.drone_assignments.items():
        if not drone_nodes:
            continue

        full_drone_path = [edge[0]] + drone_nodes + [edge[1]]

        for i in range(len(full_drone_path) - 1):
            a = full_drone_path[i]
            b = full_drone_path[i + 1]

            ax = nodes_df.loc[nodes_df["node_id"] == a, "x"].iloc[0]
            ay = nodes_df.loc[nodes_df["node_id"] == a, "y"].iloc[0]
            bx = nodes_df.loc[nodes_df["node_id"] == b, "x"].iloc[0]
            by = nodes_df.loc[nodes_df["node_id"] == b, "y"].iloc[0]

            plt.plot([ax, bx], [ay, by], linestyle="--", linewidth=1)

    plt.title(
        f"Truck-Drone Solution | Makespan = {solution.makespan:.3f}"
    )
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.grid(True)
    plt.savefig(output_path)
    plt.close()


def plot_history(history, output_path="results/convergence_plot.png"):
    """
    Plots best makespan over iterations.
    """
    iterations = [item["iteration"] for item in history]
    best_values = [item["best_makespan"] for item in history]

    plt.figure(figsize=(10, 5))
    plt.plot(iterations, best_values)
    plt.title("Convergence Plot")
    plt.xlabel("Iteration")
    plt.ylabel("Best Makespan")
    plt.grid(True)
    plt.savefig(output_path)
    plt.close()