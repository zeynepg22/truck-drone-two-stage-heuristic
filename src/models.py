class Solution:
    """
    Represents a two-stage truck-drone solution.

    truck_route:
        Ordered list of truck/synchronization nodes.

    drone_assignments:
        Dictionary where each key is a truck edge (A, B)
        and each value is a list of drone-only customers served between A and B.
    """

    def __init__(self, truck_route, drone_assignments):
        self.truck_route = truck_route
        self.drone_assignments = drone_assignments

        self.makespan = None
        self.feasible = None
        self.battery_violations = []
        self.edge_times = []

    def __repr__(self):
        return (
            f"Solution("
            f"truck_route={self.truck_route}, "
            f"makespan={self.makespan}, "
            f"feasible={self.feasible}"
            f")"
        )