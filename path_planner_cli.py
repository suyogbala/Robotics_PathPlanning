import argparse
from src.graph import GridGraph, Cell
from src.graph_search import a_star_search, breadth_first_search, depth_first_search
from src.utils import generate_plan_file


def parse_args():
    parser = argparse.ArgumentParser(description="HelloRob Path Planning Client.")
    parser.add_argument("-m", "--map", type=str, required=True, help="Path to the map file.")
    parser.add_argument("--start", type=int, nargs=2, required=True, help="Start cell.")
    parser.add_argument("--goal", type=int, nargs=2, required=True, help="Goal cell.")
    parser.add_argument("--algo", type=str, default="bfs", choices=["bfs", "dfs", "astar"],
                        help="Algorithm to use.")
    parser.add_argument("-r", "--collision-radius", type=float, default=0.15, help="Collision radius (meters).")

    args = parser.parse_args()

    return args


if __name__ == "__main__":
    args = parse_args()

    # Construct the graph.
    graph = GridGraph(args.map, collision_radius=args.collision_radius)
    # Construct the start and goal cells.
    start, goal = Cell(*args.start), Cell(*args.goal)

    # Run the planning algorithm of the user's choice.
    if args.algo == "astar":
        path = a_star_search(graph, start, goal)
    elif args.algo == "bfs":
        path = breadth_first_search(graph, start, goal)
        print(len(path))
    elif args.algo == "dfs":
        path = depth_first_search(graph, start, goal)
    else:
        print("Invalid option:", args.algo)
        exit()

    # Output the planning file for visualization.
    generate_plan_file(graph, start, goal, path)
