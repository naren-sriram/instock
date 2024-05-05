import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def parse_args(args):
    arg_dict = {}
    for i in range(1, len(args), 2):
        if i + 1 < len(args):
            arg_dict[args[i]] = args[i + 1]
    return arg_dict

def read_map(filename):
    with open(filename, 'r') as file:
        size = int(file.readline().strip())
        board = np.zeros((size, size), dtype=int)
        for line in file:
            x, y = map(int, line.strip().split(','))
            board[y, x] = 1  # Mark blocked cells
    return board, size

def read_kings(filename):
    with open(filename, 'r') as file:
        return sum(1 for _ in file)  # Count lines in the file to determine number of kings

def read_trajectories(filename, num_kings):
    trajectories = [[] for _ in range(num_kings)]
    with open(filename, 'r') as file:
        lines = [line.strip() for line in file if 'Planning time' not in line and line.strip()]
        for i, line in enumerate(lines):
            if not line or 'Planning time' in line or any(kw in line for kw in ['iterations', 'nodes', 'Statistics', 'Cost']):
                continue
            coords = line.split(',')
            if len(coords) < 4:
                continue
            index = i % num_kings
            x_start, y_start, x_end, y_end = map(int, coords[:4])
            if not trajectories[index]:  # Check if this is the first append for this king
                trajectories[index].append((x_start, y_start))  # Add the starting position
            trajectories[index].append((x_end, y_end))  # Add the ending position
    return trajectories

def visualize(board, trajectories, size):
    fig, ax = plt.subplots()
    colors = plt.cm.get_cmap('Set1', len(trajectories))  # distinct colors for each king

    ax.set_xticks(np.arange(-.5, size, 1), minor=True)
    ax.set_yticks(np.arange(-.5, size, 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=2)
    ax.set_xticks(np.arange(size))
    ax.set_yticks(np.arange(size))
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    background = np.zeros((size, size, 3))
    for x in range(size):
        for y in range(size):
            if board[y, x] == 1:
                background[y, x] = [1, 0, 0]  # Red for blocked cells
            else:
                background[y, x] = [0.9, 0.9, 0.9]  # Light grey for open cells

    img = ax.imshow(background, interpolation='none', extent=[-0.5, size-0.5, size-0.5, -0.5])

    scatter_plots = [ax.plot([], [], 'o', color=colors(i), markersize=15)[0] for i in range(len(trajectories))]

    def init():
        for scatter_plot in scatter_plots:
            scatter_plot.set_data([], [])
        return scatter_plots

    def update(frame):
        num_kings = len(trajectories)
        for i, traj in enumerate(trajectories):
            if frame > 0:  # Start moving kings after the initial frame
                move_index = (frame - 1) % num_kings  # Determine which king moves in this frame
                if i == move_index:  # Only update the current king's position
                    step = (frame - 1) // num_kings  # Calculate the step for the current king
                    if step < len(traj):
                        x, y = traj[step]
                        scatter_plots[i].set_data(x, y)  # Adjusted to directly use x, y
                    else:
                        # Keep the king in the last known position if no more moves are left
                        x, y = traj[-1]
                        scatter_plots[i].set_data(x, y)
            else:
                # Initial frame, all kings are stationary at their start positions
                x, y = traj[0]
                scatter_plots[i].set_data(x, y)  # Directly use x, y
        return scatter_plots

    ani = animation.FuncAnimation(fig, update, frames=(num_kings * max(len(traj) for traj in trajectories) + 1), init_func=init, blit=True, interval=500, repeat_delay=1000)
    # ani.save('solution_10.gif', writer='imagemagick', fps=2)
    plt.show()

if __name__ == "__main__":
    args = parse_args(sys.argv)
    map_file = args.get('input_map', "/home/naren/instock/problem-tests/1/map.txt")
    kings_file = args.get('input_kings', "/home/naren/instock/problem-tests/1/kings.txt")
    solution_file = args.get('solution', "solution_1.txt")

    board, size = read_map(map_file)
    num_kings = read_kings(kings_file)
    trajectories = read_trajectories(solution_file, num_kings)
    visualize(board, trajectories, size)
