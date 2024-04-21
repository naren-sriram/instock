import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def read_map(filename):
    with open(filename, 'r') as file:
        size = int(file.readline().strip())
        board = np.zeros((size, size))
        for line in file:
            x, y = map(int, line.strip().split(','))
            board[x][y] = 1  # Mark blocked cells
    return board, size

def read_trajectories(filename):
    trajectories = []
    with open(filename, 'r') as file:
        for line in file:
            traj = []
            description = line.split(':')[0].strip().split(' ')
            print('description: ',description)
            if description[0][0] == 'P':
                continue
            positions = line.split(':')[1].strip().split(' ')
            for pos in positions:
                if pos:
                    x, y = map(int, pos.strip('()').split(','))
                    traj.append((x, y))
            trajectories.append(traj)
    return trajectories

def visualize(board, trajectories, size):
    fig, ax = plt.subplots()
    colors = plt.cm.get_cmap('Set1', len(trajectories))  # distinct colors for each king

    # Create a background grid
    ax.set_xticks(np.arange(-.5, size, 1), minor=True)
    ax.set_yticks(np.arange(-.5, size, 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=2)
    ax.set_xticks(np.arange(size))
    ax.set_yticks(np.arange(size))
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    # Set initial background colors
    background = np.zeros((size, size, 3))
    for x in range(size):
        for y in range(size):
            if board[x][y] == 1:
                background[x][y] = [1, 0, 0]  # Red for blocked cells
            else:
                background[x][y] = [0.9, 0.9, 0.9]  # Light grey for open cells
    img = ax.imshow(background, interpolation='none', extent=[-0.5, size-0.5, size-0.5, -0.5])

    scatter_plots = []
    for i in range(len(trajectories)):
        scatter_plot, = ax.plot([], [], 'o', color=colors(i), markersize=20)  # Marker size adjusted
        scatter_plots.append(scatter_plot)

    def init():
        for scatter_plot in scatter_plots:
            scatter_plot.set_data([], [])
        return scatter_plots

    def update(frame):
        for i, traj in enumerate(trajectories):
            if frame < len(traj):
                x, y = traj[frame]
                scatter_plots[i].set_data(y, x)
        return scatter_plots

    ani = animation.FuncAnimation(fig, update, frames=max(len(traj) for traj in trajectories), init_func=init, blit=True, interval=500, repeat_delay=1000)

    plt.show()

# Usage
board, size = read_map('problem-tests/3/map.txt')
trajectories = read_trajectories('trajectory_3.txt')
visualize(board, trajectories, size)
