import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
import numpy as np
from src.utils.display import GetColors

def Animate(t, states, tasks, bounds, obstacles, frame_rate, output_name):
    np.random.seed(0)
    rand_colors = GetColors()
    state_keys = list(states.keys())
    state_keys.sort()
    colors = dict()
    for i, id, in enumerate(state_keys): 
        colors[id] = rand_colors[i]

    fig, ax = plt.subplots(figsize = (8, 8))
    
    ax.set_xlim(bounds[0][0] - 0.5, bounds[0][1] - 0.5)
    ax.set_ylim(bounds[1][0] - 0.5, bounds[1][1] - 0.5)

    fig_width, _ = fig.get_size_inches() * fig.dpi
    grid_cell_size = fig_width / bounds[0][1]
    marker_size = (grid_cell_size / fig.dpi) * 72

    def update(frame):
        ax.clear()
        ax.set_aspect('equal')
        ax.autoscale(False)
        ax.set_xlim(bounds[0][0] - 0.5, bounds[0][1] - 0.5)
        ax.set_ylim(bounds[1][0] - 0.5, bounds[1][1] - 0.5)
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_title(f'Agent Positions at Time: {t[frame]:.2f}')
        # ax.grid()

        for obstacle in obstacles:
            plt.fill_between([obstacle[0] - 0.5, obstacle[0] + 0.5],
                         obstacle[1] - 0.5, obstacle[1] + 0.5,
                         color='black')

        for i, paths in states.items(): # states maps agents to a list of paths
            if frame >= len(paths):
                ax.plot(tasks[i][1][0], tasks[i][1][1], 'o',
                    color=colors[i], markersize=marker_size*0.5,
                    label=f'Task {i + 1}')
                
                ax.plot(tasks[i][1][0], tasks[i][1][1], 'x',
                    color="black", markersize=marker_size*0.35,
                    label=f'Task {i + 1}')
                continue

            # plot path
            if len(paths) > 1:
                for j in range(len(paths[frame]) - 1): 
                    x1, y1 = paths[frame][j]
                    x2, y2 = paths[frame][j + 1]

                    ax.plot([x1, x2], [y1, y2], color=colors[i], 
                            linewidth=marker_size*0.25, alpha = 0.4)

            position = paths[frame][0]            
            # plot agent position and goal 
            ax.plot(tasks[i][1][0], tasks[i][1][1], 'X',
                    color=colors[i], markersize=marker_size*0.35,
                    label=f'Task {i + 1}')
            ax.plot(position[0], position[1], 'o',
                    color=colors[i], markersize=marker_size*0.48,
                    label=f'Robot {i + 1}')

    print("Generating Animation...")
    ani = FuncAnimation(fig, update, frames=len(t), repeat=False)

    print("Saving Video at " + str(frame_rate) + " fps...")
    FFwriter = FFMpegWriter(fps=frame_rate)
    ani.save(output_name, writer=FFwriter)


def InterpolatePath(path, num_intermediates): 
    interpolated_path = []

    for i in range(len(path) - 1): 
        x1, y1 = path[i]
        x2, y2 = path[i + 1]

        interpolated_path.append(path[i:])

        for j in range(1, num_intermediates + 1): 
            if x1 == x2 and y1 == y2: 
                interpolated_path.append(path[i:])
                continue
        
            alpha = j / (num_intermediates + 1)
            interp_x = x1 + alpha * (x2 - x1)
            interp_y = y1 + alpha * (y2 - y1)

            new_path = [(interp_x, interp_y)] + path[i+1:]
            interpolated_path.append(new_path)

    interpolated_path.append([path[-1]])
    return interpolated_path




def PreprocessAnimation(solution, num_intermediates=3):
    states = dict()
    max_time = 0
    for agent, path in solution.items(): 
        states[agent] = InterpolatePath(path, num_intermediates)
        max_time = max(max_time, len(path))
    
    time = np.linspace(0, max_time, max_time * (num_intermediates + 1) + 1)

    return time, states


