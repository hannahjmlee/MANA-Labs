import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
import numpy as np

def Animate(t, states, goals, bounds, obstacles, all_tasks, completed_tracker, frame_rate, output_name):
    rand_colors = plt.cm.viridis(np.linspace(0, 1, len(states.keys())))
    colors = dict()
    for i, id in enumerate(states.keys()):
        colors[id] = rand_colors[i]


    fig, ax = plt.subplots(figsize = (8, 8))
    
    ax.set_xlim(bounds[0][0] - 0.5, bounds[0][1] - 0.5)
    ax.set_ylim(bounds[1][0] - 0.5, bounds[1][1] - 0.5)

    fig_width, _ = fig.get_size_inches() * fig.dpi
    grid_cell_size = fig_width / bounds[0][1]
    marker_size = (grid_cell_size / fig.dpi) * 72

    completed_starts = set()

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
                continue

            # plot path
            if len(paths) > 1:
                for j in range(len(paths[frame]) - 1): 
                    x1, y1 = paths[frame][j]
                    x2, y2 = paths[frame][j + 1]

                    ax.plot([x1, x2], [y1, y2], color=colors[i], 
                            linewidth=marker_size*0.25, alpha = 0.4)
                    

        for task in all_tasks: 
            if task in completed_tracker[frame]: 
                continue
            start, goal = task
            if start not in completed_starts: 
                ax.plot(start[0], start[1], '*',
                        color="#9c9c9c", markersize=marker_size*0.35,
                        label=f'Task {i + 1}')
            ax.plot(goal[0], goal[1], 'X',
                    color="#9c9c9c", markersize=marker_size*0.35,
                    label=f'Task {i + 1}')
            

        for i, paths in states.items(): # states maps agents to a list of paths
            if frame >= len(paths):
                g_idx = 1
                if len(goals[i][-1]) == 3: 
                    g_idx = 2

                ax.plot(goals[i][-1][g_idx][0], goals[i][-1][g_idx][1], 'o', 
                    color=colors[i], markersize=marker_size*0.5,
                    label=f'Task {i + 1}')
                
                ax.plot(goals[i][-1][g_idx][0], goals[i][-1][g_idx][1], 'x', 
                    color="black", markersize=marker_size*0.35,
                    label=f'Task {i + 1}')
                continue
            position = paths[frame][0]            
            # plot agent position and goal 
            if len(goals[i][frame]) == 2: 
                ax.plot(goals[i][frame][1][0], goals[i][frame][1][1], 'X',
                    color=colors[i], markersize=marker_size*0.35,
                    label=f'Task {i + 1}')
            else: 
                if goals[i][frame][0]: 
                    ax.plot(goals[i][frame][1][0], goals[i][frame][1][1], '*',
                        color=colors[i], markersize=marker_size*0.35,
                        label=f'Task {i + 1}')
                else: 
                    completed_starts.add(goals[i][frame][1])
                ax.plot(goals[i][frame][2][0], goals[i][frame][2][1], 'X',
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



def PreprocessAnimation(path_tracker, goal_tracker, completed_goals, num_intermediates=3):
    # paths maps agent ids to a list of paths. this list of paths needs to be 
    # interpolated by the first coordinate in each one. 

    states = dict()
    goals = dict()
    max_time = 0

    for agent, paths in path_tracker.items(): 
        max_time = max(max_time, len(paths))
        states[agent] = InterpolatePaths(paths, num_intermediates)

    for agent, tasks in goal_tracker.items(): 
        goals[agent] = ExpandGoals(tasks, num_intermediates)

    completed = ExpandGoals(completed_goals, num_intermediates)

    time = np.linspace(0, max_time, max_time * (num_intermediates + 1) + 1)

    return time, states, goals, completed

def InterpolatePaths(paths, num_intermediates): 
    interpolated_paths = []
    for i in range(len(paths) - 1): 
        x1, y1 = paths[i][0]
        x2, y2 = paths[i + 1][0]

        interpolated_paths.append(paths[i])
        for j in range(1, num_intermediates + 1): 
            if x1 == x2 and y1 == y2: 
                interpolated_paths.append(paths[i])
                continue
            alpha = j / (num_intermediates + 1)
            interp_x = x1 + alpha * (x2 - x1)
            interp_y = y1 + alpha * (y2 - y1)

            new_path = [(interp_x, interp_y)] + paths[i + 1]
            interpolated_paths.append(new_path)
    interpolated_paths.append(paths[-1])
    return interpolated_paths

def ExpandGoals(goals, num_intermediates): 
    tasks = []
    for goal in goals: 
        tasks.append(goal)
        for _ in range(1, num_intermediates + 1): 
            tasks.append(goal)
    tasks.append(goals[-1])

    return tasks
