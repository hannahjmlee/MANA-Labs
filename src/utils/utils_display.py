import matplotlib.pyplot as plt
from src.utils.display import *
from src.utils.distance_metrics import *


def DisplayLineOfSight(positions, los_grids, grid, output_name, scenario_range = []):
    bounds, obstacles = PreprocessMap(grid)
    fig, ax = plt.subplots(figsize = (8,8))

    ax.clear()
    ax.set_aspect('equal')
    ax.autoscale(False)
    ax.set_xlim(bounds[0][0] - 0.5, bounds[0][1] - 0.5)
    ax.set_ylim(bounds[1][0] - 0.5, bounds[1][1] - 0.5)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')

    fig_width, _ = fig.get_size_inches() * fig.dpi
    grid_cell_size = fig_width / bounds[0][1]
    marker_size = (grid_cell_size / fig.dpi) * 72

    for obstacle in obstacles:
        plt.fill_between([obstacle[0] - 0.5, obstacle[0] + 0.5],
                        obstacle[1] - 0.5, obstacle[1] + 0.5,
                        color='black')

    for los_grid in los_grids:
        rows, cols = los_grid.shape
        for x in range(cols):
            for y in range(rows):
                if los_grid[(y, x)] == True:
                    plt.fill_between([x - 0.5, x + 0.5], y-0.5, y + 0.5, color='#73D65C')

    for i, pos in enumerate(positions):
        ax.plot(pos[0], pos[1], 'o', color='#00023D', markersize = marker_size * 0.5)
        if len(scenario_range) > 0: 
            ax.text(pos[0], pos[1], f"{scenario_range[i]}", fontsize=10, color='red')

    plt.savefig(output_name)

def DisplayProximity(positions, grid, limit, output_name, scenario_range = []):
    bounds, obstacles = PreprocessMap(grid)
    fig, ax = plt.subplots(figsize = (8,8))

    ax.clear()
    ax.set_aspect('equal')
    ax.autoscale(False)
    ax.set_xlim(bounds[0][0] - 0.5, bounds[0][1] - 0.5)
    ax.set_ylim(bounds[1][0] - 0.5, bounds[1][1] - 0.5)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')


    fig_width, _ = fig.get_size_inches() * fig.dpi
    grid_cell_size = fig_width / bounds[0][1]
    marker_size = (grid_cell_size / fig.dpi) * 72

    for i, pos in enumerate(positions):
        x, y = pos

        xbound_min = max(0, x-limit)
        xbound_max = min(bounds[0][1], x + limit)

        ybound_min = max(0, y - limit)
        ybound_max = min(bounds[1][1], y + limit)

        for x in range(xbound_min, xbound_max + 1):
            for y in range(ybound_min, ybound_max + 1):
                if EuclideanDistance((x, y), pos) <= limit:
                    plt.fill_between([x - 0.5, x + 0.5], y-0.5, y + 0.5, color='#73D65C')

        ax.plot(pos[0], pos[1], 'o', color='#00023D', markersize = marker_size * 0.5)
        if len(scenario_range) > 0: 
            ax.text(pos[0], pos[1], f"{scenario_range[i]}", fontsize=10, color='red')

    for obstacle in obstacles:
        plt.fill_between([obstacle[0] - 0.5, obstacle[0] + 0.5],
                        obstacle[1] - 0.5, obstacle[1] + 0.5,
                        color='black')

    plt.savefig(output_name)


def DisplayEnvironment(grid, map_name, output_name):
    bounds, obstacles = PreprocessMap(grid)
    fig, ax = plt.subplots(figsize = (8,8))

    ax.clear()
    ax.set_aspect('equal')
    ax.autoscale(False)
    ax.set_xlim(bounds[0][0] - 0.5, bounds[0][1] - 0.5)
    ax.set_ylim(bounds[1][0] - 0.5, bounds[1][1] - 0.5)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')

    for obstacle in obstacles:
        plt.fill_between([obstacle[0] - 0.5, obstacle[0] + 0.5],
                        obstacle[1] - 0.5, obstacle[1] + 0.5,
                        color='black')
    # ax.set_title(map_name)
    plt.savefig(output_name)
