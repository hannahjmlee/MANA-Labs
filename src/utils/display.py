import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from PIL import Image

def GetAnimationParams(size, decentralized=False): 
    # returns frame rate and number of intermediates
    # if decentralized: 
    # return 10, 0
    if size == 8: 
        return 30, 6
    elif size == 32: 
        return 20, 2
    elif size == 64: 
        return 20, 1
    elif size == 256 or size == 194:
        return 20, 0

    return 15, 0

def PreprocessMap(bool_grid):
    obstacles = []
    for x in range(bool_grid.shape[1]):
        for y in range(bool_grid.shape[0]):
            if not bool_grid[y][x]:
                obstacles.append((x, y))

    bounds = ((0, bool_grid.shape[1]), (0, bool_grid.shape[0]))

    return bounds, obstacles

def GetColors(): 
    colors = [[0.90378716, 0.40719089, 0.75530632],
              [0.05918246, 0.22558689, 0.33646525],
              [0.79661958, 0.81540406, 0.17251688],
              [0.75037329, 0.87947518, 0.84639111],
              [0.07799663, 0.41219122, 0.25893882],
              [0.82735448, 0.73251715, 0.52043857],
              [0.94785986, 0.66139572, 0.77998747],
              [0.52631452, 0.14093824, 0.40979095],
              [0.92975036, 0.18868095, 0.48641579],
              [0.16344739, 0.8982141 , 0.86538324],
              [0.92140347, 0.45975292, 0.07477697],
              [0.59272148, 0.38015801, 0.06065196],
              [0.97278199, 0.32131349, 0.24946137],
              [0.60568877, 0.75072088, 0.29643317],
              [0.05276841, 0.0380931 , 0.60558904],
              [0.95928097, 0.26444017, 0.73948965],
              [0.62355354, 0.92561875, 0.34898312],
              [0.81109441, 0.34897244, 0.75972108],
              [0.82080564, 0.94805139, 0.44150447],
              [0.47573309, 0.81454335, 0.85467776],
              [0.72033319, 0.39137803, 0.04087726],
              [0.74171654, 0.09418119, 0.86783153],
              [0.32098466, 0.9734774 , 0.48962649],
              [0.61456439, 0.29195851, 0.51693849],
              [0.37995168, 0.37462248, 0.771523  ],
              [0.78401489, 0.93839819, 0.99295815],
              [0.14461481, 0.49058616, 0.21598204],
              [0.6896863 , 0.19699802, 0.72449564],
              [0.99780827, 0.53475333, 0.19883522],
              [0.11092486, 0.01327042, 0.30048973],
              [0.45814537, 0.61936284, 0.85729164],
              [0.29871209, 0.9535194 , 0.53583922],
              [0.67106173, 0.42434676, 0.39200964],
              [0.52206164, 0.01290936, 0.94915772],
              [0.87013783, 0.84786319, 0.10210803],
              [0.43969358, 0.90414074, 0.84297592],
              [0.21010433, 0.57346002, 0.56184526],
              [0.85201887, 0.06446056, 0.49228854],
              [0.98870576, 0.08525161, 0.34842309],
              [0.90614023, 0.20527421, 0.04105141],
              [0.98880085, 0.69281337, 0.2419338 ],
              [0.256051  , 0.39322015, 0.96137753],
              [0.37381663, 0.91436242, 0.0669663 ],
              [0.58233275, 0.91612618, 0.21537491],
              [0.53571197, 0.21980604, 0.73322997],
              [0.43578558, 0.02476799, 0.89310218],
              [0.98511393, 0.12106962, 0.05629559],
              [0.02352971, 0.84041804, 0.66606146],
              [0.54466287, 0.64879132, 0.71850185],
              [0.99170493, 0.87659347, 0.4686561 ],
              [0.53963601, 0.69445658, 0.56093764],
              [0.25369752, 0.06578206, 0.4980165 ],
              [0.35806286, 0.28495234, 0.60718545],
              [0.19731901, 0.99878416, 0.76794204],
              [0.11001654, 0.68385966, 0.68779922],
              [0.146245  , 0.04970842, 0.25264178],
              [0.6615516 , 0.2708483 , 0.47682737],
              [0.20994923, 0.6115779 , 0.64498855],
              [0.38418399, 0.65744212, 0.74054896],
              [0.19187887, 0.12177265, 0.83796263],
              [0.42343167, 0.84145683, 0.97305586],
              [0.07593085, 0.26893468, 0.6025839 ],
              [0.7382199 , 0.08544637, 0.62431036],
              [0.79925791, 0.03027297, 0.12483029],
              [0.70596374, 0.33890506, 0.67264227],
              [0.84080325, 0.51540927, 0.46352717],
              [0.84007102, 0.36577635, 0.36165151],
              [0.15876452, 0.48694365, 0.00561707],
              [0.43384548, 0.65311137, 0.11844428],
              [0.9126813 , 0.20547202, 0.55263641],
              [0.2275427 , 0.5497798 , 0.96925355],
              [0.19759317, 0.08872532, 0.56289594],
              [0.8815061 , 0.98983452, 0.46491789],
              [0.32352346, 0.78375504, 0.91510018],
              [0.58723609, 0.75656741, 0.35454358],
              [0.67949405, 0.19421823, 0.39062045],
              [0.11630413, 0.25139782, 0.6559686 ],
              [0.27369033, 0.88694221, 0.94997962],
              [0.46414471, 0.24668996, 0.99791038],
              [0.35294198, 0.27810489, 0.30622135],
              [0.16501662, 0.62350865, 0.67375057],
              [0.00850956, 0.26916684, 0.92591146],
              [0.17672347, 0.0717823 , 0.05671704],
              [0.09590845, 0.84334355, 0.46223724],
              [0.69999439, 0.43270755, 0.83901513],
              [0.04021267, 0.85591329, 0.40530588],
              [0.04966279, 0.76503612, 0.04106269],
              [0.65842871, 0.96142438, 0.45837249],
              [0.49947021, 0.16425896, 0.13336146],
              [0.78171217, 0.10933412, 0.65607172],
              [0.87638274, 0.3045428 , 0.74325686],
              [0.23087515, 0.63142342, 0.86053114],
              [0.90019137, 0.11657967, 0.79719687],
              [0.85417011, 0.19856543, 0.19347984],
              [0.86450562, 0.01567449, 0.15982176],
              [0.33008224, 0.25562923, 0.6496478 ],
              [0.61031656, 0.82433248, 0.95355999],
              [0.71058766, 0.02629005, 0.51295719],
              [0.92635303, 0.24795016, 0.76917265],
              [0.11519947, 0.60873881, 0.99808794]]
    
    return colors