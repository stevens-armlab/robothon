import numpy as np
if __name__ == '__main__':
    path = []
    point = np.array([0,0])
    moves = [[0, -1], [1, 0], [0, 1], [-1, 0]]
    num_repeats = 1
    num_points_to_add = 289
    move_idx = 0
    path.append(point.copy())
    while len(path) < num_points_to_add:
        move_idx = move_idx % 4
        for j in range(num_repeats):
            point += moves[move_idx]
            path.append(point.copy())

        if move_idx == 1 or move_idx == 3:
            num_repeats += 1
        move_idx += 1

    print(path)
