fig = plt.figure(figsize=(8,8), dpi=100)
ax = fig.add_subplot(111, aspect='equal')

image = mpimg.imread('C:/Users/tatia/Desktop/UFMG/RM/TP2/mapas/imagens/cave.png')
map_dimension = np.array([35,35])
x_scale, y_scale = image.shape[:2] / map_dimension

cell_size = 0.6
rows, cols = (map_dimension / cell_size).astype(int)
grid = np.zeros((rows, cols))
ax.imshow(image, cmap='Greys', extent=(0, map_dimension[1], 0, map_dimension[0]), origin='upper')

ax.grid(which='major', axis='both', linestyle='-', color='r', linewidth=1)
ax.set_xticks(np.arange(0, map_dimension[1], cell_size))
ax.set_yticks(np.arange(0, map_dimension[0], cell_size))