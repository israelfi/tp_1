import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker


def read_file():
    with open('tangent_bug.txt', 'r') as arquivo:
        linhas = arquivo.readlines()
        eixo_x = []
        eixo_y = []
        for item in linhas:
            eixo_x.append(float(item.split('\t')[0]) + offset)
            eixo_y.append(float(item.split('\t')[1]) + offset)

    return eixo_x, eixo_y


if __name__ == '__main__':
    offset = 0.

    x, y = read_file()
    x = np.array(x)
    y = np.array(y)

    fig, ax = plt.subplots()
    ax.plot(x, y, 'r-', label='odom')
    plt.xlabel('x')
    plt.ylabel('y')

    x_goal = 30
    y_goal = 18

    plt.plot(x_goal + offset, y_goal + offset, color='green', marker='x')
    plt.annotate('goal', (x_goal-0.5 + offset, y_goal-3.5 + offset))

    img = plt.imread("../worlds/map_circle.bmp")
    img = img[10:76, 4:70, :]
    ax.imshow(img, extent=[-40, 40, -40, 40])



    plt.legend()

    # plt.grid(color='white', linestyle='-', linewidth=0.7)  # Grid settings
    ax.set_facecolor((0.9, 0.9, 0.9))  # Backgroung color
    loc = plticker.MultipleLocator(base=1.0)
    plt.style.use('ggplot')
    

    # plt.xticks(np.arange(min(x), max(x)+1, 1.0))
    # plt.gca().set_aspect('equal', adjustable='box')
    
    plt.show()
