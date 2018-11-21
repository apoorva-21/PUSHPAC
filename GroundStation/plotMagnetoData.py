import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
#to plot the data from the magnetometer's data file locally stored at the Ground Station
#run on a different terminal from that of the groundstation code that writes to the file.
style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

def animate(i):
    graph_data = open('magnetoPlot.txt','r').read()
    lines = graph_data.split('\n')
    xs = []
    ys = []
    for line in lines:
        if len(line) > 1:
            x, y = line.split(',')
            xs.append(x)
            ys.append(y)
    ax1.clear()
    ax1.plot(xs, ys)

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()
