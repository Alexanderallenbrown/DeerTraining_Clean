from numpy import *
from matplotlib import *
### CREATE ANIMATION
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation

agent = 'D'
mapa = 'constant_4'
xCar = 0
setSpeed = 25
generation = 1
ID = 101
trial = 1



Gfname = 'GenerationFiles/generations' + str(agent) + '/map_' + str(mapa) + '/xCar' + str(xCar) + '/setSpeed' + str(setSpeed) + '/trialData/generation' + str(generation) + 'ID_' + str(ID) + '/trial_' + str(trial) + '.txt';
#Gfname = 'GenerationFiles/TestTest.txt'


print Gfname

#should have an array of size m*h (of object values )

#in some way, read in a text file to fill an array
# Create car vectors to be used
car_x = []
car_y = []
car_yaw = []

# Create deer vectors to be used
deer_x = []
deer_y = []
deer_yaw = []

with open(Gfname, "r") as ins:
    for line in ins:
        values = line.split()
        #print values
        #print values
        car_x.append(float(values[5]))
        car_y.append(float(values[3]))
        car_yaw.append(float(values[7]))
        deer_x.append(float(values[17]))
        deer_y.append(float(values[18]))
        deer_yaw.append(float(values[16]))

print deer_x
print deer_y


plt.figure()
plt.plot(car_x,car_y,'ko',deer_x,deer_y,'ro')




x_car_init = 40
fakemap = 'real_tree_wismer'

# Define parameters
deer_length = 1.5 # meters
deer_width = 0.5 # meters
car_length = 4.5 # meters
car_width = 2.0 # meters



# Create figure
factor = 3
fig = plt.figure(figsize=(6.4*factor,4.8*factor))
ax = fig.add_subplot(111)
plt.axis('equal')
ax.set_xlim(x_car_init, x_car_init + 100)
ax.set_ylim(-25, 25)

# Initialize rectangles
background = patches.Rectangle((-100,-100),10000,10000,fc='g')
car_circle = patches.Circle((100,0),60,fc = 'w', alpha = 0.25)
car_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='b', alpha = 0.5)
deer_plot = patches.Rectangle((0, 0), 0, 0,angle = 0.0, fc='g', alpha = 0.5)
road = patches.Rectangle((-100,-3.75),10000,11,fc='k')
center_line_1 = patches.Rectangle((-10,(1.75+0.025)),10000,0.1,fc='y')
center_line_2 = patches.Rectangle((-10,(1.75-0.1-0.025)),10000,0.1,fc='y')
right_line = patches.Rectangle((-10,-1.75),10000,0.1,fc='w')
left_line = patches.Rectangle((-10,4.75),10000,0.1,fc='w')

# car_pred =[]
# deer_pred =[]
# for k in range(0,5):
#     car_pred.append(patches.Rectangle((0, 0), car_length, car_width,angle = 0.0, fc='w', alpha = .1))
#     deer_pred.append(patches.Rectangle((0, 0), deer_length, deer_width,angle = 0.0, fc='w', alpha = .1))

if fakemap == 'real_tree_wismer':
    trees = patches.Rectangle((100,-94),80,90,fc='g')

def init():
    ax.add_patch(background)
    ax.add_patch(car_circle)
    if fakemap == 'real_tree_wismer':
        ax.add_patch(trees)
    ax.add_patch(road)
    ax.add_patch(car_plot)
    ax.add_patch(deer_plot)
    ax.add_patch(left_line)
    ax.add_patch(right_line)
    ax.add_patch(center_line_1)
    ax.add_patch(center_line_2)

    # for k in range(0,5):
    #     ax.add_patch(car_pred[k])
    #     ax.add_patch(deer_pred[k])            

    if fakemap == 'real_tree_wismer':
        return car_circle,trees,car_plot,deer_plot,#car_pred[0],car_pred[1],car_pred[2],car_pred[3],car_pred[4],deer_pred[0],deer_pred[1],deer_pred[2],deer_pred[3],deer_pred[4],#car_pred[5],#car_pred[6],car_pred[7],car_pred[8],car_pred[9],
    else:
        return car_circle,car_plot,deer_plot,#car_pred[0],car_pred[1],car_pred[2],car_pred[3],car_pred[4],deer_pred[0],deer_pred[1],deer_pred[2],deer_pred[3],deer_pred[4],#car_pred[5],#car_pred[6],car_pred[7],car_pred[8],car_pred[9],

# Set animation
def animate(i):
    car_circle.center = (car_x[i],car_y[i])

    if fakemap == 'real_tree_wismer':
        trees.set_width(90)


    car_plot.set_width(car_length)
    car_plot.set_height(car_width)
    car_plot.set_xy([car_x[i]-(car_length/2), car_y[i]-(car_width/2)])
    car_plot.angle = car_yaw[i]*180/3.14

    deer_plot.set_width(deer_length)
    deer_plot.set_height(deer_width)
    deer_plot.set_xy([deer_x[i]-(deer_length/2*sin(deer_yaw[i])), deer_y[i]]-(deer_width/2*cos(deer_yaw[i])))
    deer_plot.angle = 90-deer_yaw[i]*180/3.14
    # if deer_visible[i] == 1.0:
    #     deer_plot.set_color('r') 
    # else:
    #     deer_plot.set_color('y')


    # for k in range(0,5):
    #     #print psiDeerPred[i,:]
    #     car_pred[k].set_xy([xCarPred[i,2*k]-(car_length/2),yCarPred[i,2*k]-(car_width/2)])
    #     car_pred[k].angle = psiCarPred[i,2*k]*180/3.14
    #     deer_pred[k].set_xy([xDeerPred[i,2*k]-(deer_length/2*sin(deer_yaw[i])),yDeerPred[i,2*k]-(deer_width/2*cos(deer_yaw[i]))])
        # deer_pred[k].angle = 90-psiDeerPred[i,2*k]*180/3.14


    if fakemap == 'real_tree_wismer':
        return car_circle,trees,car_plot,deer_plot,#car_pred[0],car_pred[1],car_pred[2],car_pred[3],car_pred[4],deer_pred[0],deer_pred[1],deer_pred[2],deer_pred[3],deer_pred[4],#car_pred[5],#car_pred[6],car_pred[7],car_pred[8],car_pred[9],   
        
    else:
        return car_circle,car_plot,deer_plot,#car_pred[0],car_pred[1],car_pred[2],car_pred[3],car_pred[4],deer_pred[0],deer_pred[1],deer_pred[2],deer_pred[3],deer_pred[4],#car_pred[5],#car_pred[6],car_pred[7],car_pred[8],car_pred[9],

# Run anumation
anim = animation.FuncAnimation(fig, animate,init_func=init,frames=len(car_x),interval=20,blit=True)
Writer = animation.writers['ffmpeg']
writer = Writer(fps=60,metadata=dict(artist='ME'),bitrate=1800*4)
anim.save('trial_animation.mp4')

### ANIMATION END

plt.show()



