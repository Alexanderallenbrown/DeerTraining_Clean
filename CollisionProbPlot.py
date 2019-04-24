from numpy import *
from matplotlib.pyplot import *

mapa = 'nothing'

agent = 'D'

xCar = 0
setSpeed = 25

agent_type = "D"

genomeFileName = 'GenerationFiles/TestGenomes/agent_' + str(agent) + '/map_' + str(mapa)  +  '/setSpeed' + str(setSpeed) + '/genome.txt'
resultFileName = 'GenerationFiles/TestGenomes/agent_' + str(agent) + '/map_' + str(mapa)  +  '/setSpeed' + str(setSpeed) + '/results.txt'

trialNum = []
IQM = []
collisions = []

with open(resultFileName, "r") as ins:
    for line in ins:
        values = line.split()
        trialNum.append(float(values[0]))
        IQM.append(float(values[1]))
        collisions.append(float(values[2]))

print(trialNum)
print(IQM)
print(collisions)

collisionProb = sum(collisions)/len(collisions)

figure()
hist(IQM)
show()
print(collisionProb)
