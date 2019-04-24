from numpy import *
from matplotlib.pyplot import *
import os

figure()
legendstr = []

xdir = '../GenerationFiles/FbData/OpenField_x0'

markers = ['ko-','ks-','k*-']

for expi, filename in enumerate(sorted(os.listdir(xdir))):

    dirname = xdir + '/' + filename

    legendstr.append('Vehicle Speed = ' + filename[-2:] + ' m/s')

    gennums = []
    mindists = []

    for geni, filename in enumerate(sorted(os.listdir(dirname))):
        print geni,filename
        sgennum = filename[10:-4]
        gennum = int(sgennum)
        thisgen = loadtxt(dirname+'/'+filename)
        mindist = mean(thisgen[:,1])
        gennums.append(gennum)
        mindists.append(mindist)

    gennums = array(gennums)
    mindists = array(mindists)

    inds = argsort(gennums)
    gennums = gennums[inds] 
    mindists = mindists[inds]

    ax = subplot(111)
    ax.plot(gennums,mindists,markers[expi],fillstyle='none',markersize=10)


    xlim([0,10])
    ylim([0,12])
    xlabel('Deer Model Genetic Algorithm Generation')
    ylabel('Average minimum distance per interaction (m)')

ax.plot([-1,21],[2,2],'r--')

legendstr.append('Collision threshold')
legend(legendstr)

savefig('Fb_mapdeer_generation_distance.pdf')
savefig('Fb_mapdeer_generation_distance.png')

show()
