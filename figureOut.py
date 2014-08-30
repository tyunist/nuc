#Version 1
import matplotlib.pyplot as plt
import re
currSample = []
vel = []
velEstimated = []
velSet = []
delayedSample = []
velAvr = []
#Testing numbers
velDelta = []
cmd = []

with open('velResponse.csv','r+') as f:
	next(f) # Skip the header line
	for line in f:
		temp = map(float, re.findall(r'\d+', line)) #find all number from line
		currSample.append(float(line.split('\t')[0]))
		delayedSample.append(float(line.split('\t')[0]) + 0.5)
		vel.append(float(line.split('\t')[2]))
		velEstimated.append(float(line.split('\t')[3]))
		velSet.append(float(line.split('\t')[1]))
		velDelta.append(float(line.split('\t')[13]))
		cmd.append(float(line.split('\t')[14])/5)


# Compute velAvr 
for i in range(len(velEstimated) ):
	if i>0 and i<len(velEstimated) - 1:
		velAvr.append( (velEstimated[i - 1] + velEstimated[i] + velEstimated[i+1])/3)
	else: 
		velAvr.append(velEstimated[i])


plt.figure(1)                # the first figure

print len(velAvr)

#New trial
plt.subplot(2,1,1)
plt.plot(currSample, velSet,'r', currSample, velEstimated, 'b', currSample, vel, 'm', currSample, velDelta, 'g', currSample, cmd,'y', currSample, velAvr, 'k')
plt.title('Velocity Responses')
plt.xlabel('time')
plt.ylabel('velocity reponses')
plt.axis([0, 10, -10, 10])
plt.grid(True)

xAxes = list(range(51))
for i in range(51):
	xAxes[i] = xAxes[i]*0.2
yAxes = list(range(-20, 21))
for i in range(41):
	yAxes[i] = yAxes[i] * 0.5
plt.xticks(xAxes) #stick sample times
plt.yticks(yAxes)

# VelAvarage

plt.subplot(2,1,2)
plt.plot(currSample, velSet,'r', currSample, cmd,'y', currSample, velAvr, 'k')
plt.title('Velocity Responses')
plt.xlabel('time')
plt.ylabel('velocity reponses')
plt.axis([0, 10, -10, 10])
plt.grid(True)
plt.xticks(xAxes) #stick sample times
plt.yticks(yAxes)

plt.show()
