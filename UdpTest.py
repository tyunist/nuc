#This program tests whether no of sample on server and client are the same and
# there is any sample that are > 0.1
import matplotlib.pyplot as plt
import re
currSample = []
circleTime = []
circleTime2 = []
outList = []
outList2 = []
count = 0 # count > 1.1 intervals
count2 = 0
sum = 0
sum2 = 0
with open('velResponse.csv','r+') as f:
	next(f) # Skip the header line
	for line in f:
		temp = map(float, re.findall(r'\d+', line)) #find all number from line
		currSample.append(float(line.split('\t')[0]))
		circleTime.append(float(line.split('\t')[11]))
		circleTime2.append(float(line.split('\t')[12]))

for i in range(len(circleTime)):
	#print('CircleTime1-',i)
	#print(circleTime[i])
	sum = sum + circleTime[i]
	if(circleTime[i] > 0.11):
		outList.append(circleTime[i])
 		count = count + 1
for i in range(len(circleTime2)):
	#print('CircleTime2 -',i)
	#print(circleTime2[i])
	sum2 = sum2 + circleTime2[i]
	if(circleTime2[i] > 0.11):
		outList2.append(circleTime2[i])
		count2 = count2 + 1
		print(i, circleTime2[i])
x = list(range(len(outList)))
x2 = list(range(len(outList2)))
plt.figure(1)                # the first figure


#New trial
plt.subplot(2,1,1)
plt.plot(x, outList, 'r', x2, outList2, 'b')
plt.title('OutList')
plt.xlabel('Samples')
plt.ylabel('Time')
plt.axis([0, 100, 0, 1.0])
plt.grid(True)

print("No off > 0.11 of circleTime1: ")
print(count)
for i in range(len(x)):
	print(outList[i])

print("No off > 0.11 of circleTime2:  ")
print(count2)
print("They are: ")
for i in range(len(x2)):
	print(outList2[i])
print('Average circleTim1:', sum/len(circleTime))
print('Average circleTim2:', sum2/len(circleTime2))

plt.show()
