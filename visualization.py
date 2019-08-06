import matplotlib.pyplot as plt
result = [[77.5, 80.0, 0], [0, 75.0, 0], [85.0, 77.5, 75.0], [82.5, 0, 80.0], [85.0, 97.5, 87.5], [85.0, 85.0, 80.0], [80.0, 95.0, 82.5], [75.0, 0, 0], [92.5, 95.0, 90.0], [80.0, 0, 0], [0, 77.5, 0], [0, 0, 0], [75.0, 80.0, 85.0], [80.0, 77.5, 0], [0, 75.0, 0], [0, 0, 0], [87.5, 0, 0], [92.5, 80.0, 87.5], [75.0, 85.0, 80.0], [0, 75.0, 0], [90.0, 90.0, 85.0], [85.0, 0, 0], [0, 0, 0], [0, 0, 0], [80.0, 87.5, 0], [0, 87.5, 0], [0, 77.5, 0], [0, 0, 0], [75.0, 0, 0], [77.5, 82.5, 80.0], [90.0, 90.0, 85.0], [95.0, 87.5, 90.0], [92.5, 90.0, 90.0], [0, 0, 0], [0, 0, 0], [75.0, 0, 0], [0, 0, 0], [75.0, 82.5, 75.0], [0, 75.0, 0], [90.0, 95.0, 85.0], [90.0, 90.0, 82.5], [0, 0, 0], [92.5, 80.0, 85.0], [75.0, 75.0, 0], [90.0, 90.0, 82.5], [85.0, 87.5, 82.5], [75.0, 80.0, 0], [95.0, 0, 0], [92.5, 92.5, 87.5], [82.5, 0, 0], [95.0, 97.5, 92.5], [92.5, 77.5, 0], [80.0, 77.5, 0], [0, 75.0, 0], [0, 75.0, 0], [0, 0, 0], [77.5, 0, 0], [0, 80.0, 0], [0, 80.0, 0], [0, 0, 0], [82.5, 87.5, 80.0], [0, 0, 0], [92.5, 80.0, 0], [92.5, 95.0, 85.0], [92.5, 92.5, 92.5], [87.5, 87.5, 0], [77.5, 80.0, 80.0], [0, 0, 0], [75.0, 75.0, 0], [0, 0, 0], [90.0, 92.5, 90.0], [95.0, 95.0, 90.0], [95.0, 75.0, 92.5], [0, 0, 0], [85.0, 80.0, 0], [0, 0, 0], [80.0, 80.0, 0], [0, 77.5, 0], [0, 80.0, 0], [90.0, 85.0, 85.0], [92.5, 95.0, 87.5], [85.0, 85.0, 82.5], [0, 75.0, 0], [77.5, 0, 0], [82.5, 0, 0], [80.0, 85.0, 0], [0, 82.5, 0], [92.5, 95.0, 87.5], [75.0, 75.0, 0], [0, 80.0, 0], [92.5, 82.5, 0], [80.0, 85.0, 0], [75.0, 97.5, 82.5], [75.0, 80.0, 0], [80.0, 82.5, 75.0], [87.5, 0, 82.5], [85.0, 0, 80.0], [77.5, 0, 0], [0, 80.0, 0], [80.0, 0, 0], [0, 75.0, 0], [92.5, 92.5, 77.5], [80.0, 0, 0], [75.0, 0, 0], [82.5, 0, 0], [80.0, 90.0, 82.5], [87.5, 95.0, 0], [92.5, 85.0, 82.5], [0, 75.0, 0], [97.5, 100.0, 95.0], [75.0, 80.0, 0], [0, 75.0, 0], [80.0, 85.0, 82.5], [0, 77.5, 0], [0, 0, 0], [75.0, 77.5, 0], [0, 77.5, 0], [92.5, 97.5, 87.5], [92.5, 92.5, 90.0], [82.5, 80.0, 75.0], [87.5, 90.0, 85.0], [0, 77.5, 0], [95.0, 100.0, 97.5], [0, 75.0, 0], [0, 0, 0], [75.0, 75.0, 75.0], [87.5, 80.0, 90.0], [0, 75.0, 0], [80.0, 0, 0], [85.0, 75.0, 0], [87.5, 85.0, 80.0], [82.5, 75.0, 80.0], [90.0, 90.0, 0], [80.0, 85.0, 75.0], [75.0, 80.0, 0], [85.0, 90.0, 87.5], [95.0, 92.5, 90.0], [0, 0, 0], [75.0, 0, 0], [92.5, 90.0, 90.0], [90.0, 95.0, 85.0], [95.0, 90.0, 85.0], [0, 75.0, 0], [75.0, 0, 0], [0, 0, 0], [75.0, 0, 77.5], [80.0, 0, 0], [90.0, 97.5, 92.5], [87.5, 87.5, 77.5], [75.0, 0, 0], [0, 0, 0], [75.0, 85.0, 0], [87.5, 77.5, 0], [0, 0, 0], [0, 75.0, 0], [85.0, 75.0, 77.5], [85.0, 80.0, 75.0], [90.0, 75.0, 87.5], [87.5, 77.5, 0], [75.0, 80.0, 75.0], [90.0, 75.0, 85.0], [0, 75.0, 0], [0, 0, 0], [87.5, 80.0, 0], [100.0, 95.0, 95.0], [87.5, 82.5, 0], [75.0, 0, 0], [0, 0, 0], [75.0, 85.0, 0], [87.5, 87.5, 80.0], [85.0, 75.0, 0], [85.0, 82.5, 85.0], [0, 90.0, 0], [75.0, 0, 0], [95.0, 97.5, 92.5], [0, 75.0, 0], [0, 0, 0], [0, 0, 0], [85.0, 87.5, 80.0], [82.5, 85.0, 80.0], [0, 77.5, 0], [92.5, 90.0, 90.0], [85.0, 0, 0], [92.5, 87.5, 90.0], [0, 75.0, 0], [0, 85.0, 0], [77.5, 0, 0], [87.5, 85.0, 87.5], [75.0, 80.0, 75.0], [0, 0, 0], [0, 75.0, 0], [80.0, 0, 0], [87.5, 75.0, 0], [75.0, 75.0, 0], [75.0, 0, 0], [82.5, 75.0, 0], [0, 75.0, 0], [90.0, 0, 85.0], [0, 0, 0], [92.5, 87.5, 85.0]]
result_RRTC = []
result_RRTM = []
result_RRT = []
w1 = 0
w2 = 0
w3 = 0
for i in result:
    result_RRTC.append(i[0])
    result_RRTM.append(i[1])
    result_RRT.append(i[2])
    if i[0] > i[1] and i[0] > i[2]:
        w1 += 1
    elif i[1] > i[0] and i[1] > i[2]:
        w2 += 1
    else:
        w3 += 1
print("w1:"+ str(w1))
print("w2:"+ str(w2))
print("w3:"+ str(w3))

plt.figure(figsize=(100, 4))
x1=range(0,len(result_RRTC))
x2=range(0,len(result_RRTM)) 
x3=range(0,len(result_RRT))
plt.plot(x1,result_RRTC,label='RRT_Connect',linewidth=2,color='w',marker='o',
markerfacecolor='red',markersize=5) 
plt.plot(x2,result_RRTM,label='RRT_Modified',linewidth=2,color='w',marker='o',
markerfacecolor='green',markersize=5) 
plt.plot(x3,result_RRT,label='RRT',linewidth=2,color='w',marker='o',
markerfacecolor='blue',markersize=5) 
plt.show()