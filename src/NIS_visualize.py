import pandas as pd
import matplotlib.pyplot as plt

datas = pd.read_csv('good.csv')
NIS_Lidar = []
NIS_Radar = []

for i in range(datas.shape[0]):
    data = datas.iloc[i, :]
    if data['sensor'] == 'L':
        print("L = ", data['value'])
        NIS_Lidar.append(data['value'])
    if data['sensor'] == 'R':
        print("R = ", data['value'])
        NIS_Radar.append(data['value'])

# Lidar : NIS 95 % - 5.991 (2 DOF)
nis_thres_lidar = 5.991
plt.plot(NIS_Lidar)
plt.title("NIS for Lidar")
plt.xlabel("steps")
plt.ylabel("NIS")
plt.plot([0, len(NIS_Lidar)-1], [nis_thres_lidar, nis_thres_lidar], 'r-')
plt.grid()
plt.savefig('../fig/NIS4lidar.png')
#plt.show()

# Radar : NIS 95 % - 7.815 (3 DOF)
nis_thres_radar = 7.815
plt.clf()
plt.plot(NIS_Radar)
plt.title("NIS for Radar")
plt.xlabel("steps")
plt.ylabel("NIS")
plt.plot([0, len(NIS_Lidar)-1], [nis_thres_radar, nis_thres_radar], 'r-')
plt.grid()
plt.savefig('../fig/NIS4radar.png')
#plt.show()
