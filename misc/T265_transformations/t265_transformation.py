import rerun as rr
import numpy as np
import matplotlib.pyplot as py 
import argparse 
from pathlib import Path
from scipy import signal
rrd_path = Path("t265_info.rrd")

with rr.server.Server(datasets={"dataset": [rrd_path]}) as server:
    dataset = server.client().get_dataset("dataset")

    t265_acc = dataset.filter_contents(["/t265_acceleration"]) 
    df = t265_acc.reader(index="log_time", fill_latest_at=True)
    pdf = df.to_pandas()
    print(pdf.columns)
    

    imu_accel = np.array([x[0] for x in pdf["/t265_acceleration:Arrows3D:vectors"]])
    imu_time = pdf["log_time"].values
    imu_time_s = (imu_time - imu_time[0]).astype('float') * 1e-9
   
    degx = np.deg2rad(180)
    rx = np.array([
        [1,0,0],
        [0,np.cos(degx),-np.sin(degx)],
        [0,np.sin(degx),np.cos(degx)]
    ])
    degz = np.deg2rad(90)
    rz = np.array([
        [np.cos(degz),-np.sin(degz),0],
        [np.sin(degz),np.cos(degz),0],
        [0,0,1] 
    ])
     
    rotation_matrix = rz @ rx
 

    imu_rotated = (rotation_matrix @ imu_accel.T).T
   

    onboard_velo = dataset.filter_contents(["/localizer/velocity"])
    velo_df = onboard_velo.reader(index="log_time", fill_latest_at=True)
    pdf_velo = velo_df.to_pandas()
    





    local_velo = np.array([x[0] for x in pdf_velo["/localizer/velocity:Arrows3D:vectors"]])
    time_for_local = pdf_velo["log_time"].values
    time_s = (time_for_local - time_for_local[0]).astype('float') * 1e-9 

    local_accel = np.gradient(local_velo,time_s,axis=0)
    print(pdf_velo.columns)


    def lowpass_filter(data, cutoff, fs, order=2,axis=0):
        sos = signal.butter(order,cutoff,btype='low',fs=fs,output='sos')
        return signal.sosfiltfilt(sos, data, axis=axis)
    
    fs = 1.0 / np.mean(np.diff(time_s))

    filtered_localizer_acc = lowpass_filter(local_accel,5,fs,order=2)
    filtered_imu_acc = lowpass_filter(imu_rotated,5,fs,order=2)


    py.figure(figsize=(10,5))


    #py.plot(imu_time_s, filtered_imu_acc[:,0], label='IMU accel x', color='r', linestyle='--')
    #py.plot(imu_time_s, filtered_imu_acc[:,1], label='IMU accel y', color='g', linestyle='--')
    py.plot(imu_time_s, filtered_imu_acc[:,2], label='IMU accel z', color='b', linestyle='--')


    # py.plot(time_s, filtered_localizer_acc[:,0], label='Local accel x', color='black')
    #py.plot(time_s, filtered_localizer_acc[:,1], label='Local accel y', color='pink')
    py.plot(time_s, filtered_localizer_acc[:,2], label='Local accel z', color='purple')

    py.ylim(-5,5)
    py.xlabel("Time (s)")
    py.ylabel("Acceleration (m/s²)")
    py.title("IMU vs Localizer Acceleration")
    py.legend()
    py.grid(True)
    py.show()