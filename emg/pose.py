import pytrigno as pytrigno
import pandas as pd

class EMG:
    def __init__(self, channel=1, host='127.0.0.1'):
        self.channel = channel
        self.dev_emg = pytrigno.TrignoEMG(channel_range=(0,self.channel-1), samples_per_read=400,
                    host=host)
        self.dev_emg.start()
        self.x, self.y, self.z = 0, 0, 0
    
    def get_single(self):
        x = self.dev_emg.read()
        self.x = pd.DataFrame(x.T)
        return x
    
    def get_pose(self):
        x = self.get_single()
        print(x)


if __name__ == '__main__':
    emg = EMG(channel=3, host='127.0.0.1')
    while True:
        emg.get_pose()
        break