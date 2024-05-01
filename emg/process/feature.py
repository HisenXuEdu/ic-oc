import math
import numpy as np



class Feature():
    def __init__(self, x):
        self.x = x
        self.time_features_matrix = []
        self.frequency_features_matrix = []



    # def features_estimation(signal, channel_name, fs, frame, step, plot=True):
    #     """
    #     Compute time, frequency and time-frequency features from signal.
    #     :param signal: numpy array signal.
    #     :param channel_name: string variable with the EMG channel name in analysis.
    #     :param fs: int variable with the sampling frequency used to acquire the signal
    #     :param frame: sliding window size
    #     :param step: sliding window step size
    #     :param plot: boolean variable to plot estimated features.
    #     :return: total_feature_matrix -- python Data-frame with.
    #     :return: features_names -- python list with
    #     """

    #     features_names = ['VAR', 'RMS', 'IEMG', 'MAV', 'LOG', 'WL', 'ACC', 'DASDV', 'ZC', 'WAMP', 'MYOP', "FR", "MNP", "TP",
    #                     "MNF", "MDF", "PKF", "WENT"]

    #     time_matrix = time_features_estimation(signal, frame, step)
    #     frequency_matrix = frequency_features_estimation(signal, fs, frame, step)
    #     time_frequency_matrix = time_frequency_features_estimation(signal, frame, step)
    #     total_feature_matrix = pd.DataFrame(np.column_stack((time_matrix, frequency_matrix, time_frequency_matrix)).T,
    #                                         index=features_names)

    #     print('EMG features were from channel {} extracted successfully'.format(channel_name))

    #     if plot:
    #         plot_features(signal, channel_name, fs, total_feature_matrix, step)

    #     return total_feature_matrix, features_names
    
    
    def time_features_estimation(self, x, frame):
        """
        Compute time features from signal using sliding window method.
        :param signal: numpy array signal.
        :param frame: sliding window size.
        :param step: sliding window step size.
        :return: time_features_matrix: narray matrix with the time features stacked by columns.
        """

        variance = []
        rms = []
        iemg = []
        mav = []
        log_detector = []
        wl = []
        aac = []
        dasdv = []

        for i in range(x.shape[0]):
            variance.append(np.var(x[i]))
            rms.append(np.sqrt(np.mean(x[i] ** 2)))
            iemg.append(np.sum(abs(x[i])))  # Integral
            mav.append(np.sum(np.absolute(x[i])) / frame)  # Mean Absolute Value
            log_detector.append(np.exp(np.sum(np.log10(np.absolute(x[i]))) / frame))
            wl.append(np.sum(abs(np.diff(x[i]))))  # Wavelength
            aac.append(np.sum(abs(np.diff(x[i]))) / frame)  # Average Amplitude Change
            dasdv.append(math.sqrt((1 / (frame - 1)) * np.sum((np.diff(x[i])) ** 2)))  # Difference absolute standard deviation value
            # zc.append(zcruce(x[i], th))  # Zero-Crossing
            # wamp.append(wilson_amplitude(x[i], th))  # Willison amplitude
            # myop.append(myopulse(x[i], th))  # Myopulse percentage rate

        self.time_features_matrix = np.column_stack((variance, rms, iemg, mav, log_detector, wl, aac, dasdv))
        print(self.time_features_matrix.shape)
        # return time_features_matrix

    # def force_process(self,y):
    #     list = []
        
    #     for i in range(y.shape[0]):
    #         self.label.append(np.mean(y[i]))
    #     self.label= np.array(list)
    

    def frequency_features_estimation(self, x, fs, frame, step):

        print("someting wrong!!!!!!!!!!!!!!!")
        """
        Compute frequency features from signal using sliding window method.
        :param signal: numpy array signal.
        :param fs: sampling frequency of the signal.
        :param frame: sliding window size
        :param step: sliding window step size
        :return: frequency_features_matrix: narray matrix with the frequency features stacked by columns.
        """

        fr = []
        mnp = []
        tot = []
        mnf = []
        mdf = []
        pkf = []

        print(x.shape[0])
        for i in range(x.shape[0]):
            frequency, power = spectrum(x[i], fs)
            print(frequency.shape)

            fr.append(frequency_ratio(frequency, power))  # Frequency ratio
            mnp.append(np.sum(power) / len(power))  # Mean power
            tot.append(np.sum(power))  # Total power
            mnf.append(mean_freq(frequency, power))  # Mean frequency
            mdf.append(median_freq(frequency, power))  # Median frequency
            pkf.append(frequency[power.argmax()])  # Peak frequency

        self.frequency_features_matrix = np.column_stack((fr, mnp, tot, mnf, mdf, pkf))



#时域###################################

def zcruce(X):
    cruce = 0
    for cont in range(len(X) - 1):
        can = X[cont] * X[cont + 1]
        can2 = abs(X[cont] - X[cont + 1])
        if can < 0 and can2 > 0:
            cruce = cruce + 1
    return cruce

def wilson_amplitude(signal, th):
    x = abs(np.diff(signal))
    umbral = x >= th
    return np.sum(umbral)

def myopulse(signal, th):
    umbral = signal >= th
    return np.sum(umbral) / len(signal)


#频域####################################

def spectrum(signal, fs):
    m = len(signal)
    n = next_power_of_2(m)
    y = np.fft.fft(signal, n)
    yh = y[0:int(n / 2 - 1)]
    fh = (fs / n) * np.arange(0, n / 2 - 1, 1)
    power = np.real(yh * np.conj(yh) / n)

    return fh, power

def next_power_of_2(x):
    return 1 if x == 0 else 2 ** (x - 1).bit_length()

def frequency_ratio(frequency, power):
    power_low = power[(frequency >= 30) & (frequency <= 250)]
    power_high = power[(frequency > 250) & (frequency <= 500)]
    ULC = np.sum(power_low)
    UHC = np.sum(power_high)

    return ULC / UHC

def mean_freq(frequency, power):
    num = 0
    den = 0
    for i in range(int(len(power) / 2)):
        num += frequency[i] * power[i]
        den += power[i]

    return num / den

def median_freq(frequency, power):
    power_total = np.sum(power) / 2
    temp = 0
    tol = 0.01
    errel = 1
    i = 0

    while abs(errel) > tol:
        temp += power[i]
        errel = (power_total - temp) / power_total
        i += 1
        if errel < 0:
            errel = 0
            i -= 1

    return frequency[i]