import matplotlib.pyplot as plt
import numpy as np

########################################################################
# The below settings need to be changed to match your setup
########################################################################
bins_to_operate_on    = 15          # The original Navtech c++ code sets this between 5 & 15
start_bin             = 10          # Allows for bins really close-in to be ignored
threshold             = 85          # 85 is the value that one customer uses for target identification
range_gain            = 1           # Radar specific value to scale the reported range-in-metres 
range_offset          = 0           # Radar specific parameter to offset the reported range-in-metres
range_resolution_metres = 0.044     # This value is rounded from the /real/ value to go in here!
max_peaks_per_azimuth = 99          # maximum number of peaks 
filename = "OneAzimuthFFT.csv"      # text file containing one line of comma separated integers
##########################################################################



########################################################################
# The below settings should not be changed
########################################################################
awaiting_rise = False
data = []
########################################################################



########################################################################
# The below section contains function definitions
########################################################################

# Raise a numer to the power 2
def square(number):
    return number ** 2

# Raise a numer to the power 3
def cube(number):
    return number ** 3

# Find the peak bin 
def find_peak_bin (data, start_bin, end_bin, bins_to_operate_upon, threshold):
    global awaiting_rise
    if (start_bin > (end_bin - bins_to_operate_upon)):
        return end_bin 
    for peak_bin in range (start_bin , end_bin - bins_to_operate_upon):    
        if (awaiting_rise and (data[peak_bin] > data[peak_bin + 1])):
            continue
        elif (awaiting_rise):
            awaiting_rise = False
        if ((data[peak_bin] > threshold) and (data[peak_bin + 1] < data[peak_bin])) :
            awaiting_rise = True
            return peak_bin
    return end_bin 

# Resolve the peak 
def peak_resolve(data, peak_bin, bins_to_operate_upon):
    x=[0] * bins_to_operate_on
    x_2=[0] * bins_to_operate_on
    x_3=[0] *bins_to_operate_on
    x_4=[0] * bins_to_operate_on
    y=[0] * bins_to_operate_on
    x_y=[0] * bins_to_operate_on
    x_2_y=[0] * bins_to_operate_on
    bins_to_offset = int((bins_to_operate_upon - 1) / 2 )   
    index = 0    
    start_value = peak_bin - bins_to_offset    
    for index in range (0, bins_to_operate_upon):
        x[index] = start_value + index
    start_bin = peak_bin - bins_to_offset 
    for index in range (0, bins_to_operate_upon):
        y[index] = data[start_bin + index]    
    s_x =  0.0
    s_x_2 = 0.0
    s_x_3 = 0.0
    s_x_4 = 0.0    
    s_x=sum(x[0:bins_to_operate_upon])
    s_x_2=sum(list(map(square,x[0:bins_to_operate_upon])))
    x_3=list(map(cube,x[0:bins_to_operate_upon]))
    s_x_3=sum(x_3[0:bins_to_operate_upon])
    x_2=list(map(square,x[0:bins_to_operate_upon]))
    x_4=list(map(square,x_2[0:bins_to_operate_upon]))
    s_x_4=sum(x_4[0:bins_to_operate_upon])
    s_y =   0.0
    s_x_y =  0.0
    s_x_2_y = 0.0    
    s_y=sum(y[0:bins_to_operate_upon])
    x_y=[X*Y for X,Y in zip(x[0:bins_to_operate_upon],y[0:bins_to_operate_upon])]
    s_x_y=sum(x_y[0:bins_to_operate_upon])
    x_2_y=[X*Y for X,Y in zip(x_2[0:bins_to_operate_upon],y[0:bins_to_operate_upon])]
    s_x_2_y=sum(x_2_y[0:bins_to_operate_upon])
    A = [s_x_2, s_x_3, s_x_4, s_x_2_y]
    B =[s_x, s_x_2, s_x_3, s_x_y]  
    C = [ bins_to_operate_upon, s_x, s_x_2, s_y ]   
    F = C[0] / A[0]    
    for index in range (0,4):
        C[index] = C[index] - (F * A[index])    
    F = B[0] / A[0]    
    for index in range (0,4):
        B[index] = B[index] - (F * A[index])    
    F = C[1] / B[1]    
    for index in range (0,4):        
        C[index] -= F * B[index]    
    b2 = C[3] / C[2]    
    b1 = (B[3] - B[2] * b2) / B[1]    
    return -b1 / (2 * b2) #+  startBin - (0 - bins_to_offset)    
######################################################################################



########################################################################
# The below section contains the main program code
########################################################################   

# Open the file containing the FFT data
try:
    data = np.genfromtxt(filename, delimiter=',')
except Exception as e:
    print("Cannot open data file {} - {}".format(filename, e))
    exit()

print ("\n######### SETTINGS #########")
print ("Bins to operate on: {}".format(bins_to_operate_on))
print ("Threshold:       {}".format(threshold))
print ("Range resolution: {}".format(range_resolution_metres))
print ("Range gain:      {}".format(range_gain))
print ("Range offset:    {}".format(range_offset))
print ("Max peaks:       {}".format(max_peaks_per_azimuth))

end_bin = len(data)
max_bins_to_operate_on = end_bin
minimum_range = bins_to_operate_on * range_resolution_metres
maximum_range = end_bin * range_resolution_metres
peaks_found = 0
peak_bin = 0
min_bin_to_operate_on = 0
min_bin_to_operate_upon = start_bin

# Plot the data
if len(data) <= 0:
    print("No data points to plot")
    exit()
plt.figure("One Azimuth Analysis - Navigation Mode peak detection")
plt.title("Single Azimuth of FFT Radar Data - 'Navigation Mode' Peak Detection\nPeakBins marked in orange, ResolvedPeaks marked in green")
plt.plot(data, ".", markersize=5, color='b')
plt.hlines(threshold, start_bin, len(data), color='red')
while ((peak_bin != end_bin) and (peaks_found < max_peaks_per_azimuth)):
    peak_bin = find_peak_bin(data, min_bin_to_operate_upon, end_bin, bins_to_operate_on, threshold)
    min_bin_to_operate_upon = peak_bin + bins_to_operate_on
    if (peak_bin < end_bin):
        print ("\n####### PEAK FOUND #########")
        print ("Peak bin found:  {}".format(peak_bin))
        plt.plot(peak_bin, data[peak_bin],".-.", color='orange', markersize=12)
        resolved_bin = peak_resolve(data, peak_bin, bins_to_operate_on)
        print ("resolved bin at: {:.3f}".format(resolved_bin))
        resolved_range = (resolved_bin * range_gain * range_resolution_metres) + range_offset
        print ("Resolved range:  {:.3f}m".format(resolved_range))
        if ((resolved_range < minimum_range) or (resolved_range > maximum_range)):
            print ("Implausible resolved range")
            continue
        peaks_found += 1
        plt.vlines(resolved_bin, min(data), max(data), color='green')
print ("\n############################")
plt.ylabel('Returned power', fontsize=12)
plt.xlabel('Reporting bin', fontsize=12)
plt.tight_layout()
plt.show()