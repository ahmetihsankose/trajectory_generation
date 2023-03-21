import matplotlib.pyplot as plt

# read in data from file
with open('output.txt', 'r') as f:
    data = [float(line.strip()) for line in f.readlines()]

# create plot
plt.plot(data)
plt.xlabel('Data Points')
plt.ylabel('Values')
plt.title('Data Plot')
plt.show()