
import pandas as pd

text_file_path = 'isaac_data_2.txt' #change to wanted file

column_name = 'xx' #change to column title

data = []

with open(text_file_path, 'r') as file:
    for line in file:
        # Split the line into columns
        columns = line.strip().split(',') 
        column_index = 0  # Change to the appropriate column index
        data.append(columns[column_index])

TOP = 1 #state 1
BELOW = 0 #state 1
best_threshhold = 0
best_step_count = 10000

state = TOP

 #check every threshold value 10,000 to 30,000 in increments of 5
for THRESH in range(10_000,30_000,5):
    steps = 0
    for i in range(1,len(data)):
        xx = int(data[i]) #accelerometer reading
        if state == TOP: #switch case block in c code
            if xx < THRESH:
                steps += 1 #add one step if state switch
                state = BELOW
        elif state == BELOW:
            if xx >= THRESH:
                state = TOP
    #print(steps)
    
    #find threshold value where steps was closest to 100
    if (abs(best_step_count - 100) > abs(steps - 100)):
        best_threshhold = THRESH
        best_step_count = steps

print(f"best step count:{best_step_count}")
print(f"best threshold value:{best_threshhold}")

