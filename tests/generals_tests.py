# Open file
file1 = open("orders_1.txt", "r")

line = 'a'
while line != "":

    # Read order
    line = file1.readline()
    order = line.split(',')
    for i in range(0, len(order[3])):
        order[3][i] = int(order[3][i])
    print(order[3])
    print(order[3])

file1.close()
