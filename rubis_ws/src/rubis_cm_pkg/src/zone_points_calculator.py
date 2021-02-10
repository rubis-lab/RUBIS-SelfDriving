import csv
import numpy as np
import matplotlib.pyplot as plt

def get_line_formula(p1, p2):

    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    a = (y1-y2)/(x1-x2)
    b = y1-a*x1

    return a,b

def get_line_function_value(a, b, x):
    return a*x+b

def visualize_zone_line(points):
    data=[]

    with open('zone_line.csv') as csvfile:
        reader = csv.reader(csvfile)
        is_first = True
        for row in reader:
            if (is_first):
                is_first = False
                continue
            line = []
            line.append(int(row[1]))
            line.append(int(row[2]))
            line.append(int(row[3]))
            line.append(int(row[4]))
            line.append(int(row[5]))
            line.append(int(row[6]))
            line.append(int(row[7]))
            line.append(int(row[8]))
            data.append(line)

    for line in data:
        l1p1 = [ points[line[0]][0], points[line[0]][1] ]
        l1p2 = [ points[line[1]][0], points[line[1]][1] ]
        l2p1 = [points[line[2]][0], points[line[2]][1]]
        l2p2 = [points[line[3]][0], points[line[3]][1]]
        l3p1 = [points[line[4]][0], points[line[4]][1]]
        l3p2 = [points[line[5]][0], points[line[5]][1]]
        l4p1 = [points[line[6]][0], points[line[6]][1]]
        l4p2 = [points[line[7]][0], points[line[7]][1]]

        a1, b1 = get_line_formula(l1p1, l1p2)
        a2, b2 = get_line_formula(l2p1, l2p2)
        a3, b3 = get_line_formula(l3p1, l3p2)
        a4, b4 = get_line_formula(l4p1, l4p2)

        lt = get_intersection_point(a1, b1, a4, b4)
        rt = get_intersection_point(a1, b1, a2, b2)
        rb = get_intersection_point(a2, b2, a3, b3)
        lb = get_intersection_point(a3, b3, a4, b4)

        plt.xlim(0, 2500)
        plt.ylim(-3000,2000)

        x_data = [-10000, 10000]

        y_data = [get_line_function_value(a1,b1, x_data[0]), get_line_function_value(a1, b1, x_data[1])]
        plt.plot(x_data, y_data, 'r')
        plt.scatter([lt[0]],[lt[1]])

        y_data = [get_line_function_value(a2, b2, x_data[0]), get_line_function_value(a2, b2, x_data[1])]
        plt.plot(x_data, y_data, 'g')
        plt.scatter([rt[0]], [rt[1]])

        y_data = [get_line_function_value(a3, b3, x_data[0]), get_line_function_value(a3, b3, x_data[1])]
        plt.plot(x_data, y_data, 'b')
        plt.scatter([rb[0]], [rb[1]])

        y_data = [get_line_function_value(a4, b4, x_data[0]), get_line_function_value(a4, b4, x_data[1])]
        plt.plot(x_data, y_data, 'y')
        plt.scatter([lb[0]], [lb[1]])
        print(lt,rt,rb,lb)

    plt.show()

def get_intersection_point(a1, b1, a2, b2):
    x = (-b1+b2)/(a1-a2)
    y = a1*x+b1
    return x, y

def get_points_for_zone(points):
    data = []

    with open('zone_line.csv') as csvfile:
        reader = csv.reader(csvfile)
        is_first = True
        for row in reader:
            if (is_first):
                is_first = False
                continue
            line = []
            line.append(int(row[1]))
            line.append(int(row[2]))
            line.append(int(row[3]))
            line.append(int(row[4]))
            line.append(int(row[5]))
            line.append(int(row[6]))
            line.append(int(row[7]))
            line.append(int(row[8]))
            data.append(line)

    file = open('zone_points.csv', 'w', newline='', encoding='utf-8')
    writer = csv.writer(file)
    writer.writerow(['zone','lt_x','lt_y','rt_x','rt_y','rb_x','rb_y','lb_x','lb_y'])

    zone_points = []
    zone_num = 1
    for line in data:
        l1p1 = [points[line[0]][0], points[line[0]][1]]
        l1p2 = [points[line[1]][0], points[line[1]][1]]
        l2p1 = [points[line[2]][0], points[line[2]][1]]
        l2p2 = [points[line[3]][0], points[line[3]][1]]
        l3p1 = [points[line[4]][0], points[line[4]][1]]
        l3p2 = [points[line[5]][0], points[line[5]][1]]
        l4p1 = [points[line[6]][0], points[line[6]][1]]
        l4p2 = [points[line[7]][0], points[line[7]][1]]

        a1, b1 = get_line_formula(l1p1, l1p2)
        a2, b2 = get_line_formula(l2p1, l2p2)
        a3, b3 = get_line_formula(l3p1, l3p2)
        a4, b4 = get_line_formula(l4p1, l4p2)

        lt = get_intersection_point(a1, b1, a4, b4)
        rt = get_intersection_point(a1, b1, a2, b2)
        rb = get_intersection_point(a2, b2, a3, b3)
        lb = get_intersection_point(a3, b3, a4, b4)


        writer.writerow([zone_num, lt[0], lt[1], rt[0], rt[1], rb[0], rb[1], lb[0], lb[1]])
        zone_num += 1


def main():
    points = [[-1,-1]]
    with open('points.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, skipinitialspace=True)
        is_first = True
        for row in reader:
            if(is_first):
                is_first = False
                continue
            line = []
            line.append(float(row[4]))
            line.append(float(row[5]))
            points.append(line)

    visualize_zone_line(points)
    # get_points_for_zone(points)


if __name__=="__main__":
    main()



