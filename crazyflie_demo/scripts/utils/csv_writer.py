""" Path to relevant routes folder:
  /home/user/catkin_ws/src/crazyflie_ros/crazyflie_demo/routes
"""
import csv

square_edge = 0.5
# z = coordination_updater.z
# square_route = [(coordination_updater.x - square_edge, coordination_updater.y - square_edge, z),
#                 (coordination_updater.x + square_edge, coordination_updater.y - square_edge, z),
#                 (coordination_updater.x + square_edge, coordination_updater.y + square_edge, z),
#                 (coordination_updater.x - square_edge, coordination_updater.y + square_edge, z)]

x = 1  # fixed x coordination because we are building square route in plane y (y up)
y = 0.5
z = 0.5
square_route = [(x, y, z),
                (x, y + y, z),
                (x, y + y, z + z),
                (x, y, z + z)]

path = "/home/user/catkin_ws/src/crazyflie_ros/crazyflie_demo/routes"

with open(path + '/square_y_plane_5.csv', 'w') as csvfile:
    fieldnames = ['point_number', 'coordination']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    for point_number, coordination in enumerate(square_route):
        writer.writerow({'point_number': point_number, 'coordination': coordination})
