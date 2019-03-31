"""Usage:

insert all routes in /home/user/catkin_ws/src/crazyflie_ros/crazyflie_demo/routes
(assuming your catkin installed on home).

to read a route, run this script with "-n name_of_route" as argument.

"""

import csv
import re

# construct the argument parse and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-n", "--name", required=True,
#                 help="name of the route")
# args = vars(ap.parse_args())
#
# # display a friendly message to the user
# name = args["name"]

cf_route = []


def get_route(file_name, cf_name, print_route=False):
    cf_number = int(re.search(r'\d+', cf_name).group())
    print("cf_number: {}".format(cf_number))
    with open(file_name + '.csv') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            cf_route.append(row[3 * cf_number - 3:3 * cf_number])
    return cf_route


# For testing:
if __name__ == '__main__':
    route = get_route("/home/user/catkin_ws/src/crazyflie_ros/crazyflie_demo/routes/swarm_route_test", "CF_1")
    print(tuple(map(float, route[0])))
