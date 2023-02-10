#!/usr/bin/env python3
import numpy as np
import rospy
import math
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32, Bool

def vector3_to_numpy(msg, hom=False):
    if hom:
        return np.array([msg.x, msg.y, msg.z, 0])
    else:
        return np.array([msg.x, msg.y, msg.z])

def rotation(velocities, rotations):
    x, y = velocities.x, velocities.y
    t3 = +2.0 * (rotations.w * rotations.z)
    t4 = +1.0 - 2.0 * (rotations.z ** 2)
    Z = math.degrees(math.atan2(t3, t4))
    if abs(Z) > 1:
        yaw = Z
    else:
        if y < 0.01:
            y = 0
        if x < 0.01:
            x = 0
        yaw = np.degrees(np.arctan2(y, x))
    return yaw

def rot_matrix(theta):
    theta = np.radians(theta)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    return R

def check_if_neighbor(current_robot_position, other_robot_position):
    """
    Method that searches for neighbor robots
    :param current_robot_position: TIP, current robot position
    :param other_robot_position: TIP, position of other robot in the world
    return: bool, true - if robot is in the FOV of the current robot, false - otherwise
    """

    [x1, y1] = current_robot_position
    [x2, y2] = other_robot_position

    # Compute distance and angle between current and other robot and compare them to FOV parameters
    distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    angle = math.atan2((y2 - y1), (x2 - x1))

    # return True if distance > 0 and distance < self.fov_radius and angle < self.fov_angle else False
    # return True if distance > 0 and distance < 20 and angle < 2 * np.pi else False
    return True

def create_cohesion_force(current_robot_position, local_center_position):
    """
    Method that creates cohesion force to attract robots to form a group
    :param current_robot_position: TIP, position of current robot
    :param neighbor_robot_position: TIP, local center position
    return: list, cohesion force vector
    """

    # Create cohesion force by substracting position vectors
    force = local_center_position - current_robot_position

    return force

def create_separation_force(current_robot_position, neighbor_robot_position):
    """
    Method that creates force in opposite direction. Force is weighted based on distance. A closer robot has more impact on separation force.
    :param current_robot_position: TIP, position of current robot
    :param neighbor_robot_position: TIP, position of other robot in the world
    return: list, weighted force vector
    """

    [x1, y1] = current_robot_position
    [x2, y2] = neighbor_robot_position

    # Compute distance current and other robot
    distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    # Create weighted force by substracting position vectors
    force = current_robot_position - neighbor_robot_position
    force = force/np.linalg.norm(force) / (distance ** 2)
    # force = -np.array([0 if not (x2-x1) else np.sign(x2-x1)/(x2-x1)**2, 0 if not (y2-y1) else np.sign(y2-y1)/(y2-y1)**2])

    return force

def create_alignment_force(current_robot_velocity, local_center_velocity):
    """
    Method that creates alignment force to attract robots to form a group
    :param current_robot_velocity: TIP, position of current robot
    :param neighbor_robot_velocity: TIP, local center position
    return: list, alignment force vector
    """

    # Create alignment force by substracting position vectors
    force = local_center_velocity - current_robot_velocity

    return force

def create_navigation_force(current_robot_position, goal_position):
    """
    Method that creates force in opposite direction. Force is weighted based on distance. A closer robot has more impact on separation force.
    :param current_robot_position: TIP, position of current robot
    :param neighbor_robot_position: TIP, position of other robot in the world
    return: list, weighted force vector
    """
    # rospy.logwarn(goal_position)
    # rospy.logerr(current_robot_position)
    [x1, y1] = current_robot_position
    [x2, y2] = goal_position.x, goal_position.y

    # Compute distance current and other robot
    # distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    # Create weighted force by substracting position vectors
    force = [x2, y2] - current_robot_position
    # force = force / (distance ** 2)
    # force = -np.array([0 if not (x2-x1) else np.sign(x2-x1)/(x2-x1)**2, 0 if not (y2-y1) else np.sign(y2-y1)/(y2-y1)**2])

    return force

# def create_navigation_force(current_robot_position, goal_position, next_point,i):
#     """
#     Method that creates force in opposite direction. Force is weighted based on distance. A closer robot has more impact on separation force.
#     :param current_robot_position: TIP, position of current robot
#     :param neighbor_robot_position: TIP, position of other robot in the world
#     return: list, weighted force vector
#     """
#     # rospy.logwarn(goal_position)
#     # rospy.logerr(current_robot_position)
#     [x1, y1] = current_robot_position
#     [x2, y2] = goal_position.x, goal_position.y
#     [x3, y3] = next_point.x, next_point.y
#
#     # Compute distance current and other robot
#     distance = ((x2 - x3) ** 2 + (y2 - y3) ** 2) ** 0.5
#
#     # Create weighted force by substracting position vectors
#     force_point = np.array([x3, y3] - current_robot_position)
#     force_goal = np.array([x2, y2] - current_robot_position)
#     force = force_point / np.linalg.norm(force_point) * np.linalg.norm(force_goal)
#     # if distance > 0.1:
#
#     # if i == 0:
#     #     rospy.logerr(f"point:{force_point}, goal:{force_goal}, force:{force}")
#     # force = -np.array([0 if not (x2-x1) else np.sign(x2-x1)/(x2-x1)**2, 0 if not (y2-y1) else np.sign(y2-y1)/(y2-y1)**2])
#
#     return force_point

def create_obstacle_force(current_robot_position, obstacle):
    """
    Method that creates force in opposite direction. Force is weighted based on distance. A closer robot has more impact on separation force.
    :param current_robot_position: TIP, position of current robot
    :param neighbor_robot_position: TIP, position of other robot in the world
    return: list, weighted force vector
    """
    # rospy.logwarn(goal_position)
    # rospy.logerr(current_robot_position)
    [x1, y1] = current_robot_position
    [x2, y2] = obstacle

    # Compute distance current and other robot
    distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    # print(distance)

    # Create weighted force by substracting position vectors
    force = current_robot_position - obstacle
    # force = force / np.linalg.norm(force) / (distance ** 2)
    force = force / np.linalg.norm(force) / (distance ** 2)
    # force = -np.array([0 if not (x2-x1) else np.sign(x2-x1)/(x2-x1)**2, 0 if not (y2-y1) else np.sign(y2-y1)/(y2-y1)**2])

    return force

def cohesion(positions):
    cohesion_forces = []

    for i in range(3):
        current_robot_position = np.array([positions[i].x, positions[i].y])
        current_robot_mass = 1
        local_center_position = 0
        total_robots_mass = 0

        for j in range(3):
            other_robot_position = np.array([positions[j].x, positions[j].y])
            if (other_robot_position == current_robot_position).all():
                continue

            if check_if_neighbor(current_robot_position, other_robot_position):
                neighbor_robot_position = other_robot_position
                neighbor_robot_mass = 1
                local_center_position += neighbor_robot_mass * neighbor_robot_position
                total_robots_mass += neighbor_robot_mass

        local_center_position = local_center_position / total_robots_mass
        current_robot_cohesion_force = create_cohesion_force(current_robot_position, local_center_position)
        cohesion_forces.append(current_robot_cohesion_force)
    # rospy.logerr(cohesion_forces)
    vel = [Twist(), Twist(), Twist()]
    for i in range(3):
        vel[i].linear.x = round(coh_strength * (cohesion_forces[i][0]), 2)
        vel[i].linear.y = round(coh_strength * (cohesion_forces[i][1]), 2)

    return vel

def separation(positions):
    separation_forces = []

    for i in range(3):
        current_robot_position = np.array([positions[i].x, positions[i].y])
        total_separation_force = np.zeros(2)

        for j in range(3):
            other_robot_position = np.array([positions[j].x, positions[j].y])
            if (other_robot_position == current_robot_position).all():
                continue

            if check_if_neighbor(current_robot_position, other_robot_position):
                neighbor_robot_position = other_robot_position
                separation_force = create_separation_force(current_robot_position, neighbor_robot_position)
                total_separation_force += separation_force

        separation_forces.append(total_separation_force)

    vel = [Twist(), Twist(), Twist()]
    for i in range(3):
        vel[i].linear.x = round(sep_strength * separation_forces[i][0], 2)
        vel[i].linear.y = round(sep_strength * separation_forces[i][1], 2)

    return vel

def alignment(positions, rotations, velocities):
    alignment_forces = []

    for i in range(3):
        current_robot_velocity = np.array([round(velocities[i].x, 2), round(velocities[i].y, 2)])
        current_robot_position = np.array([positions[i].x, positions[i].y])
        current_robot_rotation = rotation(velocities[i], rotations[i])

        if velocities[i].y < 0.01:
            current_robot_velocity = np.dot(rot_matrix(current_robot_rotation), current_robot_velocity)

        current_robot_mass = 1
        local_center_velocity = 0
        local_center_rotation = 0
        total_robots_mass = 0

        for j in range(3):
            other_robot_velocity = np.array([round(velocities[j].x, 2), round(velocities[j].y, 2)])
            other_robot_position = np.array([positions[j].x, positions[j].y])
            other_robot_rotation = rotation(velocities[j], rotations[j])

            if velocities[j].y < 0.01:
                other_robot_velocity = np.dot(rot_matrix(other_robot_rotation), other_robot_velocity)

            if (other_robot_position == current_robot_position).all():
                continue

            if check_if_neighbor(current_robot_position, other_robot_position):
                neighbor_robot_velocity = other_robot_velocity
                neighbor_robot_rotation = other_robot_rotation
                neighbor_robot_mass = 1

                local_center_velocity += neighbor_robot_mass * neighbor_robot_velocity
                local_center_rotation += neighbor_robot_rotation
                total_robots_mass += neighbor_robot_mass

        local_center_velocity = local_center_velocity / total_robots_mass
        alignment_forces.append(local_center_velocity)

    vel = [Twist(), Twist(), Twist(), Twist()]

    for i in range(3):
        vel[i].linear.x = round(alig_strength * (alignment_forces[i][0]), 2)
        vel[i].linear.y = round(alig_strength * (alignment_forces[i][1]), 2)

    return vel

def navigation(positions):
    navigation_forces = []
    global popped
    for i in range(3):
        current_robot_position = positions[i].x, positions[i].y
        current_robot_position = np.array(current_robot_position)
        if not start:
            navigation_forces.append(create_navigation_force(current_robot_position, goal_position))
        if start:
            temp = Point()
            temp.x = goal_positions[0][0]
            temp.y = goal_positions[0][1]
            navigation_forces.append(create_navigation_force(current_robot_position, temp))
    if start:
        for i in range(1):
            current_robot_position = positions[i].x, positions[i].y
            current_robot_position = np.array(current_robot_position)
            try:
                # rospy.logwarn(np.linalg.norm([current_robot_position.x-goal_positions[0][0], current_robot_position.y-goal_positions[0][1]]))
                if np.linalg.norm([current_robot_position[0]-goal_positions[0][0], current_robot_position[1]-goal_positions[0][1]]) < 0.5:
                    # rospy.logerr("CHECK")
                    if len(goal_positions) != 0 and popped == 0:
                        goal_positions.pop(0)
                        popped = 1
                else:
                    popped = 0

            except(IndexError):
                print("ERROR")
                pass

    vel = [Twist(), Twist(), Twist()]

    for i in range(3):
        vel[i].linear.x = round(nav_strength * navigation_forces[i][0], 2)
        vel[i].linear.y = round(nav_strength * navigation_forces[i][1], 2)

    return vel

def obstacle(positions):
    obstacle_forces = []
    for i in range(3):
        force = np.array([0.0, 0.0])
        robot_pos = np.array([positions[i].x, positions[i].y])
        pos_map = [100 + int(robot_pos[1] // 0.05), 100 + int(robot_pos[0] // 0.05)]
        for j in range(19):
            for k in range(19):
                try:
                    if map[pos_map[0] + j - 9][pos_map[1] + k - 9] == 100:
                        # rospy.logerr("CHECK")
                        # if i ==0:
                        #     rospy.logerr([(pos_map[1]+k-9)*0.05-5, (pos_map[0]+j-9)*0.05-5])
                        force += create_obstacle_force(robot_pos, [(pos_map[1] + k - 9) * 0.05 - 5,
                                                                   (pos_map[0] + j - 9) * 0.05 - 5])
                except(IndexError):
                    # rospy.logwarn("INDEX ERROR")
                    continue
        obstacle_forces.append(force)

    vel = [Twist(), Twist(), Twist(), Twist()]

    for i in range(3):
        vel[i].linear.x = round(obs_strength * obstacle_forces[i][0], 2)
        vel[i].linear.y = round(obs_strength * obstacle_forces[i][1], 2)

    return vel

def callback_start(data):
    global start
    start = data

def callback_goal(data):
    global goal_position
    goal_position = data

def callback_map(data):
    global map
    map = data

def callback_coh_strength(data):
    global coh_strength
    coh_strength = data.data

def callback_sep_strength(data):
    global sep_strength
    sep_strength = data.data

def callback_alig_strength(data):
    global alig_strength
    alig_strength = data.data

def callback_nav_strength(data):
    global nav_strength
    nav_strength = data.data

def callback_obs_strength(data):
    global obs_strength
    obs_strength = data.data

def callback1(data):
    velocities[0] = data.twist.twist.linear
    # positions[0] = data.pose.pose.position
    rotations[0] = data.pose.pose.orientation

def callback2(data):
    # data.pose.pose.position.x += 0.5
    velocities[1] = data.twist.twist.linear
    # positions[1] = data.pose.pose.position
    rotations[1] = data.pose.pose.orientation

def callback3(data):
    velocities[2] = data.twist.twist.linear
    # positions[2] = data.pose.pose.position
    rotations[2] = data.pose.pose.orientation

def callback4(data):
    velocities[3] = data.twist.twist.linear
    positions[3] = data.pose.pose.position
    rotations[3] = data.pose.pose.orientation

def callback1_pos(data):
    positions[0] = data

def callback2_pos(data):
    positions[1] = data

def callback3_pos(data):
    positions[2] = data

def main():
    rospy.init_node('main', anonymous=True)
    rate = rospy.Rate(10)

    pub0 = rospy.Publisher('/sphero_0/cmd_vel', Twist, queue_size=1)
    pub1 = rospy.Publisher('/sphero_1/cmd_vel', Twist, queue_size=1)
    pub2 = rospy.Publisher('/sphero_2/cmd_vel', Twist, queue_size=1)
    # pub3 = rospy.Publisher('/robot_3/cmd_vel', Twist, queue_size=1)

    rospy.Subscriber("/cohesion_strength", Float32, callback_coh_strength)
    rospy.Subscriber("/separation_strength", Float32, callback_sep_strength)
    # rospy.Subscriber("/alignment_strength", Float32, callback_alig_strength)
    rospy.Subscriber("/nav_strength", Float32, callback_nav_strength)
    # rospy.Subscriber("/obs_strength", Float32, callback_obs_strength)
    rospy.Subscriber("/goal_position", Point, callback_goal)
    # rospy.Subscriber("/map", OccupancyGrid, callback_map)
    rospy.Subscriber("/sphero_0/odom", Odometry, callback1)
    rospy.Subscriber("/sphero_1/odom", Odometry, callback2)
    rospy.Subscriber("/sphero_2/odom", Odometry, callback3)
    # rospy.Subscriber("/robot_3/odom", Odometry, callback4)
    rospy.Subscriber("/sphero_0/position", Point, callback1_pos)
    rospy.Subscriber("/sphero_1/position", Point, callback2_pos)
    rospy.Subscriber("/sphero_2/position", Point, callback3_pos)
    rospy.Subscriber("/start", Bool, callback_start)

    # rospy.wait_for_message("/map", OccupancyGrid)
    rospy.wait_for_message("/sphero_0/odom", Odometry)
    rospy.wait_for_message("/sphero_1/odom", Odometry)
    rospy.wait_for_message("/sphero_2/odom", Odometry)
    # rospy.wait_for_message("/robot_2/odom", Odometry)
    # rospy.wait_for_message("/robot_3/odom", Odometry)
    rospy.wait_for_message("/sphero_0/position", Point)
    rospy.wait_for_message("/sphero_1/position", Point)
    rospy.wait_for_message("/sphero_2/position", Point)

    # global map
    # map = np.reshape(map.data, (-1, 200))
    while not rospy.is_shutdown():
        # vel = [Twist(), Twist(), Twist(), Twist()]
        vel = [Twist(), Twist(), Twist()]
        max_vel = 0
        max_vel_comp = 0
        vel_coh = cohesion(positions)
        vel_sep = separation(positions)
        # vel_alig = alignment(positions, rotations, velocities)
        vel_nav = navigation(positions)
        # vel_obs = obstacle(positions)
        # rospy.logerr(vel_coh)
        # rospy.logwarn(vel_sep)
        ##for i in range(3):
        for i in range(3):
            """
            Uncomment if using 1 boid as a leader
            """
            ##############################################################################
            # if i != 0:
            #     vel_nav[i].linear.x, vel_nav[i].linear.y = 0, 0
            # if i == 0:
            #     vel_alig[i].linear.x, vel_coh[i].linear.x, vel_sep[i].linear.x = 0, 0, 0
            #     vel_alig[i].linear.y, vel_coh[i].linear.y, vel_sep[i].linear.y = 0, 0, 0
            ##############################################################################
            # if i == 0:
            # #     vel_alig[i].linear.x = 0
            # #     vel_alig[i].linear.y = 0
            # #     vel[i].linear.x = vel_coh[i].linear.x + vel_sep[i].linear.x + vel_alig[i].linear.x + vel_nav[i].linear.x
            # #     vel[i].linear.y = vel_coh[i].linear.y + vel_sep[i].linear.y + vel_alig[i].linear.y + vel_nav[i].linear.y
            #     vel[i].linear.x = vel_obs[i].linear.x + vel_nav[i].linear.x
            #     vel[i].linear.y = vel_obs[i].linear.y + vel_nav[i].linear.y

            # vel[i].linear.x = vel_coh[i].linear.x + vel_sep[i].linear.x + vel_alig[i].linear.x + vel_nav[i].linear.x + vel_nav[i].linear.x + vel_obs[i].linear.x
            # vel[i].linear.y = vel_coh[i].linear.y + vel_sep[i].linear.y + vel_alig[i].linear.y + vel_nav[i].linear.y + vel_nav[i].linear.y + vel_obs[i].linear.y
            # vel[i].linear.x = (vel_coh[i].linear.x + vel_sep[i].linear.x)*100
            # vel[i].linear.y = (vel_coh[i].linear.y + vel_sep[i].linear.y)*100

            for k in range(3):
                if max(abs(np.array([vel_coh[i].linear.x, vel_sep[i].linear.x, vel_coh[i].linear.y, vel_sep[i].linear.y,vel_nav[i].linear.x,vel_nav[i].linear.y]))) > max_vel_comp:
                    max_vel_comp = max(abs(np.array([vel_coh[i].linear.x, vel_sep[i].linear.x, vel_coh[i].linear.y, vel_sep[i].linear.y,vel_nav[i].linear.x,vel_nav[i].linear.y])))

            # for k in range(3):
            #     if max(abs(np.array([vel_coh[i].linear.x, vel_sep[i].linear.x, vel_coh[i].linear.y, vel_sep[i].linear.y]))) > max_vel_comp:
            #         max_vel_comp = max(abs(np.array([vel_coh[i].linear.x, vel_sep[i].linear.x, vel_coh[i].linear.y, vel_sep[i].linear.y])))
            #
            #
            for t in range(3):
                if abs(max_vel_comp) > 0.5:
                    vel_coh[i].linear.x = vel_coh[i].linear.x / max_vel_comp * 0.5
                    vel_coh[i].linear.y = vel_coh[i].linear.y / max_vel_comp * 0.5
                    vel_sep[i].linear.x = vel_sep[i].linear.x / max_vel_comp * 0.5
                    vel_sep[i].linear.y = vel_sep[i].linear.y / max_vel_comp * 0.5
                    vel_nav[i].linear.x = vel_nav[i].linear.x / max_vel_comp * 0.5
                    vel_nav[i].linear.y = vel_nav[i].linear.y / max_vel_comp * 0.5

            vel[i].linear.x = (vel_coh[i].linear.x+vel_sep[i].linear.x+vel_nav[i].linear.x)*100
            vel[i].linear.y = (vel_coh[i].linear.y+vel_sep[i].linear.y+vel_nav[i].linear.y)*100
            # vel[i].linear.x = (vel_coh[i].linear.x+vel_sep[i].linear.x) * 100
            # vel[i].linear.y = (vel_coh[i].linear.y+vel_sep[i].linear.y) * 100
            # rospy.loginfo(i)
            # rospy.logerr(vel_coh[i].linear.x)
            # rospy.logwarn(vel_sep[i].linear.x)

            if max(abs(np.array([vel[i].linear.x, vel[i].linear.y]))) > max_vel:
                max_vel = max(abs(np.array([vel[i].linear.x, vel[i].linear.y])))
            # rospy.logwarn("max_vel"+str(max_vel))
            # rospy.logerr(vel[i].linear.x)
            # rospy.sleep(0.01)
            # print(f"{i}: [{vel[i].linear.x}, {vel[i].linear.y}]")
        for i in range(3):
            if abs(max_vel) > 50:
                vel[i].linear.x = vel[i].linear.x / max_vel * 50
                vel[i].linear.y =  vel[i].linear.y / max_vel * 50
            if abs(vel[i].linear.x) < 5:
                vel[i].linear.x = round(0.0, 1)
            if abs(vel[i].linear.y) < 5:
                vel[i].linear.y = round(0.0, 1)
            # rospy.loginfo(i)
            # rospy.logwarn(vel[i].linear.x)

        rospy.logwarn(positions)
        # for i in range(3):
        #     vel[i].linear.x *= 100
        #     vel[i].linear.y *= 100
        pub0.publish(vel[0])
        # vel[1].linear.x = abs(vel[1].linear.x)
        # vel[1].linear.y = abs(vel[1].linear.y)
        pub1.publish(vel[1])
        # rospy.logwarn(positions)
        # rospy.logerr(vel_coh)
        # # rospy.logwarn(vel_sep)
        # rospy.logerr(vel)
        pub2.publish(vel[2])
        ##pub3.publish(vel[3])

        rate.sleep()

if __name__ == '__main__':
    # positions = [0, 0, 0, 0]
    # velocities = [0, 0, 0, 0]
    # rotations = [0, 0, 0, 0]
    positions = [0, 0, 0]
    velocities = [0, 0, 0]
    rotations = [0, 0, 0]
    coh_strength = 1.0/10
    sep_strength = 0.2/10
    alig_strength = 1
    nav_strength = 1/10
    obs_strength = 0.01
    # map = OccupancyGrid()
    goal_position = Point()
    start = False
    popped = 0
    # goal_positions = [[4.5,1],[4,0],[-2,-1],[-4.5,-3.5],[-3,-4], [0,-4.5]]
    main()
