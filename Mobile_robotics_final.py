import math
import winsound
import numpy as np
import random
import sim as vp
import matplotlib.pyplot as plt
import time

print("Hello Dr.Uzor!")
s_values = [] ## sensor 7 readings will be appended to this list
class Robot(): # the main Robot class with several methods
    def __init__(self):
        for i in range(1, 17): ## this loop used to activate all the sensors in the robot by looping through the sensors
            error, self.i = vp.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(i),
                                                   vp.simx_opmode_blocking)
            error = vp.simxReadProximitySensor(clientID, self.i, vp.simx_opmode_streaming)
            s_values.append(self.i)

        code, self.pos = vp.simxGetObjectHandle(clientID, "Dummy", vp.simx_opmode_blocking)

        # activating left and right motors
        code, self.R_motor = vp.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor",vp.simx_opmode_blocking)
        code, self.L_motor = vp.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor",vp.simx_opmode_blocking)

        code, self.p3dx = vp.simxGetObjectHandle(clientID, "Pioneer_p3dx", vp.simx_opmode_blocking)
        code, self.camera_1 = vp.simxGetObjectHandle(clientID, "Front_vision", vp.simx_opmode_oneshot_wait)
        code, self.camera_2 = vp.simxGetObjectHandle(clientID, "Left_vision", vp.simx_opmode_oneshot_wait)
        code, self.camera_3 = vp.simxGetObjectHandle(clientID, "Right_vision", vp.simx_opmode_oneshot_wait)

        code, self.beacon = vp.simxGetObjectHandle(clientID, "beacon", vp.simx_opmode_blocking)
        code, self.get_dist = vp.simxGetDistanceHandle(clientID, "beacon", vp.simx_opmode_blocking)
        code, self.read_dist = vp.simxReadDistance(clientID, self.get_dist, vp.simx_opmode_streaming)
        code, beacon_distance_robot = vp.simxCheckDistance(clientID, self.p3dx, self.beacon, vp.simx_opmode_streaming)
        code, self.graph = vp.simxGetObjectHandle(clientID, 'Graph', vp.simx_opmode_blocking)


        code, self.min_dist = vp.simxCheckDistance(clientID, self.p3dx, self.beacon, vp.simx_opmode_streaming)
        code, detection_state, packets = vp.simxReadVisionSensor(clientID, self.camera_1, vp.simx_opmode_streaming)
        code, detection_state1, packets1 = vp.simxReadVisionSensor(clientID, self.camera_2, vp.simx_opmode_streaming)
        code, detection_state2, packets2 = vp.simxReadVisionSensor(clientID, self.camera_3, vp.simx_opmode_streaming)





    def move(self): ## method created to move the robot with default speed 1.5 for both motors
        code = vp.simxSetJointTargetVelocity(clientID, self.R_motor, 1.5, vp.simx_opmode_blocking)
        code = vp.simxSetJointTargetVelocity(clientID, self.L_motor, 1.5, vp.simx_opmode_blocking)

    def turn(self, turn_right, turn_left):## method to steer the robot with two attributes left and right steering
        code = vp.simxSetJointTargetVelocity(clientID, self.R_motor, turn_right, vp.simx_opmode_blocking)
        code = vp.simxSetJointTargetVelocity(clientID, self.L_motor, turn_left, vp.simx_opmode_blocking)

    def position(self):  #  used to identify specific points or reference frames in the scene
        code, position = vp.simxGetObjectPosition(clientID, self.pos, -1, vp.simx_opmode_blocking)
        y = np.array(position)
        # return np.linalg.norm(y)
        return position

    def orientation(self):
        code, ori = vp.simxGetObjectOrientation(clientID, self.pos, -1, vp.simx_opmode_streaming)
        return ori

    def set_position(self):
        code = vp.simxSetObjectPosition(clientID, self.pos, -1, [-0.30, 0.30], vp.simx_opmode_oneshot)
        return code


    def set_joint(self):
        code1 = vp.simxSetJointTargetVelocity(clientID, self.R_motor, 0.5, vp.simx_opmode_oneshot)
        code2 = vp.simxSetJointTargetVelocity(clientID, self.L_motor, 0.5, vp.simx_opmode_oneshot)

        code3 = vp.simxSetJointTargetPosition(clientID, self.R_motor, 0.30, vp.simx_opmode_streaming)
        code4 = vp.simxSetJointTargetPosition(clientID, self.L_motor, 0.30, vp.simx_opmode_streaming)
        return  code3, code4


    def wandering(self): ## method to allow the robot wander if no wall detected
        x = random.randint(0, 3)
        y = random.randint(0, 3)
        code = vp.simxSetJointTargetVelocity(clientID, self.R_motor, x, vp.simx_opmode_blocking)
        code = vp.simxSetJointTargetVelocity(clientID, self.L_motor, y, vp.simx_opmode_blocking)

    def distance(self, sensor): ## method to measure the distance with one attribute sensor
        returnCode, detection, detection_point, detection_object_handle, surface_normal  = vp.simxReadProximitySensor(
            clientID, sensor, vp.simx_opmode_buffer)
        x = np.array(detection_point) ## converting the x, y and z values to array. in order to linalg.norm function
        if detection == True:
            return np.linalg.norm(x) ## normalizing the x, y and z coordinates to have a single return value
        else:
            return 1.7

    def avoid_obstcle(self): ## turn robot both sides left or right. it dependes on the sensor reading
        if robot.distance(s_values[2]) < 0.9:
            robot.turn(0,2)
        elif robot.distance(s_values[7]) < 0.9:
            robot.turn(2, 0)
        else:
            robot.turn(0, 1)


    def out_of_square(self):
        # vp.simxSetObjectPosition(clientID, self.p3dx, -1, [0.0250, 0.1750, 0.0000], vp.simx_opmode_oneshot_wait)
        # error, code = vp.simxGetObjectPosition(clientID, self.p3dx, -1, vp.simx_opmode_streaming)
        # x_coordinate = round(code[0], 2)
        # y_coordinate = round(code[1], 2)

        def check_center():
            error, code = vp.simxGetObjectPosition(clientID, self.p3dx, -1, vp.simx_opmode_streaming)
            x_coordinate = round(code[0], 2)
            y_coordinate = round(code[1], 2)
            flagg = True
            if x_coordinate > -0.35 and x_coordinate < 0.20 and y_coordinate > -0.35 and y_coordinate < 0.35 and x_coordinate != 0 and y_coordinate != 0:
                robot.turn(0, 0)
                print("this is the centre!")

                while flagg:
                    if robot.distance(s_values[3]) == 1.7 and robot.distance(s_values[4]) == 1.7:
                        robot.turn(0, 0)
                        for i in range(130):
                            robot.turn(2, 2)

                        robot.turn(0, 0)
                        print("1")
                        flagg = False



                    else:
                        robot.turn(-1, 1.5)
                    print("2")
                print("3")
            print("4")
            print("no")
            ibo = False
            return ibo

        def hor():
                    if robot.distance(s_values[3]) < 1.6 and robot.distance(s_values[4]) < 1.6:
                        robot.turn(-1, -1)
                        print(f" front {robot.distance(s_values[3])} and {robot.distance(s_values[4])}")
                        print("con 1")
                    elif robot.distance(s_values[3]) > 1.9 and robot.distance(s_values[4]) > 1.9:
                        robot.turn(1, 1)
                        print(f" front {robot.distance(s_values[3])} and {robot.distance(s_values[4])}")
                        print("con 2")

                    # else:
                        # robot.turn(0, 0)



        def ver():

                if round(robot.distance(s_values[8]) + robot.distance(s_values[9]), 1) > round(robot.distance(s_values[0]) + robot.distance(s_values[15]),1):
                    print("turn right")
                    robot.turn(2, 2)
                    robot.turn(-1, 1)
                    robot.turn(-1, 1)
                    robot.turn(-1, 1)
                    robot.turn(-1, 1)
                    robot.turn(-1, 1)
                    robot.turn(-1, 1)
                    robot.turn(-1, 1)
                    robot.turn(-1, 1)

                elif round(robot.distance(s_values[8]) + robot.distance(s_values[9]), 1) < round(robot.distance(s_values[0]) + robot.distance(s_values[15]), 1):
                    print("turn left")
                    robot.turn(2, 2)
                    robot.turn(1, -1)
                    robot.turn(1, -1)
                    robot.turn(1, -1)
                    robot.turn(1, -1)
                    robot.turn(1, -1)
                    robot.turn(1, -1)
                    robot.turn(1, -1)
                    robot.turn(1, -1)


        flag = True
        while flag:
            error, code = vp.simxGetObjectPosition(clientID, self.p3dx, -1, vp.simx_opmode_streaming)
            x = round(code[0], 2)
            if x < 1.78 and x > -2.40:
                    print("yes")
                    hor()
                    ver()
                    check_center()
            else:
                flag = False

        print("hiii ")
        return "out of square"




    def robot_position(self):
        code, position = vp.simxGetObjectPosition(clientID, self.p3dx, -1, vp.simx_opmode_blocking)
        # code, angle = vp.simxGetObjectOrientation(clientID, self.p3dx, -1, vp.simx_opmode_streaming)
        # print(position)
        # time.sleep(3)
        # print(round(position[0], 2))
        # print(round(position[1], 2))
        # time.sleep(3)
        # print(f" this is object orientation {round(angle[1], 2)}")
        return round(position[0], 2), round(position[1], 2)

    def vision_sensor(self):
        code, resolution, image = vp.simxGetVisionSensorImage(clientID, self.camera_1, 0, vp.simx_opmode_buffer)
        im = np.array(image, dtype= np.uint8)
        im.resize([resolution[0], resolution[1], 3])
        plt.imshow(im)

        return plt.imshow(im), image


    def detect_red_color(self):
        total_red_pixels = []

        for j in range(0, len(self.vision_sensor()[1]), 3):

            print(f" this is j {self.vision_sensor()[1][j]}")
            total_red_pixels.append(j)
            y = total_red_pixels.count(-1)
            print(f" total if red pixels are {y}")
            print(f" len is {len(total_red_pixels)}")
            print(total_red_pixels)


    def read_vioion(self):

        code, detection_state, packets = vp.simxReadVisionSensor(clientID, self.camera_1, vp.simx_opmode_buffer)
        codd1, detection_state1, packets1 = vp.simxReadVisionSensor(clientID, self.camera_2, vp.simx_opmode_buffer)
        code2, detection_state2, packets2 = vp.simxReadVisionSensor(clientID, self.camera_3, vp.simx_opmode_buffer)

        return packets[0][6], packets1[0][6], packets2[0][6]


    def beacon_position(self):
        code, position = vp.simxGetObjectPosition(clientID, self.beacon, -1, vp.simx_opmode_streaming)

        return position

    def guided_robot(self):
        code = vp.simxSetObjectPosition(clientID, self.p3dx, -1, [4.15, 4.20, 0.40], vp.simx_opmode_oneshot)
        return code

    def wheel_adjustment(self):
        if self.read_vioion()[0] == True:
            adjust_value = self.read_vioion()[1]
            threshold_value = 0.5
            correction = threshold_value - adjust_value
            turninig_rate = 1.1


    def avoid_obstcls(self):
        flag = True
        while flag:
            code, orientation = vp.simxGetObjectOrientation(clientID, self.p3dx, -1, vp.simx_opmode_buffer)
            print(orientation[2])


            print("obstcls in exe")
            # robot.turn(2, 2)
            if robot.distance(s_values[3]) < 0.6:## 212 front left
                robot.turn(-1, 1)
                return True
            elif robot.distance(s_values[4]) < 0.6: ## 216 front right
                robot.turn(1, -1)
                return True
            if robot.distance(s_values[2]) < 0.6: ## 212 front-left
                robot.turn(-1, 1)
                return True
            elif robot.distance(s_values[5]) < 0.6: ## 216 front-right
                robot.turn(1, -1)
                return True
            if robot.distance(s_values[1]) < 0.4: ## 212 left side
                robot.turn(-0.7, 0.7)
                return True
            elif robot.distance(s_values[6]) < 0.4:## 216 right side
                robot.turn(0.7, -0.7)
                return True
            else:
                print("son")
                flag = False
                return False
    def turn_axis(self):
        code, orientation = vp.simxGetObjectOrientation(clientID, self.p3dx, -1, vp.simx_opmode_buffer)
        print(orientation[2])
        while orientation[2] < 2.8 and orientation[2] > -2.8:
            code, orientation = vp.simxGetObjectOrientation(clientID, self.p3dx, -1, vp.simx_opmode_buffer)
            robot.turn(-1, 1)
    def turn_axis_2(self):
        code, orientation = vp.simxGetObjectOrientation(clientID, self.p3dx, -1, vp.simx_opmode_buffer)
        print(orientation[2])
        while orientation[2] < 1.30 and orientation[2] > 1.40:
            code, orientation = vp.simxGetObjectOrientation(clientID, self.p3dx, -1, vp.simx_opmode_buffer)
            robot.turn(-1, 1)


        # return detection_state, packets[0][1], packets[0][6], packets[0][11]
    def return_home(self):
        print("helloooooooo")
        code, position = vp.simxGetObjectPosition(clientID, self.p3dx, -1, vp.simx_opmode_streaming)
        code, orientation = vp.simxGetObjectOrientation(clientID, self.p3dx, -1, vp.simx_opmode_streaming)

        code, position = vp.simxGetObjectPosition(clientID, self.p3dx, -1, vp.simx_opmode_streaming)
        robot.wandering()
        print(position[0])
        if position[0] > -0.50:
            robot.turn_axis()
        flag = True
        while flag:
            code, position = vp.simxGetObjectPosition(clientID, self.p3dx, -1, vp.simx_opmode_streaming)
            robot.avoid_obstcls()
            robot.turn(2, 2)
            if position[0] < -4.00:
                flag = False

        robot.turn_axis_2()
        for i in range(120):
            robot.turn(2, 2)
        return print(position), print(orientation)

    def correct_heading(self):
        code, orientation = vp.simxGetObjectOrientation(clientID, self.p3dx, -1, vp.simx_opmode_buffer)
        code, position = vp.simxGetObjectPosition(clientID, self.p3dx, -1, vp.simx_opmode_streaming)
        print(orientation[2])
        print(position[0])
        ibo = True
        ah = True
        while ibo:
            code, position = vp.simxGetObjectPosition(clientID, self.p3dx, -1, vp.simx_opmode_streaming)
            code, orientation = vp.simxGetObjectOrientation(clientID, self.p3dx, -1, vp.simx_opmode_buffer)

            while ah:
                code, orientation = vp.simxGetObjectOrientation(clientID, self.p3dx, -1, vp.simx_opmode_buffer)
                if orientation[2] < 2.8 and orientation[2] > -2.8:
                    robot.turn(-1, 1)
                if orientation[2] > 2.8 and orientation[2] < 3:
                    robot.turn(0, 0)
                    time.sleep(3)
                    ah = False

            # robot.avoid_obstcls()
            if robot.distance(s_values[3]) < 0.5:
                robot.turn(-1, 1)
            if robot.distance(s_values[2]) < 0.5:
                robot.turn(-1, 1)
            if robot.distance(s_values[4]) < 0.4:
                robot.turn(1, -1)
            if robot.distance(s_values[5]) < 0.4:
                robot.turn(1, -1)
            if robot.distance(s_values[1]) < 0.3:
                robot.turn(-1, 1)
            if robot.distance(s_values[7]) < 0.3:
                robot.turn(1, -1)


            if position[1] >= 5: ## around y axis
                robot.turn(2, 1.8)
            if position[1] < 5:
                robot.turn(1.8, 2)

            if position[0] < -5.00:

                robot.turn(0, 0)
                time.sleep(3)
                # robot.turn(2, -2)
                # robot.turn(2, -2)
                # robot.turn(2, -2)
                # robot.turn(2, -2)
                # robot.turn(2, -2)

                ibo = False
        s = True
        while s:
            code, orientation = vp.simxGetObjectOrientation(clientID, self.p3dx, -1, vp.simx_opmode_buffer)
            robot.turn(1, -1)
            if orientation[2] > -1.8 and orientation[2] < -1.6:
                robot.turn(0, 0)
                time.sleep(1)
                s = False

        ib = True
        while ib:
            code, position = vp.simxGetObjectPosition(clientID, self.p3dx, -1, vp.simx_opmode_streaming)
            if position[0] > -5: ## around x axis
                robot.turn(1.8, 2)
            if position[0] < -5:
                robot.turn(2, 1.8)
            if position[1] <= 0.40:
                robot.turn(0, 0)
                time.sleep(3)
                ib = False
        # robot.turn(2,-2)
        # robot.turn(2, -2)
        # robot.turn(2, -2)
        # robot.turn(2, -2)
        # robot.turn(2, 2)
        # robot.turn(2, 2)
        # robot.turn(2, 2)
        # robot.turn(2, 2)
        # robot.turn(2, 2)
        # robot.turn(2, 2)
        # robot.turn(2, 2)
        # robot.turn(2, 2)
        # robot.turn(2, 2)
        # robot.turn(2, 2)
        ab = True
        while ab:
            code, orientation = vp.simxGetObjectOrientation(clientID, self.p3dx, -1, vp.simx_opmode_buffer)
            robot.turn(1, -1)
            # if orientation[2] == 0:
            #     robot.turn(0,0)
            #     ab = False
            if orientation[2] > -0.3 and orientation[2] < -0.1:
                robot.turn(0, 0)
                time.sleep(1)
                ab = False

        ia = True
        while ia:
            code, position = vp.simxGetObjectPosition(clientID, self.p3dx, -1, vp.simx_opmode_streaming)
            if position[1] > 0.30: ## y axis
                robot.turn(0.9, 1)
            if position[1] < 0.30:
                robot.turn(1, 0.9)
            if position[0] > -0.30:
                robot.turn(0 ,0)
                time.sleep(3)
                print("THE IS THE END!!!")
                ia = False

vp.simxFinish(-1)
clientID = vp.simxStart('127.0.0.1', 700, True, True, 19994, 5)
if clientID != -1:
    print('Connected to API Coppelia server')
    robot = Robot()
    robot.out_of_square()
    print("hello world")
    emp_list = [0]
    # robot.set_object()




    # robot.set_position()
    # robot.set_joint()
    code, resolution, image = vp.simxGetVisionSensorImage(clientID, robot.camera_1, 0, vp.simx_opmode_streaming)
    code, resolution1, image1 = vp.simxGetVisionSensorImage(clientID, robot.camera_2, 0, vp.simx_opmode_streaming)
    code, resolution2, image2 = vp.simxGetVisionSensorImage(clientID, robot.camera_3, 0, vp.simx_opmode_streaming)
    code, detection, packet = vp.simxReadVisionSensor(clientID, robot.camera_1, vp.simx_opmode_buffer)
    ibo = True
    while ibo:
        # robot.turn(0, 0)
        # robot.wandering
        code, orientation = vp.simxGetObjectOrientation(clientID, robot.p3dx, -1, vp.simx_opmode_streaming)
        print(orientation[2])

        if robot.avoid_obstcls() == True:
            robot.avoid_obstcls()


        # robot.avoid_obstcls()
        if robot.read_vioion()[0] == 1 or robot.read_vioion()[1] == 1 or robot.read_vioion()[2] == 1:
            print("object inside frame")
            code, position_of_beacon = vp.simxGetObjectPosition(clientID, robot.beacon, -1, vp.simx_opmode_streaming)
            flage = True
            while flage:

                code, beacon_robot_dist = vp.simxCheckDistance(clientID, robot.p3dx, robot.beacon  , vp.simx_opmode_buffer)
                # code, position_of_beacon = vp.simxGetObjectPosition(clientID, robot.beacon, -1, vp.simx_opmode_buffer)
                print(f" distance to robot {beacon_robot_dist}")
                robot.turn(2, 2)

                if beacon_robot_dist < 0.6:
                    robot.turn(0, 0)
                    robot.turn(0, 0)
                    robot.turn(0, 0)
                    robot.turn(0, 0)
                    robot.turn(0, 0)
                    robot.turn(0, 0)
                    robot.turn(0, 0)
                    robot.turn(0, 0)
                    robot.turn(0, 0)
                    robot.turn(0, 0)


                    flage = False
                    ibo = False

                if robot.avoid_obstcls() == True and robot.read_vioion()[0] == 1:
                    robot.avoid_obstcls()
                if robot.avoid_obstcls() == True and robot.read_vioion()[1] == 1:
                    robot.avoid_obstcls()
                if robot.avoid_obstcls() == True and robot.read_vioion()[2] == 1:
                    robot.avoid_obstcls()
                if robot.avoid_obstcls() == False:
                    # print(position_of_beacon)

                    if robot.read_vioion()[0] == 1 and robot.read_vioion()[1] == 1: ## object out of frame
                        robot.turn(2, -1)
                        robot.turn(2, 1)
                        # robot.turn(2, -2)
                        print("turn left")
                    if robot.read_vioion()[0] == 1 and robot.read_vioion()[2] == 1:
                        robot.turn(-1, 2)
                        robot.turn(1, 2)
                        # robot.turn(-2, 2)
                        print("turn right")
                    if robot.read_vioion()[0] == 1 and robot.read_vioion()[1] != 1 and robot.read_vioion()[2] != 1:
                        robot.turn(2, 2)
                        print("go ahead")


                    if robot.read_vioion()[1] == 1 and robot.read_vioion()[0] != 1:
                        robot.turn(2, -1)
                        robot.turn(2, 1)
                        # robot.turn(2, -2)
                        # robot.turn(2, -2)
                        print("turn left")
                    if robot.read_vioion()[2] == 1 and robot.read_vioion()[0] != 1:
                        robot.turn(-1, 2)
                        robot.turn(1, 2)
                        # robot.turn(-2, 2)
                        # robot.turn(-2, 2)
                        print("turn right")


        else:
            robot.wandering()
            print("Wandering")
    robot.turn(-2, -2)
    robot.turn(-2, -2)
    robot.turn(-2, -2)
    robot.turn(-2, -2)
    robot.turn(-2, -2)
    robot.turn(-2, -2)
    robot.turn(-2, -2)

    robot.correct_heading()
    print("THIS IS THE END")

    # robot.find_centre()
    # robot.return_home()
else:
    print('Not able to connect to API!')