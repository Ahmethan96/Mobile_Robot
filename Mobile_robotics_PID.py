import numpy as np
import random
import sim as vp
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

        code, self.ibo = vp.simxGetObjectHandle(clientID, "Dummy", vp.simx_opmode_blocking)

        # activating left and right motors
        code, self.R_motor = vp.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor",
                                                    vp.simx_opmode_blocking)
        code, self.L_motor = vp.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor",
                                                    vp.simx_opmode_blocking)
    def move(self): ## method created to move the robot with default speed 1.5 for both motors
        code = vp.simxSetJointTargetVelocity(clientID, self.R_motor, 1.5, vp.simx_opmode_blocking)
        code = vp.simxSetJointTargetVelocity(clientID, self.L_motor, 1.5, vp.simx_opmode_blocking)

    def turn(self, turn_right, turn_left):## method to steer the robot with two attributes left and right steering
        code = vp.simxSetJointTargetVelocity(clientID, self.R_motor, turn_right, vp.simx_opmode_blocking)
        code = vp.simxSetJointTargetVelocity(clientID, self.L_motor, turn_left, vp.simx_opmode_blocking)

    def position(self):  #  used to identify specific points or reference frames in the scene
        code, position = vp.simxGetObjectPosition(clientID, self.ibo,-1,vp.simx_opmode_blocking)
        y = np.array(position)
        return np.linalg.norm(y)


    def wandering(self): ## method to allow the robot wander if no wall detected
        x = random.randint(0, 6)
        y = random.randint(0, 6)
        code = vp.simxSetJointTargetVelocity(clientID, self.R_motor, x, vp.simx_opmode_blocking)
        code = vp.simxSetJointTargetVelocity(clientID, self.L_motor, y, vp.simx_opmode_blocking)



    def distance(self, sensor): ## method to measure the distance with one attribute sensor
        returnCode, detection, detection_point, detection_object_handle, surface_normal  = vp.simxReadProximitySensor(
            clientID, sensor, vp.simx_opmode_buffer)
        x = np.array(detection_point) ## converting the x, y and z values to array. in order to linalg.norm function
        if detection == True:
            return np.linalg.norm(x) ## normalizing the x, y and z coordinates to have a single return value
        else:
            return 0

    def error(self, Sp, Pv): ## error = setpoint - process value "sensor[7] readings"
        error = Sp - Pv
        return float(error)

    def proportional(self, error): ## applying the proportional component
        P = p_gain * error
        return P

    def integral(self, total_error): ## applying the integral component
        sumation = total_error
        I = i_gain * sumation
        # print(f" sumation is {sumation}")
        if I > -0.65 and I < 0.65: ## limiting the I component to avoid constant accumulation effect
            return I
        else:
            return 0
        # error_sum =+ total_error
        # integ = i_gain * error_sum
        # if integ > 0.01:
        #     error_sum = 0
        #     return integ
        # else:


    def derivative(self,previous_error, current_error): ##applying the derivative component
        error_difference = current_error - previous_error
        D = d_gain * error_difference
        return D




# set_point = 0.9
# current_value = Robot().distance()
# error_sum = [0, 0]

## PID constants (gains)
p_gain = 6
i_gain = 0.5
d_gain = 6
Sp= 0.6
vp.simxFinish(-1)
clientID = vp.simxStart('127.0.0.1', 778, True, True, 5000, 5)
if clientID != -1:
    print('Connected to API Coppelia server')
    robot = Robot()
    emp_list = [0]
    while True:


        # print(f" erorr sum {error_sum}")
        E = robot.error(Sp, robot.distance(s_values[7])) ## error in the system from sensor 7
        emp_list.append(E) ## appending the error the the emp_list to use it later in the D component
        P = robot.proportional(E)
        x = sum(emp_list)
        I = robot.integral(x)
        D = robot.derivative(emp_list[-2], E) ## since D component is the current error - previous error
                                                ## for that reason emp_list[-2] is will give the previous error
                                                    ## and E is the current error
        # print(emp_list)
        ## different combinations of PID controller
        PI = P + I
        PD = P + D
        PID = P + I + D
        print(f" ibooooo {robot.integral(robot.distance(s_values[7]))}")

        # print(f" this is distance {round(robot.distance(s_values[7]), 3)}")
        # time.sleep(2)
        # print(f" this is E {round(E, 3)}")
        # time.sleep(2)
        # print(f" this is P {round(P, 3)}")
        # time.sleep(2)
        # print(f" this is error sum {(error_sum)}")
        # time.sleep(2)
        # print(f" this is I {round(I, 3)}")
        # time.sleep(2)
        # print(f" this is D {round(D, 3)}")
        # time.sleep(2)
        # print(f" this is PID {PID}")
        # time.sleep(2)


        if robot.distance(s_values[4]) < 0.9 and robot.distance(s_values[4]) != 0:
            robot.turn(1.2, -1.2)
            print("first loop +")
        elif robot.distance(s_values[3]) < 0.7 and robot.distance(s_values[3]) != 0:
            robot.turn(1.2, -1.2)
            print("first loop ++")


        elif robot.distance(s_values[7]) < Sp and robot.distance(s_values[7]) != 0:
            robot.turn(2 + abs(round(PID,20)), 2)
            print(PID)
            print("left steering")

        elif robot.distance(s_values[7]) > Sp and robot.distance(s_values[7]) != 0:
            robot.turn(2, 2 + (abs(round(PID, 20))))
            print(PID)
            # print(robot.distance(s_values[5]))
            print("right steering")

        elif robot.distance((s_values[8])) < 2 and robot.distance(s_values[8]) != 0: ## make a U turn
            print("sensor 8")
            robot.turn(-2, 2)
            robot.turn(-2, 2)
            # robot.turn(0, 3)

        elif robot.distance(s_values[9]) < 2 and robot.distance(s_values[9]) != 0:
            print("sensor 9")
            robot.turn(-2, 2)

        elif robot.distance(s_values[10]) < 2 and robot.distance(s_values[10]) != 0:
            print("sensor 10")
            robot.turn(-2, 2)
        elif robot.distance(s_values[11]) < 2 and robot.distance(s_values[11]) != 0:
            print("sensor 11")
            robot.turn(0.4, 1.5)


        else:
                robot.wandering()
else:
    print('Not able to connect to API!')