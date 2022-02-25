import pybullet as p
import time
import math
import numpy as np

p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -10)
useRealTimeSim = 0

# p.setTimeStep(1. / 120.)
p.setRealTimeSimulation(useRealTimeSim)
track = p.loadURDF("data/plane/plane.urdf")
car = p.loadURDF("f10_racecar/racecar_differential.urdf", [0, 0, 0])


def random_obstacles():
    np.random.seed()
    xy_position = [0, 0]
    xy_position_float = np.random.rand(2)
    xy_position[0] = xy_position_float[0] + np.random.randint(1, 10)
    xy_position[1] = xy_position_float[1] + np.random.randint(1, 10)
    np.asarray(xy_position)
    #
    position = np.append(xy_position, 0.5)
    return position


for i in range(8):
    p.loadURDF('data/cube/marble_cube.urdf', random_obstacles())

for joint in range(p.getNumJoints(car)):
    p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    p.getJointInfo(car, joint)

wheels = [8, 15]

# p.setJointMotorControl2(car,10,p.VELOCITY_CONTROL,targetVelocity=1,force=10)
c = p.createConstraint(car, 9, car, 11, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 10, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 9, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 16, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 16, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 17, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 1, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
c = p.createConstraint(car, 3, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

steering = [0, 2]

hokuyo_joint = 4

replaceLines = True
#
numRays = 100
rayFrom = []
rayTo = []
rayIds = []
rayHitColor = [1, 0, 0]
rayMissColor = [0, 1, 0]
rayLen = 8
rayStartLen = 0.25
for i in range(numRays):
    rayFrom.append([rayStartLen * math.sin(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays),
                    rayStartLen * math.cos(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays), 0])
    rayTo.append([rayLen * math.sin(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays),
                  rayLen * math.cos(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays), 0])
    if replaceLines:
        rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, parentObjectUniqueId=car,
                                         parentLinkIndex=hokuyo_joint))
    else:
        rayIds.append(-1)

# frame = 0
# lineId = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
# lineId2 = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
# lineId3 = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
# print("lineId=", lineId)
lastTime = time.time()
lastControlTime = time.time()
lastLidarTime = time.time()


def drive_the_mobile(carPos, turn_angle, final_goal_pos):
    steeringAngle = 1 * turn_angle
    if steeringAngle > 1:
        steeringAngle = 1
    if steeringAngle < -1:
        steeringAngle = -1

    distance = np.sqrt(np.square(final_goal_pos[0] - carPos[0]) + np.square(final_goal_pos[1] - carPos[1]))
    maxForce = 20
    stop_threshold = 1
    if distance < stop_threshold:
        targetVelocity = 0
    else:
        targetVelocity = 50

    for wheel in wheels:
        p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=targetVelocity, force=maxForce)
    for steer in steering:
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)


def navigate_mobile(carPos, carOrn, sensor_readings, targetPos):
    carEuler = p.getEulerFromQuaternion(carOrn)
    carYaw = carEuler[2]
    hitTo_angle = []
    hitTo_Fraction = []

    angle_from_target_to_car = np.arctan2((targetPos[1] - carPos[1]), (targetPos[0] - carPos[0])) - carYaw
    if angle_from_target_to_car < -math.pi:
        angle_from_target_to_car = angle_from_target_to_car + 2 * math.pi
    if angle_from_target_to_car > math.pi:
        angle_from_target_to_car = angle_from_target_to_car - 2 * math.pi

    for i, sensor_reading in enumerate(sensor_readings):
        hitFraction = sensor_reading[2]
        localHitTo = [rayFrom[i][0] + hitFraction * (rayTo[i][0] - rayFrom[i][0]),
                      rayFrom[i][1] + hitFraction * (rayTo[i][1] - rayFrom[i][1]),
                      rayFrom[i][2] + hitFraction * (rayTo[i][2] - rayFrom[i][2])]
        rayangle_in_car = np.arctan2((localHitTo[1] - rayFrom[i][1]), (localHitTo[0] - rayFrom[i][0]))
        hitTo_angle.append(rayangle_in_car)
        hitTo_Fraction.append(hitFraction)
    min_angle_difference = 2 * math.pi
    min_rayIndex = np.round(numRays / 2)
    for i in range(0, len(hitTo_Fraction)):
        if hitTo_Fraction[i] > 0.7:
            angle_difference = abs(hitTo_angle[i] - angle_from_target_to_car)
            if angle_difference < min_angle_difference:
                min_angle_difference = angle_difference
                min_rayIndex = i
        elif hitTo_Fraction[i] > 0.5:
            angle_difference = abs(hitTo_angle[i] - angle_from_target_to_car)
            if angle_difference < min_angle_difference:
                min_angle_difference = angle_difference
                min_rayIndex = i
        elif hitTo_Fraction[i] > 0.6:
            angle_difference = abs(hitTo_angle[i] - angle_from_target_to_car)
            if angle_difference < min_angle_difference:
                min_angle_difference = angle_difference
                min_rayIndex = i
        else:
            angle_difference = abs(hitTo_angle[i] - angle_from_target_to_car)
            if angle_difference < min_angle_difference:
                min_angle_difference = angle_difference
                min_rayIndex = i

    return hitTo_angle[min_rayIndex]


# frame = 0
p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=-60, cameraPitch=271, cameraTargetPosition=[4, 4, 8])
# time.sleep(5)
p.stepSimulation()
time.sleep(5)
while (True):
    nowTime = time.time()
    # render Camera at 10Hertz
    if (nowTime - lastTime > .1):
        lastTime = nowTime

    nowControlTime = time.time()

    nowLidarTime = time.time()
    # lidar at 20Hz
    if (nowLidarTime - lastLidarTime > .03):
        numThreads = 0
        results = p.rayTestBatch(rayFrom, rayTo, numThreads, parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
        final_goal_pos = [11, 11]
        carPos, carOrn = p.getBasePositionAndOrientation(car)
        # target_ori = [0, 0]
        # maxForce = 20
        # targetVelocity = 0
        # steeringAngle = 0
        # turn_angle = 0

        for i in range(numRays):
            hitObjectUid = results[i][0]
            hitFraction = results[i][2]
            hitPosition = results[i][3]
            if (hitFraction == 1.):
                p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i],
                                   parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
            else:
                localHitTo = [rayFrom[i][0] + hitFraction * (rayTo[i][0] - rayFrom[i][0]),
                              rayFrom[i][1] + hitFraction * (rayTo[i][1] - rayFrom[i][1]),
                              rayFrom[i][2] + hitFraction * (rayTo[i][2] - rayFrom[i][2])]
                # print(localHitTo)
                p.addUserDebugLine(rayFrom[i], localHitTo, rayHitColor, replaceItemUniqueId=rayIds[i],
                                   parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
        lastLidarTime = nowLidarTime
        test_angle = navigate_mobile(carPos, carOrn, results, final_goal_pos)
        drive_the_mobile(carPos, test_angle, final_goal_pos)

        # control at 100Hz
        # if (nowControlTime - lastControlTime > .01):

        if (useRealTimeSim == 0):
            # frame += 1
            p.stepSimulation()
        lastControlTime = nowControlTime
