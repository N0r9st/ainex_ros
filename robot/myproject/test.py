import sys
sys.path.append('/home/ubuntu/software/ainex_controller')
import bus_servo_control as bsc
import time
import os
import sqlite3 as sql

def main():
    id = 23

    dev = bsc.getBusServoDeviation(id)
    pulse = bsc.getBusServoPulse(id)
    print(dev, pulse)

    # bsc.serial_servo_wirte_cmd(id, bsc.LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 0)

    # print("unloaded!")
    # time.sleep(4)

    # bsc.serial_servo_wirte_cmd(id, bsc.LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 1)
    # print("Loaded!")
    poses = [250, 750] * 2

    poses.append(500)
    time_ms = 500
    for pos in poses:
        out = bsc.setBusServoPulse(id, pos, time_ms)
        time.sleep(time_ms / (1000))
        print(bsc.getBusServoPulse(id))
    time.sleep(1)
    print(bsc.getBusServoPulse(id))

    bsc.unloadBusServo(id)
    return

def da6():
    actNum = '/home/ubuntu/software/ainex_controller/action_bk/go_forward_low.d6a'
    ag = sql.connect(actNum)
    cu = ag.cursor()
    cu.execute("select * from ActionGroup")
    for _ in range(10):
        act = cu.fetchone()
        if act is not None:
            for i in range(0, len(act)-2, 1):
                print(i+1, act[2 + i], act[1])
            # for i in range(0, len(act)-2, 1):
            #     setBusServoPulse(i+1, act[2 + i], act[1])
            time.sleep(float(act[1])/1000.0)
            print('------')
        else:   # 运行完才退出
            break
if __name__ == '__main__':
    main()
    # da6()