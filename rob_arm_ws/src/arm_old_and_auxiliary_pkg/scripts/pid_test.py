from simple_pid import PID

def controller_pid(q_target):
    pid = []
    pid.append(PID(10, 1, 1, q_target[0]))
    pid.append(PID(10, 1, 1, q_target[1]))
    pid.append(PID(10, 1, 1, q_target[2]))
    pid.append(PID(10, 1, 1, q_target[3]))
    pid.append(PID(10, 1, 1, q_target[4]))
    pid.append(PID(10, 1, 1, q_target[5]))

    return pid



    