#!/usr/bin/env python3

import ntcore
import time
from dumbdashboard import DumbDashboard

if __name__ == "__main__":
    DumbDashboard.inst.setServerTeam(3100)
    DumbDashboard.inst.setServer('localhost')
    DumbDashboard.inst.startClient4("dumbclient")
    # DumbDashboard.inst.startDSClient()
    # xSub = table.getDoubleTopic("x").subscribe(0)
    # ySub = table.getDoubleTopic("y").subscribe(0)
    while True:
        in_ = input('> ')
        parsed = (in_.split() + 5 * [None])[:5]
        if parsed[0] == 'set':
            if parsed[3]:
                type_=parsed[3]
            else: type_=str
            DumbDashboard.put(parsed[1], parsed[2], type_=type_)
        elif parsed[0] in ['exit', '\u0004']:
            quit()
        else:
            print("Bad")
            continue
