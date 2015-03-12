#!/usr/bin/env python
from demo import Demo

if __name__ == '__main__':
    demo = Demo(
        [
            #x   ,   y,   z, yaw, sleep
            [-1.5 , 1.0, 0.2, 0, 10],
            [-1.0 , 1.0, 0.3, 0, 10],
            [0.0 , 0.5, 0.5, 0, 0],
        ]
    )
    demo.run()
