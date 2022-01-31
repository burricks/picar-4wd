import picar_4wd as fc
import time
import random
from picar_4wd.speed import Speed

def main():
    while True:
        goTill()
        backUp()
        turnRandomly()

def turnRandomly():
    speed4 = Speed(25)
    speed4.start()
    options = [fc.turn_left, fc.turn_right]
    random.choice(options)(100)
    x = 0
    for i in range(random.randint(3,7)):
        time.sleep(0.1)
        speed = speed4()
        x += speed * 0.1
        #print ("%smm/s"%speed)
    #print("%smm"%x)
    speed4.deinit()
    fc.stop()   

def backUp():
    speed4 = Speed(25)
    speed4.start()
    fc.backward(100)
    x = 0
    for i in range(2):
        time.sleep(0.1)
        speed = speed4()
        x += speed * 0.1
        #print ("%smm/s"%speed)
    #print("%smm"%x)
    speed4.deinit()
    fc.stop()

def goTill():
    while fc.get_distance_at(0) > 20:
        fc.forward(20)

if __name__ == '__main__':
    main()
