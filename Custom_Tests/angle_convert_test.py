import math

def main():
    a, b, theta = (16.7042, 18.2046, 8.75)
    angle = 75
    
    # Law of cosines
    actuator_length = math.sqrt(math.pow(a, 2) + math.pow(b, 2) - (2 * a * b * math.cos(180 - angle - theta)))
    print(actuator_length)


if __name__=='__main__':
    main()
