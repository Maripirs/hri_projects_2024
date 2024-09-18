class Limb:
    def __init__(self, name,initial, final, rate):
        self.name = name
        self.initial_position = initial
        self.final_position = final
        self.current_position = initial
        self.rate = rate
        self.current_direction = 1
        self.total_distance = self.final_position - self.initial_position
        self.step = self.total_distance / self.rate

    def displayData(self):
        print(f'This limb is a {self.name}')
        print(f'Current position is {self.current_position}')
        print(f'Moving at a rate of {self.rate} cycle(s) per second')

    def move(self):
        new_position = self.current_position + (self.step * self.current_direction)
        if new_position > self.final_position or new_position < self.initial_position:
            self.current_direction*= -1
            new_position = self.current_position + (self.step * self.current_direction)
        
        self.current_position = new_position
        print(f'{self.name} moved to a new position {self.current_position}. Direction is {self.current_direction}')
    def change_rate(self, rate):
        self.rate = rate
        self.step = self.total_distance / self.rate

def main():
    print("Hello world")
    shoulder_joint = Limb('Shoulder Joint', 0, 100, 5)
    shoulder_joint.displayData()
    elbow_joint = Limb('Elbow Joint', 40, 150, 10)
    joints = [ elbow_joint]
        

    for i in range (50):
        for element in joints:
            element.move()
    shoulder_joint.change_rate(10)
    for i in range (50):
        for element in joints:
            element.move()


main()