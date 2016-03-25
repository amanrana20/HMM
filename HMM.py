import numpy as np
import random
from enum import Enum


class Directions(Enum):
    NORTH = 'NORTH'
    SOUTH = 'SOUTH'
    EAST = 'EAST'
    WEST = 'WEST'


class Moves(Enum):
    AHEAD = 'AHEAD'
    BACK = 'BACK'
    LEFT = 'LEFT'
    RIGHT = 'RIGHT'


class HMM:
    def __init__(self, h=25, w=25):
        # Reference Matrix
        self.ref_matrix = np.zeros([h, w])
        counter = 0
        for i in range(self.ref_matrix.__len__()):
            for j in range(self.ref_matrix[i].__len__()):
                self.ref_matrix[i][j] = counter
                counter += 1

        self.transition_matrix = self.create_transition_matrix(h, w)
        self.transition_matrix = self.populate_transition_matrix(self.transition_matrix, self.ref_matrix)

        self.sensor_matrix = self.create_sensor_matrix(h, w)

        self.f = np.ones(shape=(h*w*4)) / (h*w*4)

        print('Size of Transition matrix, T: ', self.transition_matrix.__len__(), 'x', self.transition_matrix[0].__len__())
        print('Size of Sensor matrix, O: ', self.sensor_matrix.__len__(), 'x', self.sensor_matrix[0].__len__())
        print('Size of Reference matrix, ref_matrix: ', self.ref_matrix.__len__(), 'x', self.ref_matrix[0].__len__())

    def create_transition_matrix(self, h, w):
        T = np.zeros([h * w * 4, w * h * 4])
        return T

    def create_sensor_matrix(self, h, w):
        sensor_matrix = np.zeros([h * w * 4, w * h * 4])
        return sensor_matrix

    def populate_transition_matrix(self, transition_matrix, ref_matrix):
        for i in range(ref_matrix.__len__()):
            for j in range(ref_matrix[i].__len__()):

                for x in range(ref_matrix.__len__()):
                    for y in range(ref_matrix[x].__len__()):

                        if ((x - i) ** 2 + (y - j) ** 2) ** (0.5) == 1:  # Taking the Euclidean distance to find the neighbours
                            # Going North
                            if i - x == 1 and j - y == 0:
                                direction_to = 0  # North

                            # Going South
                            if x - i == 1 and y - j == 0:
                                direction_to = 1  # South

                            # Going East
                            if x - i == 0 and y - j == 1:
                                direction_to = 2  # East

                            # Going West
                            if i - x == 0 and j - y == 1:
                                direction_to = 3  # West

                            for direction_from in range(4):
                                transition_matrix[ref_matrix[i][j]*4 + direction_from][ref_matrix[x][y]*4 + direction_to] = self.calculate_probability(direction_from, direction_to)

        return transition_matrix

    def calculate_probability(self, direction_from, direction_to):
        if direction_from != direction_to:
             return 0.067
        else:
            return 0.8

    def update_sensor_matrix(self, ref_matrix, x, y):
        for i in range(ref_matrix.__len__()):
            for j in range(ref_matrix[i].__len__()):
                sensed_pos_probability = 0
                pos_x = 0
                pos_y = 0

                ## Find the probability of the sensed position
                # Finding the Euclidean distance to calculate the probability
                dist = ((x - i)**2 + (y - j)**2)**0.5

                if dist == 0:
                    sensed_pos_probability = 0.1
                if dist > 0 and dist < 2:
                    sensed_pos_probability = 0.05
                if dist >= 2 and dist <3:
                    sensed_pos_probability = 0.025
                if  dist > 3:
                    sensed_pos_probability = 0

                # Update the sensor_matrix
                pos = ref_matrix[i][j] * 4

                for k in range(4):
                    self.sensor_matrix[pos + k][pos + k] = sensed_pos_probability

        return sensor_matrix


class Robot:
    def __init__(self, x, y, number_of_moves):
        #calculating the bounds for protection against Walls
        self.reference_matrix_size = HMM().ref_matrix.__len__(), HMM().ref_matrix[0].__len__()
        self.robot_facing = Directions.NORTH
        self.available_moves = [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]
        self.current_position = x, y
        self.update_position(x, y, self.robot_facing, number_of_moves)

    def update_position(self, x, y, robot_facing, number_of_moves):
        for move in range(number_of_moves):
            random_number = random.random()

            self.available_moves = self.calculate_available_moves(x, y, robot_facing)

            if random_number <= 0.8:
                if self.robot_facing in self.available_moves:
                    self.move(self.robot_facing, x, y)
            if random_number > 0.8:
                rand_num = random.random()
                if rand_num <= 0.3 and Directions.SOUTH in self.available_moves:
                    self.move(Directions.SOUTH, x, y)
                else:
                    if rand_num > 0.3 and rand_num <= 0.67 and Directions.EAST in self.available_moves:
                        self.move(Directions.EAST, x, y)
                    else:
                        if rand_num > 0.67 and Directions.WEST in self.available_moves:
                            self.move(Directions.WEST, x, y)

    def move(self, direction, x, y):
        if direction == Directions.NORTH:
            self.current_position = x - 1, y
        if direction == Directions.SOUTH:
            self.current_position = x + 1, y
            self.robot_facing = Directions.SOUTH
        if direction == Directions.EAST:
            self.current_position = x, y + 1
            self.robot_facing = Directions.EAST
        if direction == Directions.WEST:
            self.current_position = x, y - 1
            self.robot_facing = Directions.WEST

    def calculate_available_moves(self, x, y, robot_facing):
        if x == 0 and robot_facing == Directions.NORTH:
            self.available_moves.remove(robot_facing)
        if x == self.reference_matrix_size[0] and robot_facing == Directions.SOUTH:
            self.available_moves.remove(robot_facing)
        if y == 0 and robot_facing == Directions.WEST:
            self.available_moves.remove(robot_facing)
        if y == self.available_moves[1] and robot_facing == Directions.EAST:
            self.available_moves.remove(robot_facing)

    def get_sensed_position(self, x, y):
        rand_num = random.random()

        if rand_num <= 0.1:
            self.current_position = x, y
        else:
            if rand_num > 0.1 and rand_num <= 0.2:
                self.current_position = x, y
            else:
                if rand_num > 0.2 and rand_num <= 0.6:
                    rand_pos = random.randint(-1, 1), random.randint(-1, 1)
                    if rand_pos != 0:
                        self.current_position = rand_pos
                else:
                    if rand_num > 0.6 and rand_num <= 1.0:
                        rand_pos = random.randint(-2, 2), random.randint(-2,2)
                        if rand_pos in [(-2, -2), (-2, -1), (-2, 0), (-2, 1), (-2, 2), (-1, -2), (0, -2), (1, -2), (2, -2), (2, -1), (2, 0), (2, 1), (2, 2), (-1, 2), (0, 2), (1, 2)]:
                            self.current_position = rand_pos


HEIGHT, WIDTH = 10, 10

hmm = HMM(WIDTH, HEIGHT)
robot = Robot(5, 5, 10)

transition_matrix = hmm.transition_matrix
sensor_matrix = hmm.sensor_matrix

hmm.update_sensor_matrix(hmm.ref_matrix, 1, 0)
# for i in range(transition_matrix.__len__()):
#     line = ''
#     for j in range(transition_matrix[i].__len__()):
#         line += '%s\t' % transition_matrix[i][j]
#     print('\n')
# print('\n')
#
# for i in range(sensor_matrix.__len__()):
#     line = ''
#     for j in range(sensor_matrix[i].__len__()):
#         line += '%s\t' % sensor_matrix[i][j]
#     print('\n')
# print('\n')

file = open('output.txt', 'w')
file.write('Transition Matrix\n')
for i in range(transition_matrix.__len__()):
    line = ''
    for j in range(transition_matrix[i].__len__()):
        line += '%s\t' %transition_matrix[i][j]
    file.write(line)
    file.write('\n')
file.write('\n\n')

file.write('Sensor Matrix\n')
for i in range(sensor_matrix.__len__()):
    line = ''
    for j in range(sensor_matrix[i].__len__()):
        line += '%s\t' %sensor_matrix[i][j]
    file.write(line)
    file.write('\n')
file.write('\n\n')

# f = hmm.f
# f_newn = np.dot(transition_matrix, f)
# for i in range(f_newn.__len__()):
#     item = '%s\n' %f_newn[i]
#     file.write(item)

