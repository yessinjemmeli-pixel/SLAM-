import pymunk
import pygame
import math

ROBOT_SIZE = (80, 80)
DIRECTION_LINE_LENGTH = 30
ROBOT_CORNER_RADIUS = 1  #smooth round corners instead of sharp ones for better simulation


class Robot():

    def __init__(self, space: pymunk.Space) -> None:
        self.body = pymunk.Body()
        self.box_shape = pymunk.Poly.create_box(self.body, ROBOT_SIZE, ROBOT_CORNER_RADIUS)
        self.box_shape.mass = 1
        
        # On met une catégorie pour différencier les collisions des rays du Lidar avec le robot
        # et les autres objets du décor
        self.box_shape.filter = pymunk.ShapeFilter(categories=0b01)

        _D = 1.0 / math.sqrt(2)  # normalisation

        self.wheel1 = Wheel(self, ( ROBOT_SIZE[0]/2, -ROBOT_SIZE[0]/2), pymunk.Vec2d( _D,  _D))  # FL
        self.wheel2 = Wheel(self, ( ROBOT_SIZE[0]/2,  ROBOT_SIZE[0]/2), pymunk.Vec2d( _D, -_D))  # FR
        self.wheel3 = Wheel(self, (-ROBOT_SIZE[0]/2,  ROBOT_SIZE[0]/2), pymunk.Vec2d(-_D, -_D))  # BR  (ou +_D,+_D selon sens)
        self.wheel4 = Wheel(self, (-ROBOT_SIZE[0]/2, -ROBOT_SIZE[0]/2), pymunk.Vec2d(-_D,  _D))  # BL
        self.wheels = [self.wheel1, self.wheel2, self.wheel3, self.wheel4]

        self.space = space
        self.space.add(self.body, self.box_shape)

    def update(self, dt):
        for w in self.wheels:
            w.apply_force(dt)

    def update_wheel_speeds(self, wheel1_speed: float, wheel2_speed: float, wheel3_speed: float, wheel4_speed: float):
        self.wheel1.speed = wheel1_speed
        self.wheel2.speed = wheel2_speed
        self.wheel3.speed = wheel3_speed
        self.wheel4.speed = wheel4_speed

    def get_direction(self):
        return pymunk.Vec2d(1, 0).rotated(self.body.angle)

    def draw(self, screen):
        world_vertices = [tuple(self.body.position + v.rotated(self.body.angle)) for v in self.box_shape.get_vertices()]

        pygame.draw.polygon(screen, (255, 255, 255), world_vertices)

        pygame.draw.line(screen, (0, 200, 255), tuple(self.body.position),
                         tuple(self.body.position + self.get_direction() * DIRECTION_LINE_LENGTH), 3)


class Wheel():

    def __init__(self, robot, local_position, direction=pymunk.Vec2d(1, 0)):
        self.local_position = local_position
        self.direction = direction
        self.robot = robot
        self.speed = 0.

    def apply_force(self, dt):
        current_velocity = self.robot.body.velocity_at_local_point(self.local_position)
        target_velocity = self.speed * self.direction

        force = (target_velocity - current_velocity) * dt
        self.robot.body.apply_force_at_local_point(force, self.local_position)
        # self.robot.body.velocity_at_local_point()
