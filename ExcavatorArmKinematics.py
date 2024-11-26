import pygame
import math

pygame.init()

# Screen settings
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Excavator Arm Kinematics 2D")

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

# Excavator arm segments lengths (relative)
BOOM_LENGTH = 150
STICK_LENGTH = 120
BUCKET_LENGTH = 50

# Initial joint angles (in degrees)
boom_angle = 87  # Angle of the boom from the horizontal (upwards)
stick_angle = -15  # Angle of the stick relative to the boom (extends outwards)
bucket_angle = 20  # Angle of the bucket relative to the stick

# The base position of the excavator (fixed)
base_x, base_y = 350, 350

# Constraints
BOOM_MIN_ANGLE = 0      
BOOM_MAX_ANGLE = 87

STICK_MIN_ANGLE = -110   
STICK_MAX_ANGLE = -15

BUCKET_MIN_ANGLE = -110  
BUCKET_MAX_ANGLE = 20

def deg_to_rad(deg):
    # Coversion of degrees to radians
    return deg * math.pi / 180

def calculate_joint_positions():
    # Calculate the (x, y) coordinates of each joint and part of the arm
    
    # Boom end position (using boom_angle)
    boom_end_x = base_x + BOOM_LENGTH * math.cos(deg_to_rad(boom_angle))
    boom_end_y = base_y - BOOM_LENGTH * math.sin(deg_to_rad(boom_angle))
    
    # Stick end position (using stick_angle relative to boom)
    stick_end_x = boom_end_x + STICK_LENGTH * math.cos(deg_to_rad(boom_angle + stick_angle))
    stick_end_y = boom_end_y - STICK_LENGTH * math.sin(deg_to_rad(boom_angle + stick_angle))
    
    # Bucket end position (using bucket_angle relative to stick)
    bucket_end_x = stick_end_x + BUCKET_LENGTH * math.cos(deg_to_rad(boom_angle + stick_angle + bucket_angle))
    bucket_end_y = stick_end_y - BUCKET_LENGTH * math.sin(deg_to_rad(boom_angle + stick_angle + bucket_angle))
    
    return (boom_end_x, boom_end_y), (stick_end_x, stick_end_y), (bucket_end_x, bucket_end_y)

def draw_arm():
    # Draw the excavator arm on the screen
    # Calculate joint positions
    boom_end, stick_end, bucket_end = calculate_joint_positions()
    
    # Clear the screen
    screen.fill(WHITE)
    
    # Draw the base of the excavator
    pygame.draw.circle(screen, BLACK, (base_x, base_y), 10)
    
    # Draw the boom (from base to boom end)
    pygame.draw.line(screen, BLUE, (base_x, base_y), boom_end, 5)
    
    # Draw the stick (from boom end to stick end)
    pygame.draw.line(screen, GREEN, boom_end, stick_end, 5)
    
    # Draw the bucket (from stick end to bucket end)
    pygame.draw.line(screen, RED, stick_end, bucket_end, 5)
    
    # Draw the joints
    pygame.draw.circle(screen, RED, (int(boom_end[0]), int(boom_end[1])), 5)
    pygame.draw.circle(screen, GREEN, (int(stick_end[0]), int(stick_end[1])), 5)
    pygame.draw.circle(screen, RED, (int(bucket_end[0]), int(bucket_end[1])), 5)

def main():
    global boom_angle, stick_angle, bucket_angle
    running = True
    clock = pygame.time.Clock()

    while running:
        # Handle keyboard input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        keys = pygame.key.get_pressed()
        
        # Control the boom angle (up/down), ensure it's within constraints
        if keys[pygame.K_UP]:
            if boom_angle < BOOM_MAX_ANGLE:
                boom_angle += 1
        if keys[pygame.K_DOWN]:
            if boom_angle > BOOM_MIN_ANGLE:
                boom_angle -= 1
        
        # Control the stick angle (extend/retract), ensure it's within constraints
        if keys[pygame.K_LEFT]:
            if stick_angle > STICK_MIN_ANGLE:
                stick_angle -= 1
        if keys[pygame.K_RIGHT]:
            if stick_angle < STICK_MAX_ANGLE:
                stick_angle += 1
        
        # Control the bucket angle (digging/opening), ensure it's within constraints
        if keys[pygame.K_w]:
            if bucket_angle < BUCKET_MAX_ANGLE:
                bucket_angle += 1
        if keys[pygame.K_s]:
            if bucket_angle > BUCKET_MIN_ANGLE:
                bucket_angle -= 1
        
        # Draw the arm
        draw_arm()
        
        # Update the screen
        pygame.display.flip()
        
        # Limit the frame rate
        clock.tick(60)
    
    # Quit Pygame
    pygame.quit()

if __name__ == "__main__":
    main()
