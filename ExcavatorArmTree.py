import pygame
import math
import random
import numpy as np
from collections import defaultdict

# Constants 
BOOM_MIN_ANGLE = 0
BOOM_MAX_ANGLE = 87
STICK_MIN_ANGLE = -60
STICK_MAX_ANGLE = -15
BUCKET_MIN_ANGLE = -90
BUCKET_MAX_ANGLE = 20

# Scale and position constants
BOOM_LENGTH = 160  
STICK_LENGTH = 120  
BUCKET_LENGTH = 60  
VERTICAL_OFFSET = 250  # Shift everything up by this amount

class Node:
    def __init__(self, boom_angle, stick_angle, bucket_angle):
        self.boom_angle = boom_angle
        self.stick_angle = stick_angle
        self.bucket_angle = bucket_angle
        self.parent = None
        self.children = []
        self.cost = 0
        self.depth = 0

class MotionTree:
    def __init__(self):
        self.nodes = []
        self.step_size = 5  # Degrees
        self.max_nodes = 1000
        self.max_depth = 0  # Track maximum tree depth
        
    def is_valid_configuration(self, boom, stick, bucket):
        # Check if a configuration respects joint limits
        if not (BOOM_MIN_ANGLE <= boom <= BOOM_MAX_ANGLE):
            return False
        if not (STICK_MIN_ANGLE <= stick <= STICK_MAX_ANGLE):
            return False
        if not (BUCKET_MIN_ANGLE <= bucket <= BUCKET_MAX_ANGLE):
            return False
        return True
    
    def distance(self, node1, node2):
        #Calculate distance between two configurations
        d_boom = abs(node1.boom_angle - node2.boom_angle)
        d_stick = abs(node1.stick_angle - node2.stick_angle)
        d_bucket = abs(node1.bucket_angle - node2.bucket_angle)
        return math.sqrt(d_boom**2 + d_stick**2 + d_bucket**2)
    
    def nearest_neighbor(self, node):
        #Find nearest existing node
        if not self.nodes:
            return None
        return min(self.nodes, key=lambda n: self.distance(n, node))
    
    def step_towards(self, from_node, to_node):
        #Create a new node stepping from one node towards another
        d = self.distance(from_node, to_node)
        if d <= self.step_size:
            return to_node
        
        # Interpolate angles
        ratio = self.step_size / d
        new_boom = from_node.boom_angle + ratio * (to_node.boom_angle - from_node.boom_angle)
        new_stick = from_node.stick_angle + ratio * (to_node.stick_angle - from_node.stick_angle)
        new_bucket = from_node.bucket_angle + ratio * (to_node.bucket_angle - from_node.bucket_angle)
        
        return Node(new_boom, new_stick, new_bucket)
    
    def build_tree(self, start_config, goal_config=None):
        # Build RRT from start configuration, optionally towards a goal
        start_node = Node(*start_config)
        start_node.depth = 0
        self.nodes.append(start_node)
        
        while len(self.nodes) < self.max_nodes:
            # Generate random configuration or bias towards goal
            if goal_config and random.random() < 0.2:  # 20% chance to grow towards goal
                rand_node = Node(*goal_config)
            else:
                rand_boom = random.uniform(BOOM_MIN_ANGLE, BOOM_MAX_ANGLE)
                rand_stick = random.uniform(STICK_MIN_ANGLE, STICK_MAX_ANGLE)
                rand_bucket = random.uniform(BUCKET_MIN_ANGLE, BUCKET_MAX_ANGLE)
                rand_node = Node(rand_boom, rand_stick, rand_bucket)
            
            # Find nearest neighbor
            nearest = self.nearest_neighbor(rand_node)
            
            # Step towards random node
            new_node = self.step_towards(nearest, rand_node)
            
            # Check if new configuration is valid
            if self.is_valid_configuration(new_node.boom_angle, 
                                        new_node.stick_angle,
                                        new_node.bucket_angle):
                new_node.parent = nearest
                new_node.depth = nearest.depth + 1
                self.max_depth = max(self.max_depth, new_node.depth)
                nearest.children.append(new_node)
                self.nodes.append(new_node)
                
                # Check if the goal has been reached
                if (goal_config and 
                    self.distance(new_node, Node(*goal_config)) < self.step_size):
                    return True
        
        return goal_config is None


class MotionVisualizer:
    def __init__(self, width=800, height=600):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Excavator Motion Tree")
        self.clock = pygame.time.Clock()
        
        # Enhanced colors
        self.BACKGROUND = (10, 10, 30)
        self.BASE_COLOR = (200, 200, 200)
        self.START_COLOR = (0, 255, 100)
        self.GOAL_COLOR = (255, 100, 0)
        self.JOINT_COLOR = (255, 255, 0)
        self.gradient_start = (100, 200, 255)
        self.gradient_end = (255, 50, 150)
        
        # Track the bounds of all configurations
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        
    def interpolate_color(self, depth, max_depth):
        if max_depth == 0:
            return self.gradient_start
        ratio = depth / max_depth
        r = int(self.gradient_start[0] + (self.gradient_end[0] - self.gradient_start[0]) * ratio)
        g = int(self.gradient_start[1] + (self.gradient_end[1] - self.gradient_start[1]) * ratio)
        b = int(self.gradient_start[2] + (self.gradient_end[2] - self.gradient_start[2]) * ratio)
        return (r, g, b)
    
    def calculate_endpoints(self, node):
        #Calculate endpoint positions for a configuration
        base_x = self.width // 3
        base_y = self.height - VERTICAL_OFFSET  # Shifted up
        
        boom_end = (
            base_x + BOOM_LENGTH * math.cos(math.radians(node.boom_angle)),
            base_y - BOOM_LENGTH * math.sin(math.radians(node.boom_angle))
        )
        
        stick_end = (
            boom_end[0] + STICK_LENGTH * math.cos(math.radians(node.boom_angle + node.stick_angle)),
            boom_end[1] - STICK_LENGTH * math.sin(math.radians(node.boom_angle + node.stick_angle))
        )
        
        bucket_end = (
            stick_end[0] + BUCKET_LENGTH * math.cos(math.radians(node.boom_angle + node.stick_angle + node.bucket_angle)),
            stick_end[1] - BUCKET_LENGTH * math.sin(math.radians(node.boom_angle + node.stick_angle + node.bucket_angle))
        )
        
        return (base_x, base_y), boom_end, stick_end, bucket_end
            
    def draw_configuration(self, node, color, line_width=2):
        #Draw excavator configuration with enhanced visibility
        base_pos, boom_end, stick_end, bucket_end = self.calculate_endpoints(node)
        
        # Update bounds
        points = [base_pos, boom_end, stick_end, bucket_end]
        for x, y in points:
            self.min_x = min(self.min_x, x)
            self.max_x = max(self.max_x, x)
            self.min_y = min(self.min_y, y)
            self.max_y = max(self.max_y, y)
        
        # Draw links with glow effect
        for width in range(line_width + 4, line_width - 1, -1):
            alpha = 100 if width > line_width else 255
            current_color = (*color, alpha) if width > line_width else color
            
            pygame.draw.line(self.screen, current_color, base_pos, boom_end, width)
            pygame.draw.line(self.screen, current_color, boom_end, stick_end, width)
            pygame.draw.line(self.screen, current_color, stick_end, bucket_end, width)
        
        # Draw joints
        pygame.draw.circle(self.screen, self.JOINT_COLOR, (int(base_pos[0]), int(base_pos[1])), 6)
        pygame.draw.circle(self.screen, self.JOINT_COLOR, (int(boom_end[0]), int(boom_end[1])), 4)
        pygame.draw.circle(self.screen, self.JOINT_COLOR, (int(stick_end[0]), int(stick_end[1])), 4)
        
    def draw_workspace_bounds(self):
        #Draw a rectangle showing the workspace bounds
        margin = 10
        pygame.draw.rect(self.screen, (100, 100, 100), (
            self.min_x - margin,
            self.min_y - margin,
            self.max_x - self.min_x + 2*margin,
            self.max_y - self.min_y + 2*margin
        ), 1)
        
    def visualize_tree(self, motion_tree):
        #Visualize the entire motion tree with enhanced colors
        running = True
        show_bounds = False  # Toggle for workspace bounds
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_b:  # Press 'B' to toggle bounds
                        show_bounds = not show_bounds
            
            self.screen.fill(self.BACKGROUND)
            
            # Draw base platform
            base_x = self.width // 3
            base_y = self.height - VERTICAL_OFFSET
            pygame.draw.rect(self.screen, self.BASE_COLOR, 
                           (base_x - 50, base_y - 10, 100, 20))
            
            # Reset bounds for each frame
            self.min_x = float('inf')
            self.max_x = float('-inf')
            self.min_y = float('inf')
            self.max_y = float('-inf')
            
            # Draw all configurations in tree with color gradient
            for node in motion_tree.nodes:
                color = self.interpolate_color(node.depth, motion_tree.max_depth)
                self.draw_configuration(node, color)
            
            # Draw start configuration in green
            if motion_tree.nodes:
                self.draw_configuration(motion_tree.nodes[0], self.START_COLOR, 3)
            
            # Draw workspace bounds if enabled
            if show_bounds:
                self.draw_workspace_bounds()
            
            # Add helpful text
            font = pygame.font.Font(None, 36)
            text = font.render("Motion Tree Visualization", True, (255, 255, 255))
            self.screen.blit(text, (10, 10))
            
            depth_text = font.render(f"Tree Depth: {motion_tree.max_depth}", True, (255, 255, 255))
            self.screen.blit(depth_text, (10, 50))
            
            pygame.display.flip()
            self.clock.tick(30)
        
        pygame.quit()

def main():
    # Create start and goal configurations
    start_config = (0, -60, 0)
    goal_config = (45, -90, 30)
    
    # Build motion tree
    motion_tree = MotionTree()
    success = motion_tree.build_tree(start_config, goal_config)
    
    # Visualize results
    visualizer = MotionVisualizer()
    visualizer.visualize_tree(motion_tree)

if __name__ == "__main__":
    main()