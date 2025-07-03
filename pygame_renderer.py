import pygame
import numpy as np
import math
from collections import deque
from pygame.locals import *

# Initialize Pygame
pygame.init()

class PygameRenderer:
    def __init__(self, width=1200, height=900, title="N-Body Simulation"):
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption(title)
        
        # Camera and view settings
        self.camera_distance = 500
        self.camera_angle_x = 0
        self.camera_angle_y = 0
        self.zoom_factor = 1.0
        self.min_zoom = 0.1
        self.max_zoom = 5.0
        
        # Mouse interaction
        self.mouse_dragging = False
        self.last_mouse_pos = (0, 0)
        
        # Colors
        self.bg_color = (5, 5, 15)  # Dark space background
        self.trail_alpha_base = 100
        
        # Performance tracking
        self.clock = pygame.time.Clock()
        self.fps = 60
        self.frame_count = 0
        
        # UI elements
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        
        self.running = True
        
    def project_3d_to_2d(self, pos_3d):
        """Project 3D coordinates to 2D screen coordinates with perspective"""
        x, y, z = pos_3d
        
        # Apply camera rotation
        cos_x, sin_x = math.cos(self.camera_angle_x), math.sin(self.camera_angle_x)
        cos_y, sin_y = math.cos(self.camera_angle_y), math.sin(self.camera_angle_y)
        
        # Rotate around Y axis
        x_rot = x * cos_y - z * sin_y
        z_rot = x * sin_y + z * cos_y
        
        # Rotate around X axis
        y_rot = y * cos_x - z_rot * sin_x
        z_final = y * sin_x + z_rot * cos_x
        
        # Apply zoom
        x_rot *= self.zoom_factor
        y_rot *= self.zoom_factor
        z_final *= self.zoom_factor
        
        # Perspective projection
        distance = self.camera_distance + z_final
        if distance <= 0:
            distance = 1  # Prevent division by zero
            
        perspective_scale = self.camera_distance / distance
        
        screen_x = int(self.width / 2 + x_rot * perspective_scale)
        screen_y = int(self.height / 2 - y_rot * perspective_scale)  # Flip Y for screen coordinates
        
        return screen_x, screen_y, distance
    
    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == QUIT:
                self.running = False
                return False
                
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    self.running = False
                    return False
                elif event.key == K_r:
                    # Reset camera
                    self.camera_angle_x = 0
                    self.camera_angle_y = 0
                    self.zoom_factor = 1.0
                    
            elif event.type == MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    self.mouse_dragging = True
                    self.last_mouse_pos = event.pos
                    
            elif event.type == MOUSEBUTTONUP:
                if event.button == 1:
                    self.mouse_dragging = False
                    
            elif event.type == MOUSEMOTION:
                if self.mouse_dragging:
                    dx = event.pos[0] - self.last_mouse_pos[0]
                    dy = event.pos[1] - self.last_mouse_pos[1]
                    
                    # Update camera angles
                    self.camera_angle_y += dx * 0.01
                    self.camera_angle_x += dy * 0.01
                    
                    # Clamp X rotation to prevent flipping
                    self.camera_angle_x = max(-math.pi/2, min(math.pi/2, self.camera_angle_x))
                    
                    self.last_mouse_pos = event.pos
                    
            elif event.type == MOUSEWHEEL:
                # Zoom with mouse wheel
                zoom_speed = 0.1
                if event.y > 0:  # Scroll up
                    self.zoom_factor = min(self.max_zoom, self.zoom_factor * (1 + zoom_speed))
                else:  # Scroll down
                    self.zoom_factor = max(self.min_zoom, self.zoom_factor * (1 - zoom_speed))
        
        return True
    
    def draw_body(self, position, mass, color, trail_positions=None):
        """Draw a celestial body with optional trail"""
        screen_x, screen_y, distance = self.project_3d_to_2d(position)
        
        # Skip if behind camera or off screen
        if distance <= 0:
            return
            
        # Calculate size based on mass and distance
        base_size = max(3, math.log(mass + 1, 1.3))
        size = int(base_size * (self.camera_distance / distance) * self.zoom_factor)
        size = max(2, min(50, size))  # Clamp size
        
        # Draw trail if provided
        if trail_positions and len(trail_positions) > 1:
            self.draw_trail(trail_positions, color)
        
        # Draw body
        if 0 <= screen_x < self.width and 0 <= screen_y < self.height:
            # Draw glow effect for larger bodies
            if size > 8:
                glow_color = tuple(min(255, c + 50) for c in color)
                pygame.draw.circle(self.screen, glow_color, (screen_x, screen_y), size + 2)
            
            pygame.draw.circle(self.screen, color, (screen_x, screen_y), size)
            
            # Draw a small highlight
            if size > 4:
                highlight_color = tuple(min(255, c + 100) for c in color)
                pygame.draw.circle(self.screen, highlight_color, 
                                 (screen_x - size//3, screen_y - size//3), max(1, size//4))
    
    def draw_trail(self, trail_positions, color):
        """Draw a fading trail for a body"""
        if len(trail_positions) < 2:
            return
            
        trail_points = []
        for pos in trail_positions:
            screen_x, screen_y, distance = self.project_3d_to_2d(pos)
            if distance > 0:  # Only include points in front of camera
                trail_points.append((screen_x, screen_y))
        
        if len(trail_points) < 2:
            return
            
        # Draw trail segments with fading alpha
        num_segments = len(trail_points) - 1
        for i in range(num_segments):
            if i >= len(trail_points) - 1:
                break
                
            # Calculate alpha for fading effect
            alpha = int(self.trail_alpha_base * (i + 1) / num_segments)
            alpha = max(20, min(255, alpha))
            
            # Create a surface for alpha blending
            trail_surface = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
            trail_color = (*color, alpha)
            
            # Draw line segment
            if (0 <= trail_points[i][0] < self.width and 0 <= trail_points[i][1] < self.height and
                0 <= trail_points[i+1][0] < self.width and 0 <= trail_points[i+1][1] < self.height):
                pygame.draw.line(trail_surface, trail_color, trail_points[i], trail_points[i+1], 2)
                self.screen.blit(trail_surface, (0, 0))
    
    def draw_ui(self, timestep, total_timesteps, fps):
        """Draw UI elements"""
        # Background for UI
        ui_bg = pygame.Surface((300, 120), pygame.SRCALPHA)
        ui_bg.fill((0, 0, 0, 128))
        self.screen.blit(ui_bg, (10, 10))
        
        # Text information
        texts = [
            f"Timestep: {timestep}/{total_timesteps}",
            f"FPS: {fps:.1f}",
            f"Zoom: {self.zoom_factor:.2f}x",
            f"Camera: ({self.camera_angle_x:.2f}, {self.camera_angle_y:.2f})",
            "",
            "Controls:",
            "Mouse: Drag to rotate",
            "Wheel: Zoom in/out",
            "R: Reset camera",
            "ESC: Exit"
        ]
        
        for i, text in enumerate(texts):
            if text:  # Skip empty lines
                color = (255, 255, 255) if not text.startswith("Controls") else (200, 200, 100)
                font = self.small_font if text.startswith(("Mouse:", "Wheel:", "R:", "ESC:")) else self.font
                text_surface = font.render(text, True, color)
                self.screen.blit(text_surface, (15, 15 + i * 20))
    
    def clear_screen(self):
        """Clear the screen with background color"""
        self.screen.fill(self.bg_color)
    
    def present(self, timestep=0, total_timesteps=1000):
        """Present the frame to screen"""
        # Draw UI
        current_fps = self.clock.get_fps()
        self.draw_ui(timestep, total_timesteps, current_fps)
        
        # Update display
        pygame.display.flip()
        self.clock.tick(self.fps)
        self.frame_count += 1
    
    def is_running(self):
        """Check if the renderer should continue running"""
        return self.running and self.handle_events()
    
    def cleanup(self):
        """Clean up pygame resources"""
        pygame.quit()

# Color utilities
def hex_to_rgb(hex_color):
    """Convert hex color to RGB tuple"""
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

# Predefined colors for celestial bodies
COLORS = {
    'sun': (255, 255, 100),
    'mercury': (169, 169, 169),
    'venus': (255, 198, 73),
    'earth': (100, 149, 237),
    'mars': (205, 92, 92),
    'jupiter': (255, 140, 0),
    'saturn': (255, 215, 0),
    'uranus': (64, 224, 208),
    'neptune': (65, 105, 225),
    'planet': (150, 150, 255)  # Default planet color
}
