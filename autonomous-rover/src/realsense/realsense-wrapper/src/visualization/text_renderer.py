class TextRenderer:
    def __init__(self):
        """Initialize the text renderer."""
        self.distance_chars = " .:nhBXW#"  # Characters for depth visualization, from far to close

    def render(self, depth_frame, width=64, height=15):
        """
        Render a depth frame as ASCII art.
        
        Args:
            depth_frame: RealSense depth frame
            width: Width of the ASCII output
            height: Height of the ASCII output
            
        Returns:
            String representation of the depth frame
        """
        if depth_frame is None:
            return "No depth data available"
            
        # Get frame dimensions
        frame_width = depth_frame.get_width()
        frame_height = depth_frame.get_height()
        
        # Calculate step sizes
        step_x = max(1, frame_width // width)
        step_y = max(1, frame_height // height)
        
        # Initialize visualization string
        visualization = []
        
        # Process the depth frame
        for y in range(0, frame_height, step_y):
            if y >= frame_height:
                break
                
            line = ""
            for x in range(0, frame_width, step_x):
                if x >= frame_width:
                    break
                    
                # Get distance at pixel (x, y)
                distance = depth_frame.get_distance(x, y)
                
                # Convert distance to character
                if distance <= 0:
                    # No valid data
                    line += " "
                else:
                    # Map distance to character index
                    # Closer objects get darker/denser characters
                    # Further objects get lighter characters
                    # Adjust max_distance to change the scale
                    max_distance = 5.0  # 5 meters
                    normalized = min(1.0, distance / max_distance)
                    char_idx = min(len(self.distance_chars) - 1, 
                                  int(normalized * len(self.distance_chars)))
                    line += self.distance_chars[char_idx]
                    
            visualization.append(line)
            
        return "\n".join(visualization)

    def get_distance_summary(self, depth_frame):
        """
        Generate a summary of distances in the frame.
        
        Args:
            depth_frame: RealSense depth frame
            
        Returns:
            String with distance summary
        """
        if depth_frame is None:
            return "No depth data available"
            
        # Get frame dimensions
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        
        # Calculate center point
        center_x = width // 2
        center_y = height // 2
        
        # Get distance at center
        center_dist = depth_frame.get_distance(center_x, center_y)
        
        # Sample several points to get minimum distance
        min_dist = float('inf')
        sample_step = 20  # Sample every 20 pixels
        
        for y in range(0, height, sample_step):
            for x in range(0, width, sample_step):
                dist = depth_frame.get_distance(x, y)
                if dist > 0:  # Ignore invalid measurements
                    min_dist = min(min_dist, dist)
        
        if min_dist == float('inf'):
            min_dist = 0
            
        return f"Center: {center_dist:.2f}m, Closest: {min_dist:.2f}m"