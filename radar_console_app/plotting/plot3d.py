import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Plot3D:
    """
    Handles 3D scatter plotting for radar data.
    """
    def __init__(self):
        plt.ion()
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(0, 10)
        self.ax.set_zlim(-3, 3)
        
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_zlabel('Z (meters)')
        self.ax.set_title('Live Radar Detected Points (3D)')
        
        self.scatter = self.ax.scatter([], [], [], s=20, c='b')
        plt.show(block=False)

    def update(self, parsed_frame):
        """Updates the plot with new frame data."""
        points = parsed_frame.get('points', [])
        if not points:
            return

        xs = [p['x'] for p in points]
        ys = [p['y'] for p in points]
        zs = [p['z'] for p in points]

        self.ax.cla()
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(0, 10)
        self.ax.set_zlim(-3, 3)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_zlabel('Z (meters)')
        self.ax.set_title('Live Radar Detected Points (3D)')
        
        self.ax.scatter(xs, ys, zs, s=20, c='b')
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def close(self):
        plt.close(self.fig)
