import matplotlib.pyplot as plt

class Plot2D:
    """
    Handles 2D scatter plotting for radar data.
    """
    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.scatter = self.ax.scatter([], [], s=20, c='r')

        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(0, 10)
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.set_title("Live Radar Detected Points (2D)")
        self.ax.grid(True)
        plt.show(block=False)

    def update(self, parsed_frame):
        """Updates the plot with new frame data."""
        points = parsed_frame.get('points', [])
        if not points:
            return

        xs = [p['x'] for p in points]
        ys = [p['y'] for p in points]
        
        self.scatter.set_offsets(list(zip(xs, ys)))
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def close(self):
        plt.close(self.fig)
