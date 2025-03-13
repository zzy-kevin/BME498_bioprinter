import tkinter as tk
from PIL import Image, ImageTk
import numpy as np
import matplotlib.path as mpath
import matplotlib.patches as mpatches

class ImageClickApp:
    def __init__(self, root, image_path):
        self.root = root
        self.root.title("Image Click Coordinate Selector")

        # Load the image
        self.image = Image.open(image_path)
        self.tk_image = ImageTk.PhotoImage(self.image)

        # Create Canvas and display image
        self.canvas = tk.Canvas(root, width=self.image.width, height=self.image.height)
        self.canvas.pack()
        self.canvas.create_image(0, 0, anchor="nw", image=self.tk_image)

        # Bind mouse click event
        self.canvas.bind("<Button-1>", self.get_coordinates)

        # Submit button
        self.submit_button = tk.Button(root, text="Submit", command=self.submit, font=("Arial", 12))
        self.submit_button.pack(pady=10)

        # Store clicked coordinates
        self.selected_x = None
        self.selected_y = None
        self.dot = None  # Store the last drawn dot

    def get_coordinates(self, event):
        """Callback function to mark the clicked position with a red dot."""
        self.selected_x, self.selected_y = event.x, event.y
        print(f"Clicked at: ({self.selected_x}, {self.selected_y})")

        # Remove the previous dot if it exists
        if self.dot:
            self.canvas.delete(self.dot)

        # Draw a new red dot (circle)
        dot_radius = 5
        self.dot = self.canvas.create_oval(self.selected_x - dot_radius, self.selected_y - dot_radius, 
                                           self.selected_x + dot_radius, self.selected_y + dot_radius, 
                                           fill="red", outline="red")

    def submit(self):
        """Closes the GUI and returns the selected coordinates."""
        self.root.quit()  # Stops the Tkinter mainloop
        self.root.destroy()  # Destroys the window
        
class ImageMaskEditor:
    def __init__(self, root, image_path, mask_array):
        self.root = root
        self.root.title("Mask Editor")

        # Load image
        self.image = Image.open(image_path)
        self.mask = mask_array.copy()  # Copy the original mask

        self.original_mask = mask_array.copy()

        self.image_width, self.image_height = self.image.size

        # Convert mask to overlay (red transparent mask)
        self.overlay = self.create_mask_overlay()

        # Create Canvas
        self.canvas = tk.Canvas(root, width=self.image_width, height=self.image_height)
        self.canvas.pack()

        # Display initial image with mask
        self.tk_image = ImageTk.PhotoImage(self.overlay)
        self.canvas_image = self.canvas.create_image(0, 0, anchor="nw", image=self.tk_image)

        # Bind mouse events for drawing
        self.canvas.bind("<Button-1>", self.start_drawing)  # Left click to start drawing
        self.canvas.bind("<B1-Motion>", self.draw_freehand)  # Left-click drag to draw freehand
        self.canvas.bind("<ButtonRelease-1>", self.complete_drawing)  # Release to close drawing

        self.undo_button = tk.Button(root, text="Undo", command=self.undo, font=("Arial", 12))
        self.undo_button.pack(pady=10)  # Display the button

        self.clear_button = tk.Button(root, text="Clear", command=self.clear_masks, font=("Arial", 12))  # **(Added)**
        self.clear_button.pack(pady=10)

        # Submit button
        self.submit_button = tk.Button(root, text="Submit", command=self.submit, font=("Arial", 12))
        self.submit_button.pack(pady=10)

        self.undo_stack = []
        self.drawing = False
        self.points = []  # List of points that form the drawn shape
        self.current_line = None  # Store the current line being drawn

    def create_mask_overlay(self):
        """Generates an overlay image with the segmentation mask."""
        # Convert mask to an RGBA image (red overlay where mask == 1)
        mask_rgba = np.zeros((self.mask.shape[0], self.mask.shape[1], 4), dtype=np.uint8)
        mask_rgba[self.mask == 1] = [255, 0, 0, 100]  # Red overlay with transparency

        # Convert mask image to PIL format
        mask_img = Image.fromarray(mask_rgba, mode="RGBA")

        # Merge with original image
        overlay = Image.alpha_composite(self.image.convert("RGBA"), mask_img)
        return overlay.convert("RGB")  # Convert back to RGB for Tkinter display

    def start_drawing(self, event):
            """Starts the drawing process by recording the starting point."""
            self.drawing = True
            self.points = [(event.x, event.y)]  # Start with the initial point
            self.current_line = self.canvas.create_line(event.x, event.y, event.x, event.y, fill="red", width=2)

    def draw_freehand(self, event):
        """Draws the freehand shape as the user drags."""
        if self.drawing:
            # Add the current point to the points list
            self.points.append((event.x, event.y))

            # Update the current line
            self.canvas.coords(self.current_line, *sum(self.points, ()))

    def complete_drawing(self, event):
        """Completes the drawing and fills the area inside the shape."""
        if self.drawing:
            self.drawing = False
            self.points.append((event.x, event.y))  # Final point of the shape

            # Create a polygon path from the drawn points
            path = mpath.Path(self.points)

            self.save_state()
            # Fill the area inside the polygon with 1 in the mask
            self.fill_polygon(path)

            # Update the display with the new mask
            self.update_display()

            self.canvas.delete(self.current_line)

    def fill_polygon(self, path):
        """Fills the area inside the polygon in the mask."""
        # Iterate over all pixels in the mask and check if they are inside the polygon
        for y in range(self.image_height):
            for x in range(self.image_width):
                if path.contains_point((x, y)):
                    self.mask[y, x] = 1  # Set pixel inside the polygon to 1

    def update_display(self):
        """Updates the displayed image with the modified mask."""
        self.overlay = self.create_mask_overlay()
        self.tk_image = ImageTk.PhotoImage(self.overlay)
        self.canvas.itemconfig(self.canvas_image, image=self.tk_image)

    def save_state(self):
        """Saves the current state of the mask in the undo stack."""
        self.undo_stack.append(self.mask.copy()) 
    
    def undo(self):
        """Reverts to the previous state of the mask."""
        if self.undo_stack:
            # Pop the last state from the stack
            self.mask = self.undo_stack.pop()

            # Update the display with the reverted mask
            self.update_display()

    def clear_masks(self):
        """Clears all the drawn masks and hides the mask overlay."""
        # Reset the mask to a blank (no mask) state
        self.mask = np.zeros_like(self.mask)  # Create an empty mask (all zeros)
        
        # **Update the overlay to show only the image without any mask**
        self.overlay = self.image.convert("RGB")  # Only show the image without any red mask
        self.tk_image = ImageTk.PhotoImage(self.overlay)  # Convert back to Tkinter-compatible image
        self.canvas.itemconfig(self.canvas_image, image=self.tk_image)  # Update the canvas with the new image (no mask)

        # **Remove all the drawn shapes (polygons) from the canvas**
        self.canvas.delete("mask")  # Remove only the drawn shapes (we use "mask" tag)

    def submit(self):
        """Closes the GUI and returns the updated mask."""
        self.root.quit()
        self.root.update_idletasks()
        self.root.destroy()



def edit_mask(image_path, mask_array):
    root = tk.Toplevel()
    app = ImageMaskEditor(root, image_path, mask_array)
    root.mainloop()
    return app.mask  # Return the modified mask

# Main function to call the select point GUI
def select_point(image_path):
    root = tk.Toplevel()
    app = ImageClickApp(root, image_path)
    root.mainloop()
    return app.selected_x, app.selected_y  # Return the selected coordinates

def display_mask(image_path):
    root = tk.Toplevel()
    app = ImageClickApp(root, image_path)
    root.mainloop()

# Example usage
if __name__ == "__main__":
    image_path = "3cm_1.jpg" 
    x, y = select_point(image_path)
    print(f"Selected coordinates: ({x}, {y})")
