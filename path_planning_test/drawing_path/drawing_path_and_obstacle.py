import tkinter as tk
import os

class DrawingApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Drawing App")
        self.master.geometry("600x600")

        self.canvas = tk.Canvas(self.master, width=500, height=500, bg="white")
        self.canvas.pack(expand=tk.YES, fill=tk.BOTH)

        self.path_coords = []
        self.obstacle_coords = []
        self.current_mode = "path"
        self.current_shape = None

        self.master.bind("<Button-1>", self.draw)
        self.master.bind("<Button-3>", self.switch_mode)

        self.clear_button = tk.Button(self.master, text="Clear", command=self.clear_canvas)
        self.clear_button.pack(side=tk.BOTTOM)

        self.save_button = tk.Button(self.master, text="Save", command=self.save_data)
        self.save_button.pack(side=tk.BOTTOM)

    def draw(self, event):
        x, y = event.x, event.y
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()

        if x < 0 or x > canvas_width or y < 0 or y > canvas_height:
            return

        if self.current_mode == "path":
            self.path_coords.append((x/5, (500-y)/5))
            self.current_shape = self.canvas.create_oval(x-3, y-3, x+3, y+3, fill="black")
        elif self.current_mode == "obstacle":
            self.obstacle_coords.append((x/5, (500-y)/5))
            self.current_shape = self.canvas.create_line(x-5, y-5, x+5, y+5, fill="red")
            self.canvas.create_line(x+5, y-5, x-5, y+5, fill="red")

    def switch_mode(self, event):
        if self.current_mode == "path":
            self.current_mode = "obstacle"
        else:
            self.current_mode = "path"

    def clear_canvas(self):
        self.canvas.delete("all")
        self.path_coords = []
        self.obstacle_coords = []

    def save_data(self):
        if self.current_mode == "path":
            self.path_coords.pop()
        else:
            self.obstacle_coords.pop()
        path_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "path", "txt_file")
        os.makedirs(path_dir, exist_ok=True)
        with open(os.path.join(path_dir, "path.txt"), "w") as f:
            for coord in self.path_coords:
                f.write(f"{coord[0]} {coord[1]}\n")

        with open(os.path.join(path_dir, "obstacle.txt"), "w") as f:
            for coord in self.obstacle_coords:
                f.write(f"{coord[0]} {coord[1]}\n")

if __name__ == "__main__":
    root = tk.Tk()
    app = DrawingApp(root)
    root.mainloop()
