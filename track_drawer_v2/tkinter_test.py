import tkinter as tk
from tkinter import messagebox
import signal
from enum import Enum
from scipy.interpolate import interp1d
import numpy as np
import shapely


# to-do: ghost cones before placing, show distances to other cones.


class Tools(Enum):
    DRAG_CANVAS = 0
    PLACE_MIDPOINT = 1
    DRAG_MIDPOINT = 2
    INSERT_MIDPOINT = 3


class ConeTypes(Enum):
    BLUE = "dodger blue"
    YELLOW = "yellow"
    ORANGE = "orange"
    BIG_ORANGE = "darkorange1"
    MIDLINE = "red"
    UNKNOWN_COLOUR = "gray"



class TrackDrawerApp:
    def __init__(self, app_name, canvas_radius):
        self.app_name = app_name
        self.canvas_radius = canvas_radius

        self.min_zoom = 1
        self.max_zoom = 300
        self.zoom = self.min_zoom

        self.cone_radius = 0.25
        self.tool = Tools(0)
        self.conetype = ConeTypes.BLUE

        self.midpoints = list()
        self.midline_line_ids = list()

        # Window.
        self.root = tk.Tk()
        self.root.title(self.app_name)
        self.root.geometry("600x400+100+100")
        self.root.minsize(300, 200)

        # Canvas Frame.
        self.canvas_frame = tk.Frame(self.root)
        self.canvas_frame.grid(row=0, column=0)
        # Canvas.
        self.canvas = tk.Canvas(
            master=self.canvas_frame,
            bg="white",
        )
        # Scroll bars.
        self.hor_scrollbar = tk.Scrollbar(
            master=self.canvas_frame,
            orient="horizontal",
            command=self.canvas.xview,
        )
        self.vert_scrollbar = tk.Scrollbar(
            master=self.canvas_frame,
            orient="vertical",
            command=self.canvas.yview,
        )
        self.canvas.configure(
            yscrollcommand=self.vert_scrollbar.set,
            xscrollcommand=self.hor_scrollbar.set,
        )
        # One metre gridlines.
        for i in range(-self.canvas_radius, self.canvas_radius + 1):
            # Vertical line.
            self.canvas.create_line(
                i,
                -self.canvas_radius,
                i,
                self.canvas_radius,
                fill="grey",
                width=0.1,
            )
            # Horizontal line.
            self.canvas.create_line(
                -self.canvas_radius,
                i,
                self.canvas_radius,
                i,
                fill="grey",
                width=0.1,
            )
        # Grid.
        self.hor_scrollbar.grid(row=1, column=0, sticky="ew")
        self.vert_scrollbar.grid(row=0, column=1, sticky="ns")
        self.canvas.grid(row=0, column=0, sticky="nsew")

        self.update_scroll_region()
        self.update_canvas_size()

        # Handle window resizing.
        self.root.bind(sequence="<Configure>", func=lambda _: self.update_canvas_size())
        # Canvas bindings.
        self.canvas.bind("<Button-1>", self.handle_left_click)
        # Move canvas when dragging.
        self.canvas.bind("<B1-Motion>", self.handle_drag)
        # Zoom in on mouse wheel.
        self.canvas.bind(
            sequence="<Control-Button-4>",
            func=lambda event: self.change_zoom(1.2, event),
        )
        # Zoom out on mouse wheel.
        self.canvas.bind(
            sequence="<Control-Button-5>",
            func=lambda event: self.change_zoom(0.8, event),
        )
        # Reset zoom.
        self.root.bind("0", func=lambda _: self.set_zoom(2))
        
        #self.canvas.create_oval(0, 0, 10, 10, fill="yellow") # the sun

        # Zoom in so the start dominates the screen.
        self.change_zoom(self.max_zoom / (self.min_zoom * 4))

        # Check signals so app isn't left a zombie after kill in terminal.
        signal.signal(signal.SIGINT, self.destroy_app)
        # This may seem useless, but program won't catch signals
        # from mainloop alone. Something about context switching.
        self.loop_for_signals()
        self.root.protocol("WM_DELETE_WINDOW", self.on_delete_window)

        # Bind num keys to tools (DON'T HAVE MORE THAN 9 TOOLS)
        for n in range(len(Tools)):
            toolchange_lambda = self.create_lambda(self.set_tool, n)
            self.root.bind(str(n + 1), toolchange_lambda)

        self.mouse_follower = self.draw_circle(0, 0, self.cone_radius / 5, "black")
        self.canvas.bind("<Motion>", self.handle_motion)

        self.dragging = None
        self.midlinestring = None
        self.inserting_hoverer = None

        self.root.mainloop()

    def loop_for_signals(self):
        self.root.after(100, self.loop_for_signals)

    def destroy_app(self, *args):
        print("\nGoodbye!")
        self.root.destroy()

    def on_delete_window(self):
        if messagebox.askokcancel(
            f"Exit {self.app_name}",
            f"Please confirm you wish to exit {self.app_name}.",
        ):
            self.destroy_app()

    def update_scroll_region(self):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def update_canvas_size(self):
        w_width = self.root.winfo_width()
        w_height = self.root.winfo_height()
        self.canvas.config(
            width=int(w_width * 0.8) - 50,
            height=w_height - 50,
        )

    def set_zoom(self, new_zoom):
        self.change_zoom(new_zoom / self.zoom)

    def change_zoom(self, factor, event=None):
        if self.min_zoom < (new_zoom := self.zoom * factor) < self.max_zoom:
            self.zoom = new_zoom
            if event is not None:
                true_x = self.canvas.canvasx(event.x)
                true_y = self.canvas.canvasy(event.y)
            else:
                true_x = true_y = 0
            self.canvas.scale("all", true_x, true_y, factor, factor)
            self.update_scroll_region()

    def handle_left_click(self, event):
        print("handlin ", self.tool)
        match self.tool:
            case Tools.DRAG_CANVAS:
                self.canvas.scan_mark(event.x, event.y)
            case Tools.PLACE_MIDPOINT:
                self.place_midpoint(event)
            case Tools.DRAG_MIDPOINT:
                self.dragging = self.find_draggable(event)


    def handle_motion(self, event):
        match self.tool:
            case Tools.DRAG_CANVAS:
                pass
            case Tools.PLACE_MIDPOINT:
                self.move_circle(self.mouse_follower, self.canvas.canvasx(event.x), self.canvas.canvasy(event.y))
            case DRAG_MIDPOINT:
                pass

    def handle_drag(self, event):
        match self.tool:
            case Tools.DRAG_CANVAS:
                self.canvas.scan_dragto(event.x, event.y, gain=1)
            case Tools.PLACE_MIDPOINT:
                pass
            case Tools.DRAG_MIDPOINT:
                print(self.dragging)
                self.move_circle(self.dragging, self.canvas.canvasx(event.x), self.canvas.canvasy(event.y))
                self.draw_midline()


    def find_draggable(self, event):
        x = self.canvas.canvasx(event.x)
        y = self.canvas.canvasy(event.y)
        mod = 0.01
        objects_under_mouse = self.canvas.find_overlapping(x - mod, y - mod, x + mod, y + mod)
        for object_id in objects_under_mouse:
            print(object_id)
            print(self.canvas.gettags(object_id))
            if "draggable" in self.canvas.gettags(object_id):
                return object_id

    def place_midpoint(self, event):
        x = self.canvas.canvasx(event.x)
        y = self.canvas.canvasy(event.y)
        midpoint = self.draw_circle(x, y, self.cone_radius, "red", tags=("midpoint", "draggable"))
        self.midpoints.append(midpoint)
        self.draw_midline()

    def drag_midpoint(self, event):
        pass


    def draw_midline(self):
        while len(self.midline_line_ids) > 0:
            self.canvas.delete(self.midline_line_ids.pop())

        match len(self.midpoints):
            case 0:
                return
            case 1:
                return
            case 2:
                kind = "linear"
            case _:
                kind = "quadratic"
            #case _:
            #    kind = "cubic"

        x = list()
        y = list()
        for midpoint in self.midpoints:
            coords = self.canvas.coords(midpoint)
            x.append(coords[0] + self.zoom * self.cone_radius)
            y.append(coords[1] + self.zoom * self.cone_radius)

        t = np.arange(len(x))
        ti = np.linspace(0, t.max(), 10 * t.size)
        xi = interp1d(t, x, kind=kind)(ti)
        yi = interp1d(t, y, kind=kind)(ti)

        xys = []

        last_x, last_y = xi[0], yi[0]
        for (x, y) in zip(xi[1:], yi[1:]):
            line_id = self.canvas.create_line(last_x, last_y, x, y, width=self.zoom*0.02)
            last_x = x
            last_y = y
            xys.append((x, y))
            self.midline_line_ids.append(line_id)

        self.midlinestring = shapely.LineString(xys)

        blue_line = self.midlinestring.offset_curve(self.zoom * 1.5)
        last_x, last_y = blue_line.coords[0]
        for xy in blue_line.coords[1:]:
            x = xy[0]
            y = xy[1]
            line_id = self.canvas.create_line(last_x, last_y, x, y, fill="dodger blue", width=self.zoom*0.02)
            last_x = x
            last_y = y
            xys.append((x, y))
            self.midline_line_ids.append(line_id)

        blue_line = self.midlinestring.offset_curve(self.zoom * -1.5)
        last_x, last_y = blue_line.coords[0]
        for xy in blue_line.coords[1:]:
            x = xy[0]
            y = xy[1]
            line_id = self.canvas.create_line(last_x, last_y, x, y, fill="gold", width=self.zoom*0.02)
            last_x = x
            last_y = y
            xys.append((x, y))
            self.midline_line_ids.append(line_id)

    def draw_circle(self, x, y, size, colour, tags=None):
        mod = size * self.zoom
        return self.canvas.create_oval(
            x - mod,
            y - mod,
            x + mod,
            y + mod,
            fill=colour,
            tags = tags
        )

    def move_circle(self, object, x, y):
        current_coords = self.canvas.coords(object)
        mod = (current_coords[2] - current_coords[0]) / 2
        self.canvas.coords(object, x - mod, y - mod, x + mod, y + mod)

    def set_tool(self, toolnum):
        self.tool = Tools(toolnum)
        print(f"set tool to {self.tool}")

    def create_lambda(self, function, arg):
        return lambda _: function(arg)


if __name__ == "__main__":
    TrackDrawerApp("EUFS Track Drawer V2", 50)
