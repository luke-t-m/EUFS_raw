from tkinter import *
from tkinter import ttk
from tkinter.filedialog import askopenfilename, asksaveasfilename
import signal
import csv
import time

class Shape:
  def __init__(self, event, id, current_tool, current_scale):
    self.event = event
    self.id = id
    self.tool = current_tool
    self.scale = current_scale

class Oval(Shape):
  def is_oval(self): return True

class Line(Shape):
  def is_oval(self): return False


class Drawer:
  current_tool = "blue"
  current_scale = 1
  shapes = []
  background_lines = []

  def __init__(self, canvas_x, canvas_y, dot_size = 20, px_to_m = 50):
    self.canvas_x = canvas_x
    self.canvas_y = canvas_y
    self.dot_size = dot_size
    self.px_to_m = px_to_m
    self.canvas_hx = canvas_x / 2
    self.canvas_hy = canvas_y / 2

    self.root = Tk()
    self.root.title("EUFS Track Drawer")
    frame = Frame(self.root, width = self.canvas_x, height = self.canvas_y)
    frame.pack(expand=True, fill=BOTH)
    self.canvas = Canvas(frame, bg = "#FFFFFF", width = self.canvas_x, height = self.canvas_y)
    self.text = Text(self.root, bg = "#000000", fg = "#ffffff", height = 8)
    self.text.pack(side = TOP, expand = True, fill = BOTH)
    self.text.insert(END, f"grid : {self.px_to_m} pixels to 1 metre.")
    self.canvas.pack(side = LEFT, expand = True, fill = BOTH)
    self.canvas.create_oval(self.canvas_hx - 10, self.canvas_hy - 20, self.canvas_hx + 10, self.canvas_hy + 20, fill = "black", width = 0) # Car
    self.redraw_grid()

    select_blue = lambda *args: self.select_tool("blue")
    select_yellow = lambda *args: self.select_tool("yellow")
    select_orange = lambda *args: self.select_tool("orange")
    select_unknown = lambda *args: self.select_tool("unknown")
    select_midpoint = lambda *args: self.select_tool("midpoint")
    zoom_in = lambda *args: self.scale(1.1)
    zoom_out = lambda *args: self.scale(0.9)

    buttons = Frame()
    buttons.pack(side = "left", fill = "y")
    ttk.Button(buttons, text = "blue (b)", command = select_blue).pack(side = LEFT)
    ttk.Button(buttons, text = "yellow (y)", command = select_yellow).pack(side = LEFT)
    ttk.Button(buttons, text = "orange (o)", command = select_orange).pack(side = LEFT)
    ttk.Button(buttons, text = "unknown (u)", command = select_unknown).pack(side = LEFT)
    ttk.Button(buttons, text = "midpoint (m)", command = select_midpoint).pack(side = LEFT)
    ttk.Button(buttons, text = "undo (\\b)", command = self.undo).pack(side = LEFT)
    ttk.Button(buttons, text = "zoom_in (+)", command = zoom_in).pack(side = LEFT)
    ttk.Button(buttons, text = "zoom_out (-)", command = zoom_out).pack(side = LEFT)
    ttk.Button(buttons, text = "save (ctrl-s)", command = self.save).pack(side = LEFT)
    #ttk.Button(buttons, text = "load (ctrl-o)", command = load).pack(side = LEFT)
    ttk.Button(buttons, text = "exit (ctrl-c)", command = self.suicide).pack(side = LEFT)

    self.canvas.bind("<Button-1>", self.onclick)
    self.root.bind("b", select_blue)
    self.root.bind("y", select_yellow)
    self.root.bind("o", select_orange)
    self.root.bind("u", select_unknown)
    self.root.bind("m", select_midpoint)
    self.root.bind("<BackSpace>", self.undo)
    self.root.bind("+", zoom_in)
    self.root.bind("-", zoom_out)
    self.root.bind('<Control-s>', self.save)
    self.root.bind('<Control-c>', self.suicide)

    self.root.after(500, self.check)
    signal.signal(signal.SIGINT, self.suicide)

    self.root.mainloop()

  def select_tool(self, tool):
    self.current_tool = tool

  def onclick(self, event):
    x = self.canvas.canvasx(event.x)
    y = self.canvas.canvasy(event.y)
    if self.current_tool == "unknown": colour = "grey"
    elif self.current_tool == "midpoint": colour = "red"
    else: colour = self.current_tool
    # draw a circle
    mod = self.dot_size * self.current_scale / 2
    shape = self.canvas.create_oval(x - mod, y - mod, x + mod, y + mod, fill = colour, width = 0)
    self.shapes.append(Oval(event, shape, self.current_tool, self.current_scale))
    if self.current_tool == "unknown" or self.current_tool == "orange": return
    # draw a line from the circle to the last circle if it was the same colour. adjust for scale differences.
    rx = - self.canvas.canvasy(y - self.canvas_hy) / self.px_to_m / self.current_scale
    ry = - self.canvas.canvasx(x - self.canvas_hx) / self.px_to_m / self.current_scale
    self.text.insert(0.0, f"new point : {self.current_tool}, {rx}, {ry}\n")
    for index in range(len(self.shapes) - 2, -1, -1):
        shape = self.shapes[index]
        if shape.tool != self.current_tool or not shape.is_oval(): continue
        le = shape.event
        ls = shape.scale
        lx = self.canvas.canvasx(self.canvas_hx + (le.x - self.canvas_hx) / ls * self.current_scale)
        ly = self.canvas.canvasx(self.canvas_hy + (le.y - self.canvas_hy) / ls * self.current_scale)
        shape = self.canvas.create_line(lx, ly, x, y, fill = colour, width = 2)
        self.shapes.append(Line(event, shape, self.current_tool, self.current_scale))
        break

  def undo(self, *args):
    # delete last shape and maybe the one before it if it was a same colour line
    if len(self.shapes) != 0: self.canvas.delete((ls := self.shapes.pop()).id)
    if len(self.shapes) != 0 and self.shapes[-1].tool == ls.tool and not ls.is_oval(): self.canvas.delete(self.shapes.pop().id)

  def scale(self, change):
    self.current_scale *= change
    self.canvas.scale("all", self.canvas_hx, self.canvas_hy , change, change)
    self.redraw_grid()

  def suicide(self, *args):
    self.root.destroy()
    print('caught ^C')

  def check(self):
    self.root.after(500, self.check)

  def save(self, *args):
    filename = asksaveasfilename()
    with open(filename, "w", newline = "") as csvfile:
      writer = csv.writer(csvfile, delimiter = ",", quotechar = "'", quoting = csv.QUOTE_MINIMAL)
      writer.writerow(["tag",	"x",	"y",	"direction",	"x_variance",	"y_variance",	"xy_covariance"])
      for shape in self.shapes:
        if not shape.is_oval(): continue
        tool = shape.tool
        e = shape.event
        s = shape.scale
        rx = - self.canvas.canvasy(e.y - self.canvas_hy) / self.px_to_m / s
        ry = - self.canvas.canvasx(e.x - self.canvas_hx) / self.px_to_m / s
        writer.writerow([tool, rx, ry, "0", "0.01", "0.01", "0"])
      writer.writerow(["car_start", "0", "0", "0", "0", "0", "0"])

  def redraw_grid(self):
    # draws lines to form grid of metre wide squares
    for line in self.background_lines:
      self.canvas.delete(line)
    mod = self.current_scale * self.px_to_m
    for xline in range(1, int(self.canvas_y // mod)):
      for yline in range(1, int(self.canvas_x // mod)):
        id = self.canvas.create_line(self.canvas.canvasx(0), self.canvas.canvasy(xline * mod),
                                     self.canvas.canvasx(self.canvas_x), self.canvas.canvasy(xline * mod), fill = "grey", width = 1)
        self.background_lines.append(id)
        id = self.canvas.create_line(self.canvas.canvasy(yline * mod), self.canvas.canvasy(0),
                                     self.canvas.canvasy(yline * mod), self.canvas.canvasy(self.canvas_y), fill = "grey", width = 1)
        self.background_lines.append(id)


def main():
  Drawer(1600, 1000)

if __name__ == "__main__": main()