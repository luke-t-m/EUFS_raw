import tkinter as tk

root = tk.Tk()

myContainer1 = tk.Frame(root)
myContainer1.pack()

button1 = tk.Button(myContainer1)
button1.configure(text="sneed")
button1.pack()

root.mainloop()
