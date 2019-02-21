'''
Kaden Archibald
USU Ares Team

Created: Jan 11, 2019
Revised: Feb 14, 2019
Version: IPython 6.2.1 (Anaconda distribution) with Python 3.6.4

Main GUI Source Code
'''

# GUI Module
import tkinter as tk
requiredVersion = 8.5
if tk.TkVersion < requiredVersion:
    # Check version. Should be 8.5 or higher
    raise ValueError('Please update version of tkinter')
    
# Image Tools
from PIL import Image, ImageTk
    
from time import sleep
from random import randint

# Used to embed matplotlib plots into a tkinter gui
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib import pyplot as plt

# Neccessary import and use() call if matplotlib backend is not correctly set:
#import matplotlib
#matplotlib.use('TkAgg')

try:
    import DataManipulation as dm
except ModuleNotFoundError:
    print('DataManipulation.py source code not found')
    input('Press return to exit')
    quit()


# Main window
# Do not delete!
root = tk.Tk()


class MasterGUI(tk.Frame):
    def __init__(self, master = None):

        # Initialize object with the tkinter frame class
        self.master = master
        tk.Frame.__init__(self, self.master)
        self.grid()
        
        # Initialize member variables
        self.xData = []
        self.yData = []
        self.videoPanel = None
        
        self.figs = 3

        # Create widgets and elements
        self.createPlot()
        self.createCompass()
        #self.createWidgets()


    def appExec(self):
        sleepTime = 1  # sec
        
        # Initialize ros subscriber
        
        while True:
            # Update plot must take place while the gui runs, so implement
            # tkinter.mainloop() maually using frame update and frame update
            # idle tasks in an infinte loop.
            tk.Frame.update_idletasks(self)
            tk.Frame.update(self)
            
            # Update information
            self.updatePlot()
            self.updateCompass()
            self.updateTextBoxes()
            
            # Spin the ros node
            #ros.spinOnce()

            sleep(sleepTime)

        #tk.mainloop()

    def halt(self):
        root.quit()
        #root.destroy()


    def createWidgets(self):
        ''' Create buttons, slider, etc. '''
        
        quitButton = tk.Button(self.master, text = 'Exit', command = self.halt)
        quitButton.grid(column = 0, row = 1, sticky = 'W')
        
        
    def getBearing(self):
        ''' Return the bearing from a ros node. '''
        
        # Use ros here
        
        # Placeholder in order to display somethiing in the test application
        return randint(0, 360)
    
    
    def updateTextBoxes(self):
        ''' Update any text information in the gui. '''
        
        textColumn = 2
        
        # Create text box for bearing
        bearingMessage = 'Bearing: ' + str(self.getBearing())
        self.bearingText = tk.Label(self.master, text = bearingMessage)
        self.bearingText.grid(column = textColumn, row = 0, sticky = 'N')
        
        # Create text box for x position
        thisPos = dm.getLatestPositionData()
        posxMessage = 'X-pos: ' + str(round(thisPos[0], self.figs))
        self.xText = tk.Label(self.master, text = posxMessage)
        self.xText.grid(column = textColumn, row = 1, sticky = 'N')
        
        # Create text box for y position
        posyMessage = 'Y-pos: ' + str(round(thisPos[1], self.figs))
        self.yText = tk.Label(self.master, text = posyMessage)
        self.yText.grid(column = textColumn, row = 2, sticky = 'N')
        
        return None


    def createPlot(self):
        ''' Initialize the matplotlib scatter plot. '''

        # Create the figure and axes
        #self.fig, self.axes = plt.subplots()
        self.fig = plt.figure(1)
        self.axes = plt.subplot()
        self.axes.plot(0,0)
        
        # Embed in tkinter GUI
        self.fig.canvas = FigureCanvasTkAgg(self.fig, master = root)
        self.fig.canvas.get_tk_widget().grid(column = 0, row = 0)
        

    def updatePlot(self):
        '''
        Get the new position data from file and add that point to the
        scatter plot.
        '''

        # Fetch and plot data
        newPos = dm.getLatestPositionData()
        self.xData.append(newPos[0])
        self.yData.append(newPos[1])
        self.axes.plot(self.xData, self.yData, color = 'black')

        # Formatting
        self.axes.set_title('X and Y Position Relative to Starting Point')
        self.axes.set_xlabel('meters')
        self.axes.set_ylabel('meters')
        
        # Add data to plot and flush
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


    def createCompass(self):
        ''' Initialize the matplotlib polar plot. '''
        
        # Create the figure and axes
        self.polar = plt.figure(2)
        self.pole = plt.subplot(polar = True)
        self.pole.plot(0,0)
        
        # Embed
        self.polar.canvas = FigureCanvasTkAgg(self.polar, master = root)
        self.polar.canvas.get_tk_widget().grid(column = 1, row = 0)


    def updateCompass(self):
        ''' Get new bearing data and update plot. '''
        
        # Clear the old compass point
        self.pole.cla()
        
        radii = [i/100 for i in range(2)]
        thisAngle = self.getBearing()
        angles = [thisAngle for i in range(2)]
        
        # Plot data
        self.pole.plot(angles, radii, color = 'black')
        
        # Add data and flush
        self.polar.canvas.draw()
        self.polar.canvas.flush_events()
        

    def embedPhoto(self):
        ''' Embed a static image. '''

        # Specify where the video will be
        #videoDirectory = 'D:\Capstone\GUI\video\beeMovie.jpg'
        photoDir= 'beeMovie.jpg'
        
        # Open the image with PIL, then convert to tkinter friendly formatting
        framePIL = Image.open(photoDir)
        frameTK = ImageTk.PhotoImage(framePIL)
        
        # Embed the image into the gui
        self.videoPanel = tk.Label(image=frameTK)
        self.videoPanel.image = frameTK
        self.videoPanel.grid(column = 4, row = 4)
        
            





# Some free online resources used during development:
            
# https://www.pyimagesearch.com/2016/05/30/displaying-a-video-feed-with-
#opencv-and-tkinter/
# https://pillow.readthedocs.io/en/3.0.x/handbook/tutorial.html
# https://pillow.readthedocs.io/en/4.2.x/reference/Image.html
# https://www.tutorialspoint.com/python/tk_grid.htm


