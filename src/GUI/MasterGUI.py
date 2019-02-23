'''
Original Author: Kaden Archibald
ARES Team - Navigation & Autonomy
https://github.com/USU-Ares/Navigation_2019

Utah State University
Department of Mechanical and Aerospace Engineering

Created: Jan 11, 2019
Revised: Feb 20, 2019
Version: IPython 6.2.1 (Anaconda distribution) with Python 3.6.4

Main GUI Source Code
'''


# GUI Module
import tkinter as tk
# Check version. Should be 8.5 or higher
requiredVersion = 8.5
assert tk.TkVersion >= requiredVersion, 'Please update version of tkinter'


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
        
        # Initialize member variables. The member variables contain both
        # textual and graphical Navigation information.
        self.xData = []
        self.yData = []
        self.bearing = None
        self.videoPanel = None
        
        self.figs = 3                       # Sig figs for rounding
        self.updateCount = 0                # Plus one for every update cycle

        # Create widgets and elements
        self.inputGPS()
        self.getBearing()
        self.createPlot()
        self.createCompass()
        self.createWidgets()
        #self.embedPhoto()


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
            self.getBearing()
            
            # Spin the ros node
            #ros.spinOnce()

            sleep(sleepTime)
            self.updateCount += 1

        #tk.mainloop()
        
        return None


    def halt(self):
        root.quit()
        #root.destroy()
        
        return None


    def createWidgets(self):
        ''' Create buttons, slider, etc. '''
        
        quitButton = tk.Button(self.master, text = 'Exit', command = self.halt)
        quitButton.grid(column = 2, row = 1, sticky = 'SW')
        
        return None
        
        
    def getBearing(self):
        ''' Return the bearing from a ros node. '''
        
        # Use ros here
        
        # Placeholder in order to display something in the test application
        self.bearing = randint(30, 60)
        return None
    
    
    def updateTextBoxes(self):
        ''' Update any text information in the gui. '''
        
        # Create text box for bearing and xy-coordinate
        msg = ''
        thisPos = dm.getLatestPositionData(self.updateCount)
        
        msg += 'X-pos: ' + str(round(thisPos[0], self.figs)) + '\n'
        msg += 'Y-pos: ' + str(round(thisPos[1], self.figs)) + '\n'
        msg += 'Bearing: ' + str(self.bearing) + '\n'
        
        # And embed this text box in the application
        self.bearingText = tk.Label(self.master, text = msg)
        self.bearingText.config(width = 20)
        self.bearingText.config(font = ('Consolas', 12))
        
        textColumn = 2
        textRow = 0
        self.bearingText.grid(column = textColumn, row = textRow, sticky = 'NW')
        
#        gps = ''
#        gps += 'Initial' + '\n'
#        gps += 'Lat: ' + str(round(self.waypoints['start'][0], self.figs))
#        gps += 'Lon: ' + str(round(self.waypoints['start'][1], self.figs))
#        
#        self.gpsText = tk.Label(self.master, text = gps)
        
        return None


    def createPlot(self):
        ''' Initialize the matplotlib scatter plot. '''

        # Create the figure and axes
        self.fig = plt.figure(1)
        self.axes = plt.subplot()
        
        # Embed in tkinter GUI
        self.fig.canvas = FigureCanvasTkAgg(self.fig, master = root)
        self.fig.canvas.get_tk_widget().grid(column = 0, row = 0)
        
        # Plot the first point
        self.axes.plot([0], [0], color = 'black', label = 'Path')
        
        # Plot the GPS waypoints
        self.axes.plot([0], [0], color = 'red', marker = 'o', \
                       label = 'Initial')
        newLoc = self.calcWaypoints()
        self.axes.plot([newLoc[0]], [newLoc[1]], color = 'green', \
                       marker = 'o', label = 'Final')
        
        return None
        

    def updatePlot(self):
        '''
        Get the new position data from file and add that point to the
        scatter plot.
        '''

        # Fetch and plot data
        newPos = dm.getLatestPositionData(self.updateCount)
        self.xData.append(newPos[0])
        self.yData.append(newPos[1])
        self.axes.plot(self.xData, self.yData, color = 'black')

        # Formatting
        self.axes.set_title('X and Y Position Relative to Starting Point')
        self.axes.set_xlabel('meters')
        self.axes.set_ylabel('meters')
        self.axes.legend(loc = 'lower right')
        
        # Add data to plot and flush
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        return None


    def createCompass(self):
        ''' Initialize the matplotlib polar plot. '''
        
        # Create the figure and axes
        self.polar = plt.figure(2)
        self.pole = plt.subplot(polar = True)
        self.pole.set_yticklabels([])
        self.pole.set_xticklabels(['E', 'NE', 'N', 'NW', 'W', 'SW', 'S', 'SE'])
        
        # Embed
        self.polar.canvas = FigureCanvasTkAgg(self.polar, master = root)
        self.polar.canvas.get_tk_widget().grid(column = 0, row = 1)
        
        return None


    def updateCompass(self):
        ''' Get new bearing data and update plot. '''
        
        # Clear the old compass point
        self.pole.cla()
        
        radii = [0, 1]
        # After many days of debugging, it was found that pyplot expects
        # theta coordiantes in radians even though the default theta 
        # coordinate axes display degrees. Good to know!
        thisAngle = dm.degToRad(self.bearing)
        angles = [thisAngle for i in range(2)]
        
        # Plot data
        self.pole.plot(angles, radii, color = 'red')
        
        # Formatting
        #self.pole.set_title('Bearing')
        self.pole.set_yticklabels([])
        self.pole.set_xticklabels(['E', 'NE', 'N', 'NW', 'W', 'SW', 'S', 'SE'])
        
        # Add data and flush
        self.polar.canvas.draw()
        self.polar.canvas.flush_events()
        
        return None
        

    def inputGPS(self):
        ''' End user must manually specify starting and stopping GPS. '''
        
#        textInputApp = tk.Tk()
#        textInput = tk.simpledialog.askfloat('Input GPS', \
#            'Enter Lat in Deg: ', parent = textInputApp, minvalue = -180, \
#            maxvalue = 180)
        
#        self.halt(textInputApp)
        
        
        self.waypoints = {}
        
#        startLat = float(input('Enter Starting Latitutde in Deg> '))
#        startLon = float(input('Enter Starting Longitude in Deg> '))
        startLat = 0
        startLon = 0
        self.waypoints['initial'] = [startLat, startLon]
        
#        endLat = float(input('Enter Ending Latitutde in Deg> '))
#        endLon = float(input('Enter Ending Longitude in Deg> '))
        endLat = 0.0005
        endLon = 0.0005
        self.waypoints['final'] = [endLat, endLon]
        
        # Before we do anything, convert to radians
        for key in self.waypoints.keys():
            #map(dm.degToRad, self.waypoints[key])
            for i in range(len(self.waypoints[key])):
                self.waypoints[key][i] = dm.degToRad(self.waypoints[key][i])
        
        return None
    
    
    def calcWaypoints(self):
        ''' Given the waypoints, find their xy-coordinates. '''
        
        startLoc = dm.haversine(*self.waypoints['initial'])
        endLoc = dm.haversine(*self.waypoints['final'])
        
        return [endLoc[0]-startLoc[0], endLoc[1]-startLoc[1]]



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
        self.videoPanel.grid(column = 4, row = 0)
        
        return None





# Some free online resources used during development:
            
# https://www.pyimagesearch.com/2016/05/30/displaying-a-video-feed-with-
#opencv-and-tkinter/
# https://pillow.readthedocs.io/en/3.0.x/handbook/tutorial.html
# https://pillow.readthedocs.io/en/4.2.x/reference/Image.html
# https://www.tutorialspoint.com/python/tk_grid.htm
# and lots of stack overflow

