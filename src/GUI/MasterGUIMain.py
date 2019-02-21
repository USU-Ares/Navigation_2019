'''
Original Author: Kaden Archibald
ARES Team - Navigation & Autonomy

Utah State University
Department of Mechanical and Aerospace Engineering

Created: Jan 11, 2019
Revised: Feb 20, 2019
Version: IPython 6.2.1 (Anaconda distribution) with Python 3.6.4

Driver Code for Main GUI
'''


try:
    import MasterGUI as gui
except ModuleNotFoundError:
    print('MasterGUI.py source code not found')
    input('Press return to exit')
    quit()


def main(*args, **kwargs):
    
    # Create application
    application = gui.MasterGUI(master = gui.root)
    application.master.title('USU Ares Rover')
    
    # Run application
    application.appExec()
    
    return None



if __name__ == '__main__':
    try:
        print('Starting GUI...')
        main()
        
    except KeyboardInterrupt as keyStop:
        print('Error: ', keyStop)
        
    except gui.tk.TclError as tkStop:
        pass
        
    finally:
        print('Terminating GUI...')
        gui.MasterGUI.halt(gui.MasterGUI)
        