#!/usr/bin/python
#
# use socat -d -d pty pty to emulate two serial ports
# 	run above before starting this script and use one of the pty's
#	as the serial port for the gui.py app
#
# reference for Tkinter: http://effbot.org/tkinterbook/tkinter-hello-tkinter.htm
#

import  sys
import  serial
import  Tkinter as tk

#
# window class
#
class pidControlApp(tk.Tk):
    def __init__(self,parent=None):
        tk.Tk.__init__(self,parent)
        self.parent = parent
        self.initialize()

    def initialize(self):
        self.grid()

        # Kp field
        kp_label = tk.Label(self,text=u"Kp")
        kp_label.grid(column=0,row=0,columnspan=1,sticky='E')
        self.kp_entryVariable = tk.StringVar()
        self.kp_entry = tk.Entry(self,textvariable=self.kp_entryVariable)
        self.kp_entry.grid(column=1,row=0,sticky='W')
        self.kp_entry.bind("<Return>", self.kp_OnPressEnter)

        # Ki field
        ki_label = tk.Label(self,text=u"Ki")
        ki_label.grid(column=0,row=1,columnspan=1,sticky='E')
        self.ki_entryVariable = tk.StringVar()
        self.ki_entry = tk.Entry(self,textvariable=self.ki_entryVariable)
        self.ki_entry.grid(column=1,row=1,sticky='W')
        self.ki_entry.bind("<Return>", self.ki_OnPressEnter)

        # Kd field
        kd_label = tk.Label(self,text=u"Kd")
        kd_label.grid(column=0,row=2,columnspan=1,sticky='E')
        self.kd_entryVariable = tk.StringVar()
        self.kd_entry = tk.Entry(self,textvariable=self.kd_entryVariable)
        self.kd_entry.grid(column=1,row=2,sticky='W')
        self.kd_entry.bind("<Return>", self.kd_OnPressEnter)

        # lean field
        lean_label = tk.Label(self,text=u"lean")
        lean_label.grid(column=0,row=3,columnspan=1,sticky='E')
        self.lean_entryVariable = tk.StringVar()
        self.lean_entry = tk.Entry(self,textvariable=self.lean_entryVariable)
        self.lean_entry.grid(column=1,row=3,sticky='W')
        self.lean_entry.bind("<Return>", self.lean_OnPressEnter)

        # terminal field
        # need to add scrollable contect: http://infohost.nmt.edu/tcc/help/pubs/tkinter/web/entry-scrolling.html
        self.term = tk.Text(self,width=30,fg="white",bg="black")
        self.term.grid(column=0,row=5,columnspan=3,padx=5,pady=5,sticky='EW')

        # port field
        port_label = tk.Label(self,text=u"serial")
        port_label.grid(column=0,row=11,columnspan=1,sticky='E')
        self.port_entryVariable = tk.StringVar()
        self.port_entry = tk.Entry(self,textvariable=self.port_entryVariable)
        self.port_entry.grid(column=1,row=11,sticky='W')
        self.port_entry.bind("<Return>", self.port_OnPressEnter)

        # connect button
        con_button = tk.Button(self,text=u"Connect",takefocus=True,command=self.con_OnButtonClick)
        con_button.grid(column=2,row=11,padx=5,pady=5,sticky='E')

        # exit button
        exit_button = tk.Button(self,text=u"Exit",takefocus=True,command=self.exit_OnButtonClick)
        exit_button.grid(column=2,row=12,padx=5,pady=5,sticky='E')

        self.grid_columnconfigure(0,weight=1)
        self.resizable(False,False)
        self.update()
        self.geometry(self.geometry())

#
# Kp field event handler
#
    def kp_OnPressEnter(self,event):
        command = "set kp "+self.kp_entryVariable.get()+"\r\n"
        try:
            self.ser.write(command)                             # send new kp value
        except AttributeError:
            self.term.insert(tk.END,"** serial error\n")        # print error message if connection failed
        else:
            line = self.handleReply()                           # ** not sure this will capture both echod command and response to the command
            self.term.insert(tk.END,line)                       # print the echod command and the response

        self.term.see(tk.END)
#
# Ki field event handler
#
    def ki_OnPressEnter(self,event):
        self.term.insert(tk.END,self.ki_entryVariable.get() + "\n")
        self.term.see(tk.END)

#
# Kd field event handler
#
    def kd_OnPressEnter(self,event):
        self.term.insert(tk.END,self.kd_entryVariable.get() + "\n")
        self.term.see(tk.END)

#
# lean field event handler
#
    def lean_OnPressEnter(self,event):
        self.term.insert(tk.END,self.lean_entryVariable.get() + "\n")
        self.term.see(tk.END)

#
# port field event handler
#
    def port_OnPressEnter(self,event):
        pass                                                        # do nothing on <enter>

#
# connect button event handler
#
    def con_OnButtonClick(self):
        # configure serial port and handle excptions
        try:
            self.ser = serial.Serial(self.port_entryVariable.get(),19200,timeout=1)
        except IOError:
            self.term.insert(tk.END,"** serial error\n")            # print error message if connection failed
        else:
            self.term.insert(tk.END,self.ser.name + " connected\n") # print connection status
            self.ser.write("\r\n\r\n")                              # send a CR+CR to the serial interface
            self.handleReply()
            self.ser.write("set prompt off\r\n")                    # turn off the 'prompt'
            self.handleReply()

        self.term.see(tk.END)

#
# exit button event handler
#
    def exit_OnButtonClick(self):
        # close serial port, other cleanup and exit
        try:
            self.ser.close()
        except:
            pass    # exit even if self.ser is not defined

        sys.exit()

#
# function to handle serial text responses
#   read text line from the serial input buffer and output to the window "terminal" field
#   the function will return the text that was received
#
    def handleReply(self):
        line = self.ser.readline()          # read reply from serial buffer to EOL
        self.term.insert(tk.END,line+"\n")  # print out
        self.term.see(tk.END)
        return line

#
# main program
#
if __name__ == "__main__":

    app = pidControlApp()
    app.title('my application')
    app.mainloop()
