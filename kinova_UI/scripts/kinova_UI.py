#! /usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import threading
from tkinter import *
from geometry_msgs.msg import Twist
import subprocess
from std_msgs.msg import String

import threading


class UI:

    def __init__(self, master=None):
        
        self._enable = False

        self.__real_robot_bt = Button(master, text="Real Robot",bg='#49A', width = 10)
        self.__virtual_robot_bt = Button(master, text="Virtual Robot",bg="green", width = 10)
        

        #Botones
        self.__real_robot_bt.bind("<Button-1>", self.real_robot_launch)
        self.__virtual_robot_bt.bind("<Button-1>", self.virtual_robot_launch)
        self.__real_robot_bt.grid(row=0, column=0, columnspan=1)
        self.__virtual_robot_bt.grid(row=1, column=0, columnspan=1)
        
        #Etiquetas
        self.__label1 = Label(master,text="SELECCIONE EL TIPO DE ROBOT")
        self.__label1.grid(column=4, row=0, padx=10, pady=1)
        self.__label1.config(bg="white")

        self.__label2 = Label(master,text="",bg='white')
        self.__label2.grid(column=4, row=1, padx=10, pady=1)
        self.__label2.config(bg="white")

        #Imagen
        self.__image=PhotoImage(file="src/kinova_UI/image/kinovaPhoto.png")
        self.__imageLable=Label(master,image=self.__image)
        self.__imageLable.config(bg="white")
        self.__imageLable.place(x=100, y=100, width=200, height=80)
        
        self.__robot_selected = False
        
        # Lista de comandos
        self.__cmd = []
        

    def real_robot_launch(self, event):
        if not self.__robot_selected:
            self.__robot_selected = True
            
            subprocess.run(["gnome-terminal","--", "sh", "-c", "roslaunch controllers scullion.launch type:=real"])
            
            #Para que no de error de conexión
            rospy.sleep(3)

            #Se inicializa el nodo
            rospy.init_node("kinova_ui")
            
            # Suscriptor al nodo del control por voz
            rospy.Subscriber("/voice_ui", String, self.__cb)
            
            # Suscriptor al nodo de la cámara
            rospy.Subscriber("/ready", String, self.__cb)

            self.__label1.config(text="¿QUÉ INGREDIENTE DESEA?")



    def virtual_robot_launch(self, event):
        if not self.__robot_selected:
            self.__robot_selected = True
            #Abre terminal y ejecuta el comando
            subprocess.run(["gnome-terminal","--", "sh", "-c","roslaunch controllers scullion.launch"])

            #Para que no de error de conexión
            rospy.sleep(3)

            #Se inicializa el nodo
            rospy.init_node("kinova_ui")
            
            # Suscriptor al nodo del control por voz
            rospy.Subscriber("/voice_ui", String, self.__cb)
            
            # Suscriptor al nodo de la cámara
            rospy.Subscriber("/ready", String, self.__cb)

            self.__label1.config(text="¿QUÉ INGREDIENTE DESEA?")

    #Resetea las etiquetas
    def reset_label(self, kinova_frame):

        self.__label2.config(text="")


    # Bucle de control
    def control_loop(self,kinova_frame):
        
            # Bucle de control
            if not rospy.is_shutdown():

                # Comprueba que hay un comando a realizar
                if len(self.__cmd) > 0:

                    # Saca el comando de la lista de comandos
                    command = self.__cmd.pop(0)
                    command = command.lower().split()

                    #Mensaje para comunicar a la interfaz el ingrediente que se coge
                    mensaje = String()
                    
                    # Realiza el pick and place si tiene que dar un ingrediente, luego pone la tupla a False (no hay ingrediente en la zona de almacén)
                    if len(command) == 1:

                        if command[0] == "sal":
                            self.__label2.config(text="Cogiendo la sal")
                            kinova_frame.after(15000, self.reset_label,kinova_frame)

                            
                        elif command[0] == "azúcar":
                            self.__label2.config(text="Cogiendo el azúcar")                    
                            kinova_frame.after(15000, self.reset_label,kinova_frame)
                                               
                        elif command[0] == "pimienta":
                            self.__label2.config(text="Cogiendo la pimienta")
                            kinova_frame.after(15000, self.reset_label,kinova_frame)

                    elif len(command) > 1:

                        X = float(command[0])
                        Y = float(command[1])
                        # Realiza el pick and place si tiene que devolver un ingrediente, luego pone la tupla a True (hay ingrediente en la zona de almacén)
                        if command[2] == "0":
                            self.__label2.config(text="Devolviendo la sal")
                            kinova_frame.after(15000, self.reset_label,kinova_frame)
                           
                        
                        elif command[2] == "1":
                            self.__label2.config(text="Devolviendo la pimienta")
                            kinova_frame.after(15000, self.reset_label,kinova_frame)

                            
                        elif command[2] == "2":
                            self.__label2.config(text="Devolviendo el azúcar")
                            kinova_frame.after(15000, self.reset_label,kinova_frame)


                kinova_frame.after(500, self.control_loop,kinova_frame)

     
     
     
    # Callback de la interfaz por voz y cámara: recoge los comandos y los almacena
    def __cb(self, data):
        self.__cmd.append(data.data)
        	
def main():

    try:

        kinova_frame = Tk()
        kinova_frame.geometry("370x200")
        kinova_frame.title("Kinova UI")
        kinova_frame['bg']= 'white'
        kinova_frame.resizable(width=False, height=False)

        kinova_UI=UI(kinova_frame)

        kinova_UI.control_loop(kinova_frame)
        kinova_frame.mainloop()

        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  
    main()

  
    
