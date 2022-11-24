#!/usr/bin/env python3

from pynput import keyboard as kb
import rospy
from std_msgs.msg import String

rospy.init_node("teclas")
pub = rospy.Publisher("/teclas", String, queue_size=10)

def callback(tecla):
    s = String()
    
    print("Se ha pulsado la tecla ")
    
    if(str(tecla) == "'r'"):
        print("R")
        s.data = "rojo"
        
    elif(str(tecla) == "'g'"):
        print("G")
        s.data = "verde"
           
    elif(str(tecla) == "'b'"):
        print("B")
        s.data = "azul"

    pub.publish(s)


# Main
if __name__ == '__main__':
    
    
    print("------- Start --------")
    kb.Listener(callback).run()
