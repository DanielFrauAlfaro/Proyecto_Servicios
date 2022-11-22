#!/usr/bin/python3
# -*- coding: utf-8 -*-
import speech_recognition as sr
import os
import rospy 
from std_msgs.msg import String

topic = "/voice_ui"

rospy.init_node("voice_ui")
pub = rospy.Publisher(topic,String,queue_size=10)
rate = rospy.Rate(10)

r = sr.Recognizer()
with sr.Microphone() as micro:

    os.system('clear')

    #r.adjust_for_ambient_noise(micro)
    print('Escuchando.', end='',flush=True)
    while not rospy.is_shutdown():

        print(".", end='')
        audio =  r.listen(micro, 10, 3)

        try:
            text = r.recognize_google(audio,language="es-ES")
            print('\n Has dicho: {}'.format(text))
            
            text = text.lower().split()
            mensaje = String()

            if("sal" in text):
                mensaje.data = "sal"
            elif("pimienta" in text):
                mensaje.data = "pimienta"
            elif("azúcar" in text):
                mensaje.data = "azucar"
            
            pub.publish(mensaje)

            print('Escuchando.', end='',flush=True)

        except:
            print('.', end='',flush=True)

        rate.sleep()

