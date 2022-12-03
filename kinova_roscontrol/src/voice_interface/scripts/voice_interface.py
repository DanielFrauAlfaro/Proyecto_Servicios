#!/usr/bin/python3
# -*- coding: utf-8 -*-

import speech_recognition as sr
import os
import rospy 
from std_msgs.msg import String
import pyttsx3 
from random import randint

# Lista de ingredientes disponibles
INGREDIENTS = ["sal","azucar","pimienta"]

# Text to speech
def tts(text):
    engine.say(text)
    engine.runAndWait()

# Función para que las frases sean aleatorias
def random_tts(text):
    comandos ={ 0:"Marchando",
                1:"Voy a por",
                2:"Cogiendo",
                3: "Agarrando" }

    tts(comandos[randint(0,3)] + text)

# Callback para modificar la lista de ingredientes disponibles
# Si estaba disponible significa que lo ha cogido, por lo que ya no está disponible
# Si no lo estaba significa que lo ha guardado, por lo que se vuelve disponible
def ingredients_cb(ingredient):
    if ingredient.data in INGREDIENTS: 
        INGREDIENTS.remove(ingredient.data)

    else: 
        INGREDIENTS.append(ingredient.data)

# Función main
if __name__ == "__main__":

    #Inicializaciones de ROS
    rospy.init_node("voice_ui")

    #Publica el ingrediente solicitado
    pub = rospy.Publisher("/voice_ui",String,queue_size=10)

    #Recibe si el ingrediente está disponible
    rospy.Subscriber("/ingredients",String,ingredients_cb)

    #Frecuencia de 10Hz
    rate = rospy.Rate(10)

    #Inicializacion del reconocimiento de voz
    r = sr.Recognizer()

    #Inicialización de texto a voz
    engine = pyttsx3.init()
    engine.setProperty('rate',120)
    engine.setProperty('voice','spanish')

    #Frase inicial
    tts("¿Qué ingredientes quieres?")

    #Texto reconocido
    text = ""

    #BUCLE
    while not rospy.is_shutdown():

        with sr.Microphone() as micro:

            # r.adjust_for_ambient_noise(micro)

            audio =  r.listen(micro,10,3)

            try:
                #Reconoce el texto
                text = r.recognize_google(audio,language="es-ES")
                print('\n Has dicho: {}'.format(text))

                text = text.lower().split()

            except:
                print('.', end='',flush=True)

        mensaje = String()
            
        if("sal" in text and "sal" in INGREDIENTS):
            random_tts("la sal")
            mensaje.data = "sal"
            pub.publish(mensaje)

        if("pimienta" in text and "pimienta" in INGREDIENTS):
            random_tts("la pimienta")
            mensaje.data = "pimienta"
            pub.publish(mensaje)

        if("azúcar" in text and "azucar" in INGREDIENTS):
            random_tts("el azúcar")
            mensaje.data = "azucar"
            pub.publish(mensaje)

        text = ""
        rate.sleep()

