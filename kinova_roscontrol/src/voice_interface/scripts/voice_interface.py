#!/usr/bin/python3
# -*- coding: utf-8 -*-

import speech_recognition as sr
import os
import rospy 
from std_msgs.msg import String
import pyttsx3 
from random import randint

# Lista de ingredientes disponibles
INGREDIENTS = ["sal","azúcar","pimienta"]

# Text to speech
def tts(text):
    print(f"\n -kinova: {text}")
    engine.say(text)
    engine.runAndWait()

# Función para que las frases sean aleatorias
def random_tts_phrases(text,returning=False):
    phrases ={  0: "Marchando",
                1: "Voy a por",
                2: "Cogiendo",
                3: "Aquí tienes",
                4: "Guardando",
                5: "Dejando",
                6: "Almacenando",
                7: "Devolviendo" }
    
    offset = 0
    if returning: offset = 4 
    tts(f"{phrases[randint(0,3)+offset]} {text}")

# Callback para modificar la lista de ingredientes disponibles
def ingredients_cb(ingredient):
    if ingredient.data in INGREDIENTS:
        random_tts_phrases(ingredient.data) 
        INGREDIENTS.remove(ingredient.data)
    else:
        random_tts_phrases({ingredient.data}, returning=True) 
        INGREDIENTS.append(ingredient.data)

#Comandos de voz
def process_commands(text):
    #Listar ingredientes
    if "ingredientes" in text:
        tts(f"Los ingredientes disponibles son: {', '.join(INGREDIENTS)}")

    #Comandos de ingredientes    
    msg = String()

    for ingredient in INGREDIENTS:
        if ingredient in text:
            msg.data = ingredient
            pub.publish(msg)

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

    #Texto reconocido
    text = None

    #Comienzo
    setup = False
    tts("¿Qué ingredientes quieres?")

    #BUCLE
    while not rospy.is_shutdown():

        with sr.Microphone() as micro:
            
            #Primera iteracion
            if not setup:
                os.system("clear")
                print("Escuchando",end='',flush=True)
                setup = True

            r.adjust_for_ambient_noise(micro)
            audio =  r.listen(micro,None,4)

            try:
                #Reconoce el texto
                text = r.recognize_google(audio,language="es-ES")
                print(f"\n -Has dicho: {text}")
                text = text.lower().split()

            except sr.RequestError:
                #No hay conexion con la API
                tts("Conexión fuera de alcance")

            except sr.UnknownValueError:
                #No se pudo reconocer el habla
                print(".",end='',flush=True)

        if text is not None:
            process_commands(text)
            text = None
            os.system("clear")
            print("\nEscuchando",end='',flush=True)

        rate.sleep()

