import speech_recognition as sr
import os

r = sr.Recognizer()
with sr.Microphone() as micro:
    
    os.system('clear')
    print('Escuchando.', end='',flush=True)
    while True:

        print(".", end='')
        audio = r.listen(micro)

        try:
            text = r.recognize_google(audio,language="es-ES")
            print('\n Has dicho: {}'.format(text))
            
            text = text.lower().split()
            
            if('finalizar' in text):
                print("Tarea terminada")
                break

            print('Escuchando.', end='',flush=True)

        except:
            print('.', end='',flush=True)
