#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
import pyttsx3

# Variable global para saber si estamos realizando un movimiento complejo
is_complex = None   

def process_and_speak(msg):
    rospy.loginfo(f"Comando recibido: {msg.data}")
    response = ""

    global is_complex       # Puede que no haga falta el global

    # Genera una respuesta basada en el comando
    # Comandos básicos
    if msg.data == "adelante":
        response = "Avanzando hacia adelante."
    elif msg.data == "atrás":
        response = "Retrocediendo."
    elif msg.data == "izquierda":
        response = "Girando a la izquierda."
    elif msg.data == "derecha":
        response = "Girando a la derecha."
    elif msg.data == "detener":
        response = "Deteniéndome."
    
    # Comandos complejos
    if is_complex is None:

        if msg.data == "cocina":
            response = "Dirigiéndome a la cocina."
        elif msg.data == "comedor":
            response = "Dirigiéndome al comedor."
        elif msg.data == "habitación":
            response = "Dirigiéndome a la habitación."
        
    elif msg.data == "reanudar":
        response = "Reanudando la marcha"
    elif msg.data == "finalizar":
        response = "Finalizando operación. Hasta luego."

    rospy.loginfo(f"Respuesta generada: {response}")

    # Reproducir la respuesta con pyttsx3
    engine = pyttsx3.init()
    engine.say(response)
    engine.runAndWait()
    engine.stop()

def configure_engine():
    global engine
    engine = pyttsx3.init()

    # Ajustar la velocidad (rate) a un valor más lento
    rate = engine.getProperty('rate')  # Obtiene el valor actual
    rospy.loginfo(f"Velocidad actual de síntesis: {rate}")
    engine.setProperty('rate', 220)  # Reduce la velocidad a 150 palabras por minuto

    # Ajustar el volumen
    volume = engine.getProperty('volume')  # Obtiene el volumen actual
    rospy.loginfo(f"Volumen actual de síntesis: {volume}")
    engine.setProperty('volume', 0.9)  # Establece el volumen al 90%

    # Cambiar la voz (elige una más natural si está disponible)
    voices = engine.getProperty('voices')
    rospy.loginfo("Voces disponibles:")
    for idx, voice in enumerate(voices):
        rospy.loginfo(f"{idx}: {voice.name} ({voice.languages})")
    engine.setProperty('voice', voices[26].id)  # Cambia a la primera voz disponible (ajusta según preferencia)

def complex(msg):
    global is_complex
    is_complex = msg.data
    rospy.loginfo(f"Estado de movimiento complejo actalizado: {is_complex}")

def command_processor_and_speaker():
    rospy.init_node('command_processor_and_speaker_node')
    configure_engine()
    rospy.Subscriber('voice_commands_persona', String, process_and_speak)
    rospy.Subscriber('complex', Bool, complex)
    rospy.loginfo("Nodo combinado de procesamiento y síntesis listo.")
    rospy.spin()
    engine.stop()  # Detener el motor al finalizar

if __name__ == '__main__':
    try:
        command_processor_and_speaker()
    except rospy.ROSInterruptException:
        pass
