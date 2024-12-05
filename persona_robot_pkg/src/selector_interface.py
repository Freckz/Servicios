#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import String
import pyttsx3

class InterfaceSelector:
    def __init__(self):
        rospy.init_node('interface_selector_node')
        self.engine = pyttsx3.init()
        self.selector_node = rospy.get_name()  # Obtiene el nombre del nodo actual
        self.subVoice = rospy.Subscriber('voice_commands_persona', String, self.process_and_speak)
        self.interface_selected = False  # Bandera para verificar si ya se ha elegido una interfaz

    def process_and_speak(self, msg):
        # Si ya se ha elegido una interfaz, no procesamos más comandos
        if self.interface_selected:
            return

        rospy.loginfo(f"Comando recibido: {msg.data}")
        response = ""

        # Generar respuesta según el comando de voz
        if msg.data == "visual":
            response = "Se va a iniciar la interfaz gráfica."
        elif msg.data == "voz":
            response = "Se va a iniciar la interfaz de voz."
        elif msg.data == "ambas":
            response = "Se van a inicializar ambas interfaces."
        elif msg.data == "salir":
            response = "Finalizando la operación."
        else:
            response = "No entendí el comando. Por favor, repítelo."

        rospy.loginfo(f"Respuesta generada: {response}")

        # Reproducir la respuesta con pyttsx3
        self.engine.say(response)
        self.engine.runAndWait()
        self.engine.stop()

        # Lanzar el comando para ejecutar la interfaz correspondiente
        self.control_interface(msg.data)

        # Marcar que se ha hecho la selección y detener el nodo selector
        self.interface_selected = True
        
        self.kill_selector_node()

    def control_interface(self, command):
        """ Lanzar el comando para iniciar la interfaz correspondiente """
        if command == "visual":
            rospy.loginfo("Iniciando la interfaz gráfica...")
            os.system("roslaunch persona_robot_pkg visual.launch")
            # Matar el nodo de procesamiento de voz, ya que no se necesita
            self.kill_voice_processing_node()
        elif command == "voz":
            rospy.loginfo("Iniciando la interfaz de voz...")
            os.system("roslaunch persona_robot_pkg voz.launch")
        elif command == "ambas":
            rospy.loginfo("Iniciando ambas interfaces...")
            os.system("roslaunch persona_robot_pkg todo.launch")
        elif command == "salir":
            rospy.loginfo("Finalizando todos los nodos...")
            os.system("rosnode kill all")  # Matar todos los nodos (detener todo)
        else:
            rospy.logwarn("Comando no reconocido para iniciar interfaz.")

    def kill_selector_node(self):
        # """ Matar el nodo selector una vez que se ha elegido una interfaz """
        rospy.loginfo(f"Matar el nodo selector: {self.selector_node}")
        os.system(f"rosnode kill {self.selector_node}")

    def kill_voice_processing_node(self):
        # """ Matar el nodo de procesamiento de voz una vez que se selecciona una interfaz """
        rospy.loginfo("Matar el nodo de procesamiento de voz")
        os.system("rosnode kill /escuhar")  # Matar el nodo de procesamiento de voz
        rospy.signal_shutdown("Interfaz seleccionada. Nodo de voz detenido.")  # Finaliza el nodo actual

    def listen_and_publish(self):
        rospy.loginfo("Esperando comandos de voz para elegir interfaz...")
        rospy.spin()

if __name__ == '__main__':
    try:
        interface_selector = InterfaceSelector()
        interface_selector.listen_and_publish()
    except rospy.ROSInterruptException:
        pass
