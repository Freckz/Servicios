#!/usr/bin/env python3
import rospy
import tkinter as tk
from tkinter import ttk
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageTk
from std_msgs.msg import String
import cv2

class RosTkinterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Teleoperation Interface")
        self.root.geometry("1770x500")

        # Inicializar ROS node
        rospy.init_node('robot_teleoperation', anonymous=True)
        
        # Publicador de ROS para comandos de movimiento y comandos de voz
        self.pub_movement = rospy.Publisher('robot_movement', String, queue_size=10)
        self.pub_voice = rospy.Publisher('voice_commands_persona', String, queue_size=10)
        
        # Suscriptor para el estado del robot
        rospy.Subscriber('voice_commands_persona', String, self.update_robot_status)

        # Suscriptor para las imágenes de la cámara
        rospy.Subscriber('/image', Image, self.update_camera_image)

        # Inicializar CvBridge
        self.bridge = CvBridge()

        # Crear elementos de la interfaz
        self.create_widgets()

        # Agregar la imagen para obtener coordenadas
        self.create_image_widgets()

    def create_widgets(self):
        label_font = ("Arial", 12)
        button_font = ("Arial", 12)
        entry_width = 15

        # Campos de coordenadas
        coord_frame = tk.Frame(self.root)
        coord_frame.grid(row=0, column=0, padx=10, pady=10, sticky="w")

        tk.Label(coord_frame, text="Coordenada x", font=label_font).grid(row=0, column=0, padx=5, pady=5)
        self.coord_x_entry = tk.Entry(coord_frame, width=entry_width)
        self.coord_x_entry.grid(row=0, column=1, padx=5, pady=5)

        tk.Label(coord_frame, text="Coordenada y", font=label_font).grid(row=0, column=2, padx=5, pady=5)
        self.coord_y_entry = tk.Entry(coord_frame, width=entry_width)
        self.coord_y_entry.grid(row=0, column=3, padx=5, pady=5)

        tk.Button(coord_frame, text="Enviar Coordenadas", font=button_font, command=self.send_coordinates).grid(row=0, column=4, padx=5, pady=5)

        # Menú desplegable de habitaciones y otros lugares
        self.room_selector = ttk.Combobox(coord_frame, values=["Habitacion", "Cocina", "Comedor", "Baño"], state="readonly", width=entry_width)
        self.room_selector.grid(row=0, column=0, padx=5, pady=5)
        self.room_selector.set("Seleccionar destino")

        # Botones para detener y reanudar el robot
        button_frame = tk.Frame(self.root)
        button_frame.grid(row=1, column=0, padx=20, pady=10, sticky="w")

        tk.Button(button_frame, text="Parar el robot", bg="red", fg="white", font=button_font, command=self.stop_robot).grid(row=0, column=0, padx=10, pady=5)
        tk.Button(button_frame, text="Reanudar el robot", bg="green", fg="white", font=button_font, command=self.resume_robot).grid(row=0, column=1, padx=10, pady=5)

        # Área de visualización del estado del robot
        self.status_label = tk.Label(self.root, text="Robot moviéndose", font=("Arial", 14), borderwidth=2, relief="solid", width=20, height=2)
        self.status_label.grid(row=2, column=0, padx=20, pady=10, sticky="w")

        # Barra de progreso para el estado
        self.progress = ttk.Progressbar(self.root, orient="horizontal", length=500, mode="determinate")
        self.progress.grid(row=3, column=0, padx=20, pady=10, sticky="w")

        # Flechas para el movimiento
        control_frame = tk.Frame(self.root)
        control_frame.grid(row=4, column=0, padx=20, pady=10, sticky="w")

        tk.Button(control_frame, text="↑", font=button_font, width=5, command=self.move_forward).grid(row=0, column=1, padx=10, pady=5)
        tk.Button(control_frame, text="←", font=button_font, width=5, command=self.turn_left).grid(row=1, column=0, padx=10, pady=5)
        tk.Button(control_frame, text="↓", font=button_font, width=5, command=self.move_backward).grid(row=1, column=1, padx=10, pady=5)
        tk.Button(control_frame, text="→", font=button_font, width=5, command=self.turn_right).grid(row=1, column=2, padx=10, pady=5)

        # Ajustar la imagen de la cámara
        self.camera_label = tk.Label(self.root)
        self.camera_label.grid(row=0, column=5, rowspan=6, padx=10, pady=10)  # Cambié el índice de columna a 5

    def create_image_widgets(self):
        # Cargar la imagen para obtener coordenadas (ajustar la ruta según sea necesario)
        image_path = "/home/franciscocj/catkin_ws/src/persona_robot_pkg/src/map.pgm"
        self.image = PILImage.open(image_path)
        
        # Convertir la imagen a un formato compatible con Tkinter
        self.image_tk = ImageTk.PhotoImage(self.image)

        # Crear un widget Label para mostrar la imagen
        self.label = tk.Label(self.root, image=self.image_tk)
        self.label.grid(row=0, column=6, rowspan=6, padx=10, pady=10)  # Cambié el índice de columna a 6

        # Enlazar el clic izquierdo en la imagen a la función obtener_coordenadas
        self.label.bind("<Button-1>", self.obtener_coordenadas)

    def obtener_coordenadas(self, event):
        # Obtener las coordenadas relativas al Frame de la ventana pequeña
        x, y = event.x, event.y

        # Determinar qué habitación corresponde a las coordenadas
        if 203 <= x <= 300 and 42 <= y <= 96:
            self.send_command("Habitacion 1")
        elif 169 <= x <= 264 and 289 <= y <= 333:
            self.send_command("Habitacion 2")
        elif 79 <= x <= 159 and 285 <= y <= 330:
            self.send_command("Habitacion 3")
        elif 83 <= x <= 188 and 184 <= y <= 284:
            self.send_command("Habitacion 4")
        elif 93 <= x <= 191 and 34 <= y <= 141:
            self.send_command("Habitacion 5")
        elif 87 <= x <= 171 and 140 <= y <= 186:
            self.send_command("Habitacion 6")
        else:
            print(f"Coordenadas del clic: ({x}, {y}) - Sin habitación asociada")

    # Función para recibir y mostrar el estado del robot
    def update_robot_status(self, msg):
        self.status_label.config(text=msg.data)

    # Función para actualizar la imagen de la cámara
    def update_camera_image(self, msg):
        try:
            # Convertir la imagen de ROS a OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convertir la imagen de BGR a RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Convertir la imagen de OpenCV a un objeto PIL
            pil_image = PILImage.fromarray(rgb_image)

            # Convertir la imagen de PIL a formato compatible con Tkinter
            tk_image = ImageTk.PhotoImage(pil_image)

            # Actualizar la imagen en la interfaz
            self.camera_label.config(image=tk_image)
            self.camera_label.image = tk_image  # Mantener una referencia para evitar que se recoja la basura

        except Exception as e:
            rospy.logerr("Error al procesar la imagen: %s", str(e))

    # Funciones para los comandos del robot
    def send_coordinates(self):
        # Obtener las coordenadas desde el campo de texto o desde la habitación seleccionada
        coord_x = self.coord_x_entry.get()
        coord_y = self.coord_y_entry.get()

        if not coord_x and not coord_y:  # Si no hay coordenadas, tomar el destino seleccionado
            room = self.room_selector.get()
            if room != "Seleccionar habitación":
                self.pub_voice.publish(room)  # Publicar el lugar seleccionado al topic /voice_commands_persona
                rospy.loginfo(f"Comando enviado: {room}")
        else:
            coord_data = f"{coord_x},{coord_y}"
            self.pub_movement.publish(coord_data)
            self.pub_voice.publish(f"{coord_x},{coord_y}")

    def move_forward(self):
        self.send_command("adelante")

    def move_backward(self):
        self.send_command("atras")

    def turn_left(self):
        self.send_command("izquierda")

    def turn_right(self):
        self.send_command("derecha")

    def stop_robot(self):
        self.send_command("detener")

    def resume_robot(self):
        self.send_command("reanudar")

    def send_command(self, command):
        self.pub_movement.publish(command)
        self.pub_voice.publish(command)

# Código principal para ejecutar la aplicación
if __name__ == '__main__':
    root = tk.Tk()
    app = RosTkinterApp(root)
    root.mainloop()
