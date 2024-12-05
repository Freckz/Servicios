#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from smach import State, StateMachine
from smach_ros import IntrospectionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib
from tf.transformations import quaternion_from_euler

# Variable global para el destino
global_goal = None

# Publicador para los comandos de movimiento directo
pub = None

# Publicador para no hablar durante la ruta
pub_ruta = None
complex = False

# Variables para reanudar la marcha
activate_detner = False
last_move = None

# Procesa comandos de voz para movimiento directo
def procesar_comando_voz(comando):
    twist = Twist()
    if comando == "adelante":
        twist.linear.x = 0.2
    elif comando == "atrás":
        twist.linear.x = -0.2
    elif comando == "izquierda":
        twist.angular.z = 0.5
    elif comando == "derecha":
        twist.angular.z = -0.5
    elif comando == "detener":
        twist.linear.x = 0.0
        twist.angular.z = 0.0
    else:
        return None  # Comando no reconocido para movimiento directo
    return twist


# Clase de estado para esperar un destino
class EsperarDestino(State):
    def __init__(self):
        State.__init__(self, outcomes=['nuevo_destino', 'basics', 'finalizar'])
        self.movimiento_activo = False  # Indica si se está en movimiento básico

    def execute(self, userdata):
        rospy.loginfo("Esperando un comando de voz para definir el destino...")

        global activate_detner, last_move

        # No se esta realizando un movimiento complejo
        global complex
        complex = False
        pub_ruta.publish(complex)

        global global_goal

        while not rospy.is_shutdown():
            comando = rospy.wait_for_message('voice_commands_persona', String).data.strip().lower()

            if comando == "finalizar":
                rospy.loginfo("Fin del programa.")
                return "finalizar"

            elif comando == "reanudar" and activate_detner:
                global_goal = last_move
                last_move = None
                activate_detner = False
                return 'nuevo_destino'

            # Si se están ejecutando comandos básicos
            if comando in ["adelante", "atrás", "izquierda", "derecha"]:
                rospy.loginfo(f"Comando básico '{comando}' recibido.")
                self.movimiento_activo = True
                twist = procesar_comando_voz(comando)
                if twist:
                    pub.publish(twist)
                continue  # No salir del bucle aún

            elif comando == "detener":
                rospy.loginfo("Comando 'detener' recibido. Movimiento básico detenido.")
                self.movimiento_activo = False
                twist = Twist()  # Publicar velocidades cero
                pub.publish(twist)
                continue

            # Si el robot no está en movimiento básico, aceptar comandos de destino
            if not self.movimiento_activo and comando in ["cocina", "comedor", "habitación"]:
                rospy.loginfo(f"Comando '{comando}' recibido. El robot se moverá a {comando}.")
                coordenadas = {
                    "cocina": (-1, 4, 1.57),
                    "comedor": (6, 1, 1.57),
                    "habitación": (-6, 2, 1.57)
                }

                if activate_detner: 
                    activate_detner = False

                x, y, yaw = coordenadas[comando]
                global_goal = self.crear_destino(x, y, yaw)
                return "nuevo_destino"

            rospy.loginfo("Comando no válido o no permitido en este estado.")
            continue

    def crear_destino(self, x, y, yaw=0.0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        quaternion = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        return goal


# Clase de estado final
class Finalizado(State):
    def __init__(self):
        State.__init__(self, outcomes=['end'])

    def execute(self, userdata):
        rospy.loginfo("El robot ha finalizado la tarea.")
        return 'end'


# Clase de estado para manejar los movimientos básicos
class MovimientoBasico(State):
    def __init__(self):
        State.__init__(self, outcomes=['detener', 'finalizar'])

    def execute(self, userdata):
        rospy.loginfo("El robot está listo para movimientos básicos. Envíe 'detener' para finalizar.")

        while not rospy.is_shutdown():
            comando = rospy.wait_for_message('voice_commands_persona', String).data.strip().lower()
            if comando == 'detener':
                # Publicar Twist con velocidades en cero para detener el robot
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pub.publish(twist)
                rospy.loginfo("Robot detenido.")
                return 'detener'
            elif comando == 'finalizar':
                rospy.loginfo("Finalizando el programa desde movimientos básicos.")
                return 'finalizar'
            else:
                # Procesar otros comandos de movimiento básico
                twist = procesar_comando_voz(comando)
                if twist:
                    pub.publish(twist)


# Clase de estado para mover el robot al destino
class MoverRobot(State):
    def __init__(self):
        State.__init__(self, outcomes=['detener', 'completado', 'fallido'])

    def execute(self, userdata):
        rospy.loginfo("El robot se está moviendo al destino.")

        global complex
        complex = True
        pub_ruta.publish(complex)    

        global activate_detner, last_move

        global global_goal
        # Cliente para la acción de move_base
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        move_base_client.wait_for_server()

        # Enviar el destino al cliente de acción
        goal = global_goal

        if activate_detner is False:
            last_move = goal 

        move_base_client.send_goal(goal)

        # Mientras el robot esté en movimiento, escuchar comandos de voz
        while not rospy.is_shutdown():
            # Verificar el estado del cliente
            state = move_base_client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Destino alcanzado con éxito.")
                complex = False
                pub_ruta.publish(complex)
                return 'completado'
            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                rospy.loginfo("No se pudo alcanzar el destino.")
                complex = False
                pub_ruta.publish(complex)
                return 'fallido'

            # Escuchar comandos de voz
            try:
                comando = rospy.wait_for_message('voice_commands_persona', String, timeout=0.5).data.strip().lower()
                if comando == 'detener':
                    rospy.loginfo("Comando 'detener' recibido. Cancelando el objetivo.")
                    move_base_client.cancel_goal()  # Se cancela el movimiento del SimpleActionState
                    complex = False
                    activate_detner = True
                    pub_ruta.publish(complex)
                    return 'detener'
            except rospy.ROSException:
                pass  # No se recibió ningún mensaje en el tiempo de espera

        rospy.loginfo("El robot finalizó el estado de mover robot.")
        return 'fallido'


if __name__ == '__main__':
    rospy.init_node("robot_control_por_voz")

    # Publicador de movimiento directo
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Publicador de movimiento complejo
    pub_ruta = rospy.Publisher('complex', Bool, queue_size=10)  

    # Crear la máquina de estados
    sm = StateMachine(outcomes=['end'])

    with sm:
        # Estado: EsperarDestino
        StateMachine.add(
            'EsperarDestino',
            EsperarDestino(),
            transitions={'nuevo_destino': 'MoverRobot', 'basics': 'MovimientoBasico', 'finalizar': 'Finalizado'}
        )

        # Estado: MoverRobot
        StateMachine.add(
            'MoverRobot',
            MoverRobot(),
            transitions={'detener': 'EsperarDestino', 'completado': 'EsperarDestino', 'fallido': 'EsperarDestino'}
        )

        # Estado: MovimientoBasico
        StateMachine.add(
            'MovimientoBasico',
            MovimientoBasico(),
            transitions={'detener': 'EsperarDestino', 'finalizar': 'Finalizado'}
        )

        # Estado final: Finalizado
        StateMachine.add(
            'Finalizado',
            Finalizado(),
            transitions={'end': 'end'}
        )

    # Servidor de introspección para SMACH
    sis = IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    rospy.loginfo("Nodo de control por voz iniciado.")
    sm.execute()
    rospy.spin()
