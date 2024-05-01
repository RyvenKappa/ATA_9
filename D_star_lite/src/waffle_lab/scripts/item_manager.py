#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Para poder recibir Strings con el elemento a borrar
from gazebo_msgs.srv import DeleteModel



def remove_gazebo_model(msg):
    """
    Removes a Gazebo model from the simulation.

    Args:
        model_name (str): The name of the model to remove.
    """
    try:
        # Create a service proxy for the delete_model service
        delete_model_proxy = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        # Call the service to delete the model
        if msg.data!="":
            delete_model_proxy(msg.data)
            rospy.loginfo(f"Model '{msg.data}' removed from Gazebo.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Error calling delete_model service: {e}")



def main_loop():
    #Comunicación con ROS publicando los topics en los que se espera recibir mensajes
    rospy.init_node('trash_manager')
    sub = rospy.Subscriber("/manage_trash",String,remove_gazebo_model)
    rate = rospy.Rate(4)
    while True:
        pass

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
