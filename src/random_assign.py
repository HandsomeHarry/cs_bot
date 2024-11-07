import random
import rospy

def assign_personality_and_weapon():
    # Initialize the ROS node
    rospy.init_node('random_assign', anonymous=True)

    # Define
    personalities = ['rusher', 'normie', 'baiter']
    weapons = ['rifle', 'sniper', 'smg']

    # Assign random personality and weapon per robot
    for i in range(1, 5):
        personality = random.choice(personalities)
        weapon = random.choice(weapons)
        
        # Set params
        rospy.set_param(f'/robot{i}/personality', personality)
        rospy.set_param(f'/robot{i}/weapon', weapon)
        print(f'Robot {i}: Personality = {personality}, Weapon = {weapon}')

if __name__ == "__main__":
    try:
        assign_personality_and_weapon()
    except rospy.ROSInterruptException:
        pass
