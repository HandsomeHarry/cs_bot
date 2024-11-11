import rospy

class Gun:
    # Define weapon presets as class attributes
    WEAPON_TYPES = {
        'rifle': {
            'damage': 25,
            'fire_rate': 0.2,
            'accuracy': 0.8,
            'ammo_capacity': 30
        },
        'sniper': {
            'damage': 100,
            'fire_rate': 1.0,  # Slower fire rate
            'accuracy': 0.95,  # Higher accuracy
            'ammo_capacity': 5
        },
        'smg': {
            'damage': 15,
            'fire_rate': 0.1,  # Faster fire rate
            'accuracy': 0.7,   # Lower accuracy
            'ammo_capacity': 50
        }
    }

    def __init__(self, weapon_type='rifle'):
        # Convert to lowercase and default to rifle if invalid type
        weapon_type = weapon_type.lower()
        if weapon_type not in self.WEAPON_TYPES:
            rospy.logwarn(f"Invalid weapon type '{weapon_type}'. Defaulting to rifle.")
            weapon_type = 'rifle'
        
        # Get weapon specs from preset
        specs = self.WEAPON_TYPES[weapon_type]
        
        # Initialize attributes
        self.name = weapon_type
        self.damage = specs['damage']
        self.fire_rate = specs['fire_rate']
        self.accuracy = specs['accuracy']
        self.ammo_capacity = specs['ammo_capacity']
        self.ammo = self.ammo_capacity
        
        rospy.loginfo(f"Initialized {weapon_type}: DMG={self.damage}, AMMO={self.ammo_capacity}")

    def shoot(self):
        if self.ammo > 0:
            self.ammo -= 1
            rospy.logdebug(f"{self.name} fired! Ammo left: {self.ammo}")
            return True
        rospy.logdebug(f"{self.name} out of ammo!")
        return False
    
    def reload(self):
        self.ammo = self.ammo_capacity
        rospy.loginfo(f"{self.name} reloaded! Ammo: {self.ammo}")

    def get_stats(self):
        return {
            'name': self.name,
            'damage': self.damage,
            'fire_rate': self.fire_rate,
            'accuracy': self.accuracy,
            'ammo': self.ammo,
            'ammo_capacity': self.ammo_capacity
        }
