#!/usr/bin/env python

import rospy
import random

class Gun:
    # Define weapon presets as class attributes
    WEAPON_TYPES = {
        'rifle': {
            'damage': 25,
            'fire_rate': 0.2,
            'accuracy': 0.8,
            'ammo_capacity': 30,
            'reload_time': 2.5
        },
        'sniper': {
            'damage': 100,
            'fire_rate': 1.0,  # Slower fire rate
            'accuracy': 0.95,  # Higher accuracy
            'ammo_capacity': 5,
            'reload_time': 3.5
        },
        'smg': {
            'damage': 15,
            'fire_rate': 0.1,  # Faster fire rate
            'accuracy': 0.7,   # Lower accuracy
            'ammo_capacity': 50,
            'reload_time': 1.5
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
        self.reload_time = specs['reload_time']
        self.ammo = self.ammo_capacity
        self.is_reloading = False
        self.last_shot_time = 0 # for reload checking
        
        rospy.loginfo(f"Initialized {weapon_type}: DMG={self.damage}, AMMO={self.ammo_capacity}")

    def can_shoot(self):
        """check if can shoot"""
        current_time = rospy.Time.now().to_sec()
        if self.is_reloading:
            if current_time - self.last_shot_time >= self.reload_time:
                self.is_reloading = False
                self.ammo = self.ammo_capacity
                return True
            return False
        return current_time - self.last_shot_time >= self.fire_rate:

    def shoot(self):
        """shooting"""
        if self.can_shoot():
            self.ammo -= 1
            self.last_shot_time = rospy.Time.now().to_sec()
            if self.ammo <= 0:
                self.is_reloading = True
            if random.random() <= accuracy:
                return self.damage
        return 0

    def get_stats(self):
        return {
            'name': self.name,
            'damage': self.damage,
            'fire_rate': self.fire_rate,
            'accuracy': self.accuracy,
            'ammo': self.ammo,
            'ammo_capacity': self.ammo_capacity
        }