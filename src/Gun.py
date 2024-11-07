class Gun:
    def __init__(self, name="Rifle", damage=25, fire_rate=0.2, accuracy=0.8, ammo_capacity=30):
        self.name = name
        self.damage = damage
        self.fire_rate = fire_rate
        self.accuracy = accuracy
        self.ammo_capacity = ammo_capacity
        self.ammo = ammo_capacity

class Player:
    def __init__(self, namespace, gun=None):
        self.namespace = namespace
        # If gun is None, create a default Gun object
        self.gun = gun if gun is not None else Gun()
        
    def __repr__(self):
        return f"Player(namespace={self.namespace}, gun={self.gun.name})"


    def reload(self):
        self.ammo = self.ammo_capacity
        print(f"{self.name} reloaded. Ammo: {self.ammo}/{self.ammo_capacity}")

    def shoot(self):
        if self.ammo > 0:
            self.ammo -= 1
            print(f"{self.name} fired! Ammo left: {self.ammo}/{self.ammo_capacity}")
            return True  # Indicates a shot was fired
        else:
            print(f"{self.name} is out of ammo! Reload needed.")
            return False  # No shot fired
