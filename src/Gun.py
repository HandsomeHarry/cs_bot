class Gun:
    def __init__(self, name, damage, fire_rate, accuracy, ammo_capacity):
        self.name = name
        self.damage = damage
        self.fire_rate = fire_rate  # Shots per second
        self.accuracy = accuracy  # Percentage chance to hit
        self.ammo_capacity = ammo_capacity
        self.ammo = ammo_capacity

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
