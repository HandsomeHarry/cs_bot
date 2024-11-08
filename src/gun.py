class Gun:
    def __init__(self, name="Rifle", damage=25, fire_rate=0.2, accuracy=0.8, ammo_capacity=30):
        self.name = name
        self.damage = damage
        self.fire_rate = fire_rate
        self.accuracy = accuracy
        self.ammo_capacity = ammo_capacity
        self.ammo = ammo_capacity