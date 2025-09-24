class BaseStation:
    """
    Represents a base station (Raspberry Pi and DWM1001-DEV module) and related atributes.

    """
    
    def __init__(self, id, hostname, ip, x, y):
        """ 
        ID: String which is printed from the les command in minicom
            once connection has been established between the Pi and 
            DEV module. The print out will be of the form 
            ID[x,y,z]=dist. The ID is what you insert as this argument.

        x: Float, x coordinate of base station's position.

        y: Float, y coordinate of base station's position.
        
        hostname: String, hostname of the raspberry pi.

        ip: String, local ip address of the pi on your network.

        dist: Float, stores last recorded distance between rover and base station.

        """

        self.id = id
        self.x = x
        self.y = y
        self.hostname = hostname
        self.ip = ip
        self.dist = -1


    def __repr__(self):
        return f"Basestation(id={self.id}, x={self.x}, y={self.y}, hostname={self.hostname}, ip={self.ip})"
    
    def print_data(self):
        print(f"x={self.x},y={self.y},dist={self.dist}")

    def update_dist(self, dist):
        self.dist = dist

    

    
