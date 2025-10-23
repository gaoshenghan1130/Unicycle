import struct

class MotorData:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, "_initialized"):
            self.pg = 0.0
            self.pdg = 0.0
            self.mp = 0.0
            self.mdp = 0.0
            self.raw_bytes = bytearray(128)
            self.values = []
            self._initialized = True
            # answer data
            self.mt = 0 
            self.pt = 0
            self.response_bytes = bytearray(128)  # to send back

    def update_from_bytes(self, data: bytearray):
        self.raw_bytes = data
        first32 = data[:32]
        self.values = list(struct.unpack('<4d', first32))
        self.pg, self.pdg, self.mp, self.mdp = self.values
        print("Decoded first 4 doubles:", self.values)
        
    def controlFunction(self):
        # Placeholder for control logic, get mt,pt
        pass

    def write_to_bytes(self):
        # Prepare bytes to send back
        self.controlFunction()
        self.response = struct.pack('<2d', self.mt, self.pt) # pack mt and pt as doubles
        self.response_bytes[:len(self.response)] = self.response

    def update(self):
        self.controlFunction()
        self.write_to_bytes()
        pass
    
    