# Example implementation in gps_service.py
class GPSService:
    _instance = None
    
    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = GPSService()
        return cls._instance
    
    def __init__(self):
        if GPSService._instance is not None:
            raise RuntimeError("Use GPSService.get_instance() instead")
        self._gps = GPSMonitor()
        
    def get_position_and_heading(self):
        return self._gps.get_position_and_heading()