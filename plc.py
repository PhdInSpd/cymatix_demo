import time

class RTrigger:
    def __init__(self) -> None:
        self.Q = False
        self._PreviousInput = False
    def CLK(self,input: bool) -> bool:
        if  ( not input ):
            self._PreviousInput = input
            self.Q=False
            return self.Q
        if (not self._PreviousInput):
            self._PreviousInput = input
            self.Q=True
            return self.Q
        self.Q=False
        return self.Q
        
class Ton:
    def __init__(self, SecTimeout:float) -> None:
        self.SecTimeout = SecTimeout
        self.Q = False
        self._SecStart = -1
    def CLK(self,active: bool) -> bool:
        if  ( not active ):
            self._SecStart = -1
            self.Q=False
            return self.Q
        if (self._SecStart == -1):
            self._SecStart = time.perf_counter()
        self.Q = (time.perf_counter() -self._SecStart) >= self.SecTimeout
        return self.Q