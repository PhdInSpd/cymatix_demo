from plc import Ton


class SteadyCount:
    def  __init__(self, steadySec):
        self.steadySec: float = steadySec
        self._lastCount: int = 0
        self._tonSteady = Ton(self.steadySec)

    def CLK( self, count: int)->bool:
        sameCount: bool = count == self._lastCount
        self._lastCount = count
        self._tonSteady.SecTimeout = self.steadySec
        return self._tonSteady.CLK(sameCount)
    def Reset(self):
        self._tonSteady.CLK(False)