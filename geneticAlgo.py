import numpy as np
import numba 
import pycuda.driver as cudaDrvr
# import pycuda.autoinit
from pycuda.compiler import SourceModule

__all__ = ["GeneticAlgorithm"]

CURANDSTATE_SIZE_BYTES = 48

class GeneticAlgorithm:
    def __init__(self, testCase_data, population_size,  mutationPopulationFraction = 0.25, elitePopulationFraction = 0.1, truck_empty_probability = 0.3, seed = None, divice_id = 0):
        self.device = cudaDrvr.Device(divice_id)
        MAX_THREADS_PER_MULTIPROCESSOR = self.device.get_attributes()[cudaDrvr.device_attribute.MAX_THREADS_PER_MULTIPROCESSOR]
        MAX_THREADS_PER_BLOCK = self.device.get_attributes()[cudaDrvr.device_attribute.MAX_THREADS_PER_BLOCK]
        MULTIPROCESSOR_COUNT = self.device.get_attributes()[cudaDrvr.device_attribute.MULTIPROCESSOR_COUNT]
        self.numberOfActiveBlocks = MULTIPROCESSOR_COUNT *( MAX_THREADS_PER_MULTIPROCESSOR / MAX_THREADS_PER_BLOCK )
        self.threadsPerBlock = MAX_THREADS_PER_BLOCK
        self.totalActiveThreads = self.numberOfActiveBlocks * self.threadsPerBlock

    
    def __call__(self):
        pass
