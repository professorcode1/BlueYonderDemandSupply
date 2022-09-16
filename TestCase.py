import numpy as np
import opensimplex
import random
from time import sleep
import scipy.linalg as sla
import numba 

__all__ = ["TestCase"]

@numba.jit(nogil = True)  
def normal_random_distribution(data_array, X_array, expd_vl):
    # print("here")
    EPSILON = 0.001
    data_len = data_array.size
    data_array[:] = 1 / data_len
    current_E = np.sum(data_array * X_array)
    middle = np.argmax(X_array > expd_vl) - 1
    while abs(current_E - expd_vl) > EPSILON:
        # print(abs(current_E - expd_vl) / expd_vl)
        # print(0, middle, expd_vl)
        left_i = random.randint( 0, middle )
        rght_i = random.randint( middle + 1, data_len - 1 )
    
        if current_E < expd_vl:
            up_lim = min(data_array[left_i] , ( expd_vl - current_E ) / ( X_array[rght_i] - X_array[left_i] ) )
            mu = up_lim / 2
            sigma = up_lim / 6
            x = np.clip(np.random.normal( mu, sigma, 1 ), 0, up_lim)[0]
            data_array[left_i] -= x
            data_array[rght_i] += x
            current_E += x * ( X_array[rght_i] - X_array[left_i] )
        else:
            up_lim = min(data_array[rght_i] , ( current_E - expd_vl ) / ( X_array[rght_i] - X_array[left_i] ) ) 
            mu = up_lim / 2
            sigma = up_lim / 6
            x = np.clip(np.random.normal( mu, sigma, 1 ), 0, up_lim)[0]
            data_array[left_i] += x
            data_array[rght_i] -= x
            current_E -= x * ( X_array[rght_i] - X_array[left_i] )


#     1)number of stores 
#     2)number of products :- an int representing the number of products
#     3timeframe :- an int representing the number of days we simulate
#     4)factory_capacity :- an int representing how many total products the factory can produce in a day
#     5)DC_capacity :- an int representing how many total products can be stored:
#     6)demand range min :- an int saying the least value of each products order
#     7)demand range max :- an int saying the max value of each products order
#     8)truck_capacity :- total products truck can carry
#     9)data = array of shape (stores, products, demand_range, timeframe) where data[w][x][y][z] = probability of
#         store w wanting product x in (y + demand_range_min) amount on day z 
#     10)truck_max_cap :- hypothetically, the number of trucks on a day can be min(max_supply, max_demand)
#         i.e. 1 truck per product from demand to supply 
#         max_supply = DC_capacity
#         max_demand = demand_range_max * number of stores * number of products
#         but simulating this wastes a lot of space, so we resonably cap the number of trucks to save space
#     11)DC_cap_to_noise_mean_factor: -I generate 4d simplex noise and multiply that with DC_Capacity to get 
#             data array, but if demand always tends towards max supply then because of inoptimality 
#         it will keep on cummilating, keeping this near 1 is like a stress test for the algorithm

@numba.njit(parallel= True, nogil = True)
def getDataInter(data, demand_range_min, demand_range_max):
    (number_of_stores, number_of_products, timeframe, _) = data.shape
    for store in numba.prange(number_of_stores):
        print(store* 100 / number_of_stores)
        for product in np.arange(number_of_products):
            for time in np.arange(timeframe):
                expd_vl = data[store, product, time, 0]
                normal_random_distribution(data[store, product, time], np.arange(demand_range_min, demand_range_max + 1), expd_vl)

class TestCase:
    def __init__(self, number_of_stores,number_of_products, timeframe = 90, factory_capacity = 200, 
                       DC_capacity = 50, demand_range_min = 10, demand_range_max = 400,
                      truck_capacity = 20, truck_max_cap = 0.1):
        self.number_of_stores = number_of_stores
        self.number_of_products = number_of_products
        self.timeframe = timeframe
        self.factory_capacity = factory_capacity
        self.DC_capacity = DC_capacity
        self.demand_range_min = demand_range_min
        self.demand_range_max = demand_range_max
        self.truck_capacity = truck_capacity
        self.truck_max_cap = truck_max_cap

        self.maximum_truck = truck_max_cap * min(DC_capacity, demand_range_max * number_of_stores*number_of_products )
    
    def getData(self, DC_cap_to_noise_mean_factor = 1, seed = None, simplex_reduction_factor = 0.3, suppress_error = False):
#         simplex_reduction_factor :- making this value low will reduce variance and all data will look the same
#                                    keeping it high will make the data look completely random . 0.3 is just the right value
        demand_range = self.demand_range_max - self.demand_range_min + 1
        data = np.zeros((self.number_of_stores, self.number_of_products, self.timeframe, demand_range), 
                        dtype=np.float32)
        if seed == None:
            opensimplex.random_seed()
        else:
            opensimplex.seed(seed)
        for store in np.arange(self.number_of_stores):
            for product in np.arange(self.number_of_products):
                for time in np.arange(self.timeframe):
                    data[store, product, time, 0] = opensimplex.noise3(store*simplex_reduction_factor, 
                                                                       product*simplex_reduction_factor, 
                                                                       time*simplex_reduction_factor)
        #normalising from -1,1 -> 0, 1
        data[:, :, :, 0] += 1
        data[:, :, :, 0] /= 2
        
        total = np.sum(data[:,:,:,0])
        cumm_dmnd_ = self.timeframe * self.DC_capacity * DC_cap_to_noise_mean_factor
        data[:, :, :, 0] *= cumm_dmnd_ / total 
        clipps_required = np.sum(np.logical_or(data[:,:,:,0] <= self.demand_range_min, data[:,:,:,0] > self.demand_range_max))
        total_size = self.number_of_stores * self.number_of_products * self.timeframe
        print(f"clipping {clipps_required} out of {total_size} values")
        if not suppress_error:
            assert clipps_required <  0.05 * total_size, "DC Capacity is way to low for this system to ever work, if you belive this is false (unlikely) decrease simplex_reduction_factor , or set suppress_error to true"
        data[:, :, :, 0]
        getDataInter(data, self.demand_range_min, self.demand_range_max)
        
        return data
if __name__ == "__main__":
    tc = TestCase(1,1, DC_capacity=50)
    hlpr = tc.getData(DC_cap_to_noise_mean_factor = 1, simplex_reduction_factor = 0.3)