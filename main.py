import numpy as np
import opensimplex
import random
from time import sleep
import scipy.linalg as sla
import sys

EPSILON = sys.float_info.epsilon
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
    
    def getData(self, DC_cap_to_noise_mean_factor = 1, seed = None, simplex_reduction_factor = 0.3,
                expected_value_to_distribution = "normal random distribution" ):
#         simplex_reduction_factor :- making this value low will reduce variance and all data will look the same
#                                    keeping it high will make the data look completely random 
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
        def normal_random_distribution(data_array, X_array, expd_vl):
            def recurse(left_i, rght_i, E, P):
                if left_i + 3 <= rght_i:
                    middle = left_i + np.argmax( X_array[ left_i : rght_i ] > E ) - 1
                    if middle + 2 == rght_i:
                        return recurse(middle, rght_i, E, P)
                    if left_i == middle:
                        return recurse(left_i, left_i +2, E, P)
                    p_l = random.uniform(0, P)
                    p_r = P - p_l
                    x_l_upper =  X_array[ middle ] * p_l
                    x_l_lower =  X_array[ left_i ] * p_l
                    x_l = random.uniform(x_l_lower, x_l_upper)
                    x_r = (E - p_l * x_l) / p_r
                    print("Probabilitys \t",p_l, p_r, P)
                    print("Left X       \t",x_l_lower, x_l_upper, x_l)
                    print("Right X      \t",x_r)
                    print("X            \t",x_l,  E, x_r)
                    print("Indexse      \t",left_i, middle, rght_i)
                    print()
                    sleep(5)
                    recurse(left_i, middle +1, x_l, p_l)
                    recurse(middle +1, rght_i, x_r, p_r)
                elif left_i + 2 == rght_i:
                    x_l = X_array[ left_i ]
                    x_r = X_array[ rght_i ]
                    data_array[ left_i ] = (E - P * x_r) / (x_l - x_r)
                    data_array[ rght_i ] = (E - P * x_l) / (x_r - x_l)
                elif left_i + 1 == rght_i:
                    assert False, "Algorithm is broken, this should not have happened"
                else:
                    return 
            data_len = data_array.size
            data_array[:] = 0
            X_array = np.sort(X_array)
            assert X_array[0] <=  expd_vl and expd_vl < X_array[-1], "Not possible to generate distribution"
            recurse(0, data_len, expd_vl, 1)
            print(data_array)
                
                
                
        if not callable(expected_value_to_distribution):
            if expected_value_to_distribution == "normal random distribution":
                expected_value_to_distribution = normal_random_distribution
        
        for store in np.arange(self.number_of_stores):
            for product in np.arange(self.number_of_products):
                for time in np.arange(self.timeframe):
                    expd_vl = data[store, product, time, 0]
                    expected_value_to_distribution(data[store, product, time], 
                                                   np.arange(self.demand_range_min, self.demand_range_max + 1),
                                                   expd_vl)
                    return data
#                     print(np.sum(data[store, product, time]))
        return data
tc = TestCase(2,2, DC_capacity=200)
hlpr = tc.getData(DC_cap_to_noise_mean_factor = 1, simplex_reduction_factor = 0.3)
