/***
Assumptopn
    Invoked via (number of active grids,1,1) (bx,by,xz)
    where bx * by * xz = number of active threads 
***/
#define tIdx threadIdx.x
#define tIdy threadIdx.y
#define tIdz threadIdx.z

#define bDmx blockDim.x
#define bDmy blockDim.y
#define bDmz blockDim.z

#define bIdx blockIdx.x
#define bIdy blockIdx.y
#define bIdz blockIdx.z

#define gDmx gridDim.x
#define gDmy gridDim.y
#define gDmz gridDim.z

#define threadsPerBlock bDmx*bDmy*bDmz

#define WARP_SIZE 32

#include <curand_kernel.h>


__device__ __forceinline__ int row_major_4D(int w, int x, int y, int z, int W, int X,int Y, int Z){
    return z + Z * ( y + Y * ( x + X * w ) );
}

__device__ __forceinline__ int col_major_4D(int w, int x, int y, int z, int W, int X,int Y, int Z){
    return w + W *( x + X * (y + Y * z) );
}

__device__ __forceinline__ int col_major_3D(int x, int y, int z, int X,int Y, int Z){
    return x + X * (y + Y * z) ;
}

extern "C"{
__global__ void create_initial_population(int *population,const int population_size,const int time_frame,const int number_of_trucks,const int truck_capacity, 
    const int number_of_stores,const int number_of_products,const float probability_of_no_truck, curandState_t* rndm_nmbr_gnrtr){
        __shared__ int population_iter;
        __shared__ int time_iter;
        __shared__ int trucks_iter;
        __shared__ int population_per_block;
        __shared__ int block_solution_padding;
        
        int block_thread_id = col_major_3D(tIdx, tIdy, tIdz, bDmx, bDmy, bDmz);
        int global_thread_id = threadsPerBlock * bIdx + block_thread_id;
        
        if(block_thread_id == 0){
            population_per_block = population_size / gDmx + (population_size % gDmx != 0);
            block_solution_padding = population_per_block * bIdx;
            population_iter = 0;
        }
        __threadfence_block();
        
        while( population_iter * bDmx < population_per_block ){
            
            int thread_sol_index = block_solution_padding + population_iter * bIdx + tIdx;
       
            if(block_thread_id == 0){
                time_iter = 0;
            }
            __threadfence_block();

            while( time_iter * bDmy < time_frame ){
            
                int thread_time_index = time_iter * bIdy + tIdy;
                
                if(block_thread_id == 0){
                    trucks_iter = 0;
                }
                __threadfence_block();

                while( trucks_iter * bDmz < number_of_trucks ){
                    
                    int thread_truck_index = trucks_iter * bDmz + tIdz;

                    float rndm_nmbr_1, rndm_nmbr_2;
                    rndm_nmbr_1 = curand_uniform( rndm_nmbr_gnrtr + global_thread_id ) ;
                    rndm_nmbr_2 = curand_uniform( rndm_nmbr_gnrtr + global_thread_id ) ;
                    
                    int store = ceil(rndm_nmbr_1 * number_of_stores)  - 1; //curand_uniform includes 1 and not 0.
                    store = rndm_nmbr_2 <= probability_of_no_truck ? -1 : store;
                    int index = row_major_4D(thread_sol_index, thread_time_index, thread_truck_index, 0, population_size, time_frame,  number_of_trucks, truck_capacity);
                    population[ index ] = store; 

                    for(int thread_capacity_index = 1 ; thread_capacity_index <= truck_capacity ; thread_capacity_index++){
                        population[ index + thread_capacity_index ] = ceil(curand_uniform( rndm_nmbr_gnrtr + global_thread_id ) * number_of_products) - 1;
                    }
                    
                    if(block_thread_id == 0){
                        trucks_iter++;
                    }
                    __syncthreads();
                }                

                if(block_thread_id == 0){
                    time_iter++;
                }
                __syncthreads();
            
            }

            if( block_thread_id == 0 ){
                population_iter++;
            }
            __syncthreads();
        
        }
    }
}