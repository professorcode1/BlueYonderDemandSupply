#include <curand_kernel.h>

extern "C"{
    __global__ void initkernel(curandState_t* rndm_nmbr_gnrtr){
        int tidx = threadIdx.x + blockIdx.x * blockDim.x;
        curandState_t* s = new curandState_t;
        curand_init( %(SEED)s, tidx, 0, s);
        rndm_nmbr_gnrtr[tidx] = *s;
    }

}