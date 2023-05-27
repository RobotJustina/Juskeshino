#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>

#define NUM_BLOCKS 600
#define THREADS_PER_BLOCK 512

__global__ void kernel_transform_cloud(float* xyz, double* tf, bool* is_free_space, unsigned char* bgr)
{
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    float xc = xyz[3*i];
    float yc = xyz[3*i+1];
    float zc = xyz[3*i+2];
    float x  = xc*tf[0] + yc*tf[4] + zc*tf[ 8] + tf[12];
    float y  = xc*tf[1] + yc*tf[5] + zc*tf[ 9] + tf[13];
    float z  = xc*tf[2] + yc*tf[6] + zc*tf[10] + tf[14];

    is_free_space[i] = z < 0.05;
    if(is_free_space[i])
    {
        bgr[3*i  ] = 0;
        bgr[3*i+1] = 0;
        bgr[3*i+2] = 0;
        xyz[3*i]   = x;
        xyz[3*i+1] = y;
        xyz[3*i+2] = z;
    }
    else
    {
        xyz[3*i]   = 0;
        xyz[3*i+1] = 0;
        xyz[3*i+2] = 0;
    }
    
}

__global__ void kernel_get_first_centroid(float* xyz, float* centroids)
{
    __shared__ float partial_sums_x[THREADS_PER_BLOCK];
    __shared__ float partial_sums_y[THREADS_PER_BLOCK];
    __shared__ float partial_sums_z[THREADS_PER_BLOCK];
    
    partial_sums_x[threadIdx.x] = 0;
    partial_sums_y[threadIdx.x] = 0;
    partial_sums_z[threadIdx.x] = 0;
    __syncthreads();
    
    for(size_t i=0; i< NUM_BLOCKS; i++)
    {
        partial_sums_x[threadIdx.x] += xyz[3*(i*NUM_BLOCKS + threadIdx.x)    ];
        partial_sums_y[threadIdx.x] += xyz[3*(i*NUM_BLOCKS + threadIdx.x) + 1];
        partial_sums_z[threadIdx.x] += xyz[3*(i*NUM_BLOCKS + threadIdx.x) + 2];
    }
    __syncthreads();
    return;

    for(unsigned int i=THREADS_PER_BLOCK/2; i>0; i>>=1)
        if(threadIdx.x < i)
        {
            partial_sums_x[threadIdx.x] += partial_sums_x[threadIdx.x + i];
            partial_sums_y[threadIdx.x] += partial_sums_y[threadIdx.x + i];
            partial_sums_z[threadIdx.x] += partial_sums_z[threadIdx.x + i];
            __syncthreads();
        }
    __syncthreads();
    if(threadIdx.x == 0)
    {
        centroids[0] = partial_sums_x[0];
        centroids[1] = partial_sums_y[0];
        centroids[2] = partial_sums_z[0];
    }
}

void vq_cuda_main(unsigned char* src_bgr, float* src_xyz, unsigned char* dst_bgr, float* dst_xyz, double* tf, float* centroids)
{
    int N_pixels = 480*640;
    unsigned char* device_bgr;
    float*  device_xyz;
    double* device_tf;
    float*  device_centroids;
    bool*   device_is_free_space;
    
    cudaMalloc(&device_bgr, N_pixels*3);
    cudaMalloc(&device_xyz, N_pixels*12);
    cudaMalloc(&device_tf , 16*sizeof(double));
    cudaMalloc(&device_centroids, 64*3*sizeof(float));
    cudaMalloc(&device_is_free_space, N_pixels*sizeof(bool));
    
    cudaMemcpy(device_bgr, src_bgr, N_pixels*3 , cudaMemcpyHostToDevice);
    cudaMemcpy(device_xyz, src_xyz, N_pixels*12, cudaMemcpyHostToDevice);
    cudaMemcpy(device_tf , tf     , 16*sizeof(double), cudaMemcpyHostToDevice);
    
    kernel_transform_cloud<<<NUM_BLOCKS, THREADS_PER_BLOCK>>>(device_xyz, device_tf, device_is_free_space, device_bgr);
    cudaDeviceSynchronize();
    kernel_get_first_centroid<<<1, THREADS_PER_BLOCK>>>(device_xyz, device_centroids);
    cudaDeviceSynchronize();
    
    cudaMemcpy(dst_bgr, device_bgr, N_pixels*3 , cudaMemcpyDeviceToHost);
    cudaMemcpy(dst_xyz, device_xyz, N_pixels*12, cudaMemcpyDeviceToHost);
    cudaMemcpy(centroids, device_centroids, 64*3*sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(device_bgr);
    cudaFree(device_xyz);
    cudaFree(device_tf);
    cudaFree(device_centroids);
    cudaFree(device_is_free_space);
    return;
}
