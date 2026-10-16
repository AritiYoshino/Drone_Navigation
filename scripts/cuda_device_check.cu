#include <cuda_runtime.h>
#include <iostream>
__global__ void verify(int* result) { *result = 42; }
int main() {
    int count = 0;
    cudaError_t error = cudaGetDeviceCount(&count);
    if (error != cudaSuccess || count < 1) {
        std::cerr << cudaGetErrorString(error) << ", devices=" << count << '\n'; return 1;
    }
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    int* device = nullptr;
    if (cudaMalloc(&device, sizeof(int)) != cudaSuccess) return 1;
    verify<<<1, 1>>>(device);
    int value = 0;
    error = cudaMemcpy(&value, device, sizeof(int), cudaMemcpyDeviceToHost);
    cudaFree(device);
    if (error != cudaSuccess || value != 42) {
        std::cerr << cudaGetErrorString(error) << ", value=" << value << '\n'; return 1;
    }
    std::cout << prop.name << " sm_" << prop.major << prop.minor << "; CUDA kernel PASS\n";
    return 0;
}
