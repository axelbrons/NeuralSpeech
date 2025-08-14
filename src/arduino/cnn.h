#include <Arduino.h>
#define POOL_SIZE 2 // max-pooling size
#define DATA_ROW 48
#define DATA_COL 13
#define DATA_SIZE 624
#define KERNEL_SIZE 3
#define LAYER 1
#define NUM_FILTERS 2

#define TRAIN_MEAN 2.826826947731756
#define TRAIN_STD 0.4871766034330845

const float kernels1[NUM_FILTERS][KERNEL_SIZE][KERNEL_SIZE] = {
  {
    { -0.33250648, 0.78796738, 0.99798071 },
    { 0.12851319, 0.06539030, 0.91324061 },
    { -0.66332191, 0.45526010, 0.35313651 }
  },
  {
    { -0.53418630, -0.17267044, -0.49413496 },
    { -0.02854088, -0.78559637, 0.62152886 },
    { -0.34201038, 0.28001729, -0.09649735 }
  }
};

const float kernel1[KERNEL_SIZE][KERNEL_SIZE] = {{ 0.08743796, 0.15135571, 0.18066438 },
    { 0.01900421, 0.98057938, -0.77661097 },
    { 1.17499030, 0.67993224, 0.11731741 }};

void maxPooling(const float poolinput[DATA_ROW][DATA_COL], float pool[DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE] );
void maxPooling_Multi(const float pool_in[NUM_FILTERS][DATA_ROW][DATA_COL], float pool_out[NUM_FILTERS][DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE]);

void Convolution2D(const float input[DATA_ROW][DATA_COL], const float kernel[KERNEL_SIZE][KERNEL_SIZE], float output[DATA_ROW][DATA_COL]);
void Convolution2D_Multi(const float input[DATA_ROW][DATA_COL], const float kernels[NUM_FILTERS][KERNEL_SIZE][KERNEL_SIZE], float output[NUM_FILTERS][DATA_ROW][DATA_COL]);

void Convolution2D_2(const float input[DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE], const float kernel[KERNEL_SIZE][KERNEL_SIZE], float output[DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE]);
void Convolution2D_Multi_2(const float input[NUM_FILTERS][DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE], const float kernels[NUM_FILTERS][KERNEL_SIZE][KERNEL_SIZE], float output[NUM_FILTERS][DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE]);

void maxPooling_2(const float input[DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE], const float kernel[KERNEL_SIZE][KERNEL_SIZE], float pool[DATA_ROW/(POOL_SIZE*LAYER)][DATA_COL/(POOL_SIZE*LAYER)] );
void maxPooling_Multi_2(const float pool_in[NUM_FILTERS][DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE], float pool_out[NUM_FILTERS][DATA_ROW/(POOL_SIZE*2)][DATA_COL/(POOL_SIZE*2)]);

void flatten2vector(float input[DATA_ROW/(POOL_SIZE*LAYER)][DATA_COL/(POOL_SIZE*LAYER)], float output[(DATA_ROW/(POOL_SIZE*LAYER)) * (DATA_COL/(POOL_SIZE*LAYER))]);
void flatten2vector_multi(float input[NUM_FILTERS][DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE], float output[NUM_FILTERS * (DATA_ROW/POOL_SIZE) * (DATA_COL/POOL_SIZE)]);

void cnn(const float input[DATA_ROW][DATA_COL], float output[(DATA_ROW/(POOL_SIZE*LAYER)) * (DATA_COL/(POOL_SIZE*LAYER))]);
void cnn_multi(const float input[DATA_ROW][DATA_COL], float output[NUM_FILTERS * (DATA_ROW/(POOL_SIZE*2)) * (DATA_COL/(POOL_SIZE*2))]);

void testPrint(float output[(DATA_ROW/(POOL_SIZE*LAYER)) * (DATA_COL/(POOL_SIZE*LAYER))]);
