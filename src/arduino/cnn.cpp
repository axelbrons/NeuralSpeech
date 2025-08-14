#include "cnn.h"

void maxPooling(const float pool_in[DATA_ROW][DATA_COL], float pool_out[DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE] )
{
  for(int i = 0; i < DATA_ROW/POOL_SIZE; i++) {
    for(int j = 0; j < DATA_ROW/POOL_SIZE; j++) {
      float maxVal = -INFINITY;
      for(int k = 0; k < POOL_SIZE; k++) {
        for(int l = 0; l < POOL_SIZE; l++) {
          maxVal = max(maxVal, pool_in[i*POOL_SIZE + k][j*POOL_SIZE + l]);
        }
      }
      pool_out[i][j] = maxVal;
    }
  }
}

void Convolution2D(const float input[DATA_ROW][DATA_COL], const float kernel[KERNEL_SIZE][KERNEL_SIZE], float output[DATA_ROW][DATA_COL])
{
    float paddedInput[DATA_ROW+KERNEL_SIZE-1][DATA_COL+KERNEL_SIZE-1];
    for(int i = 0; i < DATA_ROW+KERNEL_SIZE-1; i++) {
        for(int j = 0; j < DATA_COL+KERNEL_SIZE-1; j++) {
            if(i < KERNEL_SIZE/2 || i >= DATA_ROW+KERNEL_SIZE/2 || j < KERNEL_SIZE/2 || j >= DATA_COL+KERNEL_SIZE/2) {
                paddedInput[i][j] = 0;
            } else {
                paddedInput[i][j] = input[i-KERNEL_SIZE/2][j-KERNEL_SIZE/2];
            }
        }
    }

    for(int i = 0; i < DATA_ROW; i++) {
        for(int j = 0; j < DATA_COL; j++) {
            float sum = 0;
            for(int k = 0; k < KERNEL_SIZE; k++) {
                for(int l = 0; l < KERNEL_SIZE; l++) {
                    sum += paddedInput[i+k][j+l] * kernel[k][l];
                }
            }
            output[i][j] = sum;
        }
    }
}

void maxPooling_2(const float pool_in[(DATA_ROW/POOL_SIZE)][(DATA_COL/POOL_SIZE)], float pool_out[(DATA_ROW/POOL_SIZE)/POOL_SIZE][(DATA_COL/POOL_SIZE)/POOL_SIZE] )
{
  for(int i = 0; i < (DATA_ROW/POOL_SIZE)/POOL_SIZE; i++) {
    for(int j = 0; j < (DATA_COL/POOL_SIZE)/POOL_SIZE; j++) {
      float maxVal = -INFINITY;
      for(int k = 0; k < POOL_SIZE; k++) {
        for(int l = 0; l < POOL_SIZE; l++) {
          maxVal = max(maxVal, pool_in[i*POOL_SIZE + k][j*POOL_SIZE + l]);
        }
      }
      pool_out[i][j] = maxVal;
    }
  }
}

void Convolution2D_2(const float input[((DATA_ROW/POOL_SIZE)/POOL_SIZE)][(DATA_COL/POOL_SIZE)], const float kernel[KERNEL_SIZE][KERNEL_SIZE], float output[(DATA_ROW/POOL_SIZE)][(DATA_COL/POOL_SIZE)])
{
    float paddedInput[(DATA_ROW/POOL_SIZE)+KERNEL_SIZE-1][(DATA_COL/POOL_SIZE)+KERNEL_SIZE-1];
    for(int i = 0; i < (DATA_ROW/POOL_SIZE)+KERNEL_SIZE-1; i++) {
        for(int j = 0; j < (DATA_COL/POOL_SIZE)+KERNEL_SIZE-1; j++) {
            if(i < KERNEL_SIZE/2 || i >= (DATA_ROW/POOL_SIZE)+KERNEL_SIZE/2 || j < KERNEL_SIZE/2 || j >= (DATA_COL/POOL_SIZE)+KERNEL_SIZE/2) {
                paddedInput[i][j] = 0;
            } else {
                paddedInput[i][j] = input[i-KERNEL_SIZE/2][j-KERNEL_SIZE/2];
            }
        }
    }

    for(int i = 0; i < (DATA_ROW/POOL_SIZE); i++) {
        for(int j = 0; j < (DATA_COL/POOL_SIZE); j++) {
            float sum = 0;
            for(int k = 0; k < KERNEL_SIZE; k++) {
                for(int l = 0; l < KERNEL_SIZE; l++) {
                    sum += paddedInput[i+k][j+l] * kernel[k][l];
                }
            }
            output[i][j] = sum;
        }
    }
}




void flatten2vector(float input[DATA_ROW/(POOL_SIZE*LAYER)][DATA_COL/(POOL_SIZE*LAYER)], float output[(DATA_ROW/(POOL_SIZE*LAYER)) * (DATA_COL/(POOL_SIZE*LAYER))]) 
{
  int idx = 0;
  for(int i = 0; i < DATA_ROW/(POOL_SIZE*LAYER); i++) {
    for(int j = 0; j < DATA_COL/(POOL_SIZE*LAYER); j++) {
      
      output[idx] = input[i][j];
      idx++;
      
    }
  }
}

void cnn(const float input[DATA_ROW][DATA_COL], float output[(DATA_ROW/(POOL_SIZE*LAYER)) * (DATA_COL/(POOL_SIZE*LAYER))])
{
  
    float conv1[DATA_ROW][DATA_COL];
    //Convolution2D(input, kernel1, conv1);
    float pool1[DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE];
    //maxPooling(conv1, pool1);
    /*
    float conv2[(DATA_ROW/POOL_SIZE)][(DATA_COL/POOL_SIZE)];
    Convolution2D_2(pool1, kernel1, conv2);
    float pool2[(DATA_ROW/POOL_SIZE)/POOL_SIZE][(DATA_COL/POOL_SIZE)/POOL_SIZE];
    maxPooling_2(conv2, pool2);*/
    
    //flatten2vector(input, output);
    //flatten2vector(pool1, output);
    //testPrint(output);
    //Serial.println("cnn");
  
}
/*
void cnn_multi(const float input[DATA_ROW][DATA_COL], float output[NUM_FILTERS * (DATA_ROW/(POOL_SIZE*2)) * (DATA_COL/(POOL_SIZE*2))]) {
  // Première convolution + pooling
  //Serial.println("boucle");
  float conv1[NUM_FILTERS][DATA_ROW][DATA_COL];
  //Serial.println("conv1");
  for (int f = 0; f < NUM_FILTERS; f++) {
    Convolution2D(input, kernels1[f], conv1[f]);
  }

  float pool1[NUM_FILTERS][DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE];
  //Serial.println("pool1");
  maxPooling_Multi(conv1, pool1);

  //Serial.println("flatten");
  // Flatten
  flatten2vector_multi(pool1, output);
  //Serial.println("cnn");
}*/

void cnn_multi(const float input[DATA_ROW][DATA_COL], float output[NUM_FILTERS * (DATA_ROW/(POOL_SIZE*2)) * (DATA_COL/(POOL_SIZE*2))]) {
  // 1. Normalisation des entrées
  float normalized_input[DATA_ROW][DATA_COL];
  for(int i=0; i<DATA_ROW; i++) {
    for(int j=0; j<DATA_COL; j++) {
      normalized_input[i][j] = (input[i][j] - TRAIN_MEAN) / TRAIN_STD;
    }
  }

  // 2. Première couche convolutive
  float conv1[NUM_FILTERS][DATA_ROW][DATA_COL] = {0}; // Initialisation à zéro
  
  for (int f = 0; f < NUM_FILTERS; f++) {
    Convolution2D(normalized_input, kernels1[f], conv1[f]);
    
    // Application de ReLU
    for(int i=0; i<DATA_ROW; i++) {
      for(int j=0; j<DATA_COL; j++) {
        conv1[f][i][j] = max(0.0, conv1[f][i][j]); // ReLU
      }
    }
  }

  // 3. Pooling
  float pool1[NUM_FILTERS][DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE];
  maxPooling_Multi(conv1, pool1);

  // 4. Flatten
  flatten2vector_multi(pool1, output);
}



void testPrint(float output[(DATA_ROW/(POOL_SIZE*LAYER)) * (DATA_COL/(POOL_SIZE*LAYER))])
{
    for(int i = 0; i < (DATA_ROW/(POOL_SIZE*LAYER)) * (DATA_COL/(POOL_SIZE*LAYER)); i++) {
        Serial.print(output[i]);
        Serial.print(" ");
    }
    Serial.println();
}

void Convolution2D_Multi(const float input[DATA_ROW][DATA_COL], const float kernels[NUM_FILTERS][KERNEL_SIZE][KERNEL_SIZE], float output[NUM_FILTERS][DATA_ROW][DATA_COL]) {
  for (int f = 0; f < NUM_FILTERS; f++) {
    Convolution2D(input, kernels[f], output[f]);
  }
}

void maxPooling_Multi(const float pool_in[NUM_FILTERS][DATA_ROW][DATA_COL], float pool_out[NUM_FILTERS][DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE]) {
  for (int f = 0; f < NUM_FILTERS; f++) {
    maxPooling(pool_in[f], pool_out[f]);
  }
}
/*
void Convolution2D_Multi_2(const float input[NUM_FILTERS][DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE], const float kernels[NUM_FILTERS][KERNEL_SIZE][KERNEL_SIZE], float output[NUM_FILTERS][DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE]) {
  for (int f = 0; f < NUM_FILTERS; f++) {
    Convolution2D(input[f], kernels[f], output[f]);
  }
}

void maxPooling_Multi_2(const float pool_in[NUM_FILTERS][DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE], float pool_out[NUM_FILTERS][DATA_ROW/(POOL_SIZE*2)][DATA_COL/(POOL_SIZE*2)]) {
  for (int f = 0; f < NUM_FILTERS; f++) {
    maxPooling(pool_in[f], pool_out[f]);
  }
}
*/
void flatten2vector_multi(float input[NUM_FILTERS][DATA_ROW/POOL_SIZE][DATA_COL/POOL_SIZE], float output[NUM_FILTERS * (DATA_ROW/POOL_SIZE) * (DATA_COL/POOL_SIZE)]) {
  int idx = 0;
  for (int f = 0; f < NUM_FILTERS; f++) {
    for (int i = 0; i < DATA_ROW/POOL_SIZE; i++) {
      for (int j = 0; j < DATA_COL/POOL_SIZE; j++) {
        output[idx++] = input[f][i][j];
      }
    }
  }
}
