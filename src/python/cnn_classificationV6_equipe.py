import tensorflow as tf
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense
import numpy as np
from tensorflow.keras import layers
from tensorflow.keras import Sequential
import matplotlib.pyplot as plt
import random

# Avec led erreur (blank) 55 erreurs

seed = 43884  # Changez cette valeur !
np.random.seed(seed)
tf.random.set_seed(seed)
random.seed(seed)

# Define if you want to use biases
IS_BIASED = True

# Enable 32-bit floating-point precision
tf.keras.backend.set_floatx('float32')

data_train = np.loadtxt("train_equipe4.txt")
data_test = np.loadtxt("test_equipe4.txt")

X_train = data_train.reshape((650,48,13))
X_test = data_test.reshape((65,48,13))

y_train = np.zeros((650,), dtype=np.float32)
y_train[50:100] = 1
y_train[100:150] = 2
y_train[150:200] = 0
y_train[200:250] = 1
y_train[250:300] = 2
y_train[300:350] = 0
y_train[350:400] = 1
y_train[400:450] = 2
y_train[450:500] = 0
y_train[500:550] = 1
y_train[550:600] = 2
y_train[600:650] = 3

y_test = np.zeros((65,), dtype=np.float32)
y_test[5:10] = 1
y_test[10:15] = 2
y_test[15:20] = 0
y_test[20:25] = 1
y_test[25:30] = 2
y_test[30:35] = 0
y_test[35:40] = 1
y_test[40:45] = 2
y_test[45:50] = 0
y_test[50:55] = 1
y_test[55:60] = 2
y_test[60:65] = 3

print("Train mean : ",X_train.mean())
print("Train std : ",X_train.std())

X_train = (X_train - X_train.mean()) / X_train.std()
X_test = (X_test - X_test.mean()) / X_test.std()

# Mélange synchronisé des données d'entraînement
indices = np.arange(len(X_train))
np.random.shuffle(indices)

X_train = X_train[indices]
y_train = y_train[indices]

#print("y_train mélangé :", y_train[:20])

# Define a simple CNN model
def create_model():
    initializer = tf.keras.initializers.HeNormal()
    model = tf.keras.Sequential([
        #tf.keras.layers.Conv2D(1, (3, 3), activation='relu', input_shape=(48, 13, 1), use_bias = IS_BIASED, kernel_initializer=initializer),  # First conv layer
        #tf.keras.layers.MaxPooling2D(pool_size=(2, 2)),  # Downsample the feature maps
        #tf.keras.layers.Conv2D(1, (3, 3), activation='relu', use_bias = IS_BIASED, kernel_initializer=initializer),  # Second conv layer
        #tf.keras.layers.MaxPooling2D(pool_size=(2, 2)),  # Downsample again
        tf.keras.layers.Flatten(),  # Flatten the output for the dense layer
        #tf.keras.layers.Dense(20, activation='relu', use_bias = IS_BIASED, kernel_initializer=initializer),  # Fully connected layer
        #tf.keras.layers.Dense(32, activation='relu', use_bias = IS_BIASED),  # Fully connected layer
        tf.keras.layers.Dense(20, activation='relu', use_bias=IS_BIASED),  # Fully connected layer
        tf.keras.layers.Dropout(0.5),  # 50 % des neurones désactivés pendant l’entraînement
        tf.keras.layers.Dense(4, activation='softmax', use_bias = IS_BIASED, kernel_initializer=initializer)  # Output layer (10 classes)
    ])
    return model

# Compile the model
model = create_model()
model.compile(optimizer=Adam(0.001), loss='sparse_categorical_crossentropy', metrics=['accuracy'])




# Compile the model
#optimizer = Adam(learning_rate=0.001)
#model.compile(optimizer=optimizer, loss='binary_crossentropy', metrics=['accuracy'])
model.summary()

# Train the model
history = model.fit(X_train, y_train, epochs=50, verbose=1,validation_data=(X_test, y_test))

# Affichage des courbes de loss et d'accuracy
plt.figure(figsize=(12, 5))

# Courbe de la loss
plt.subplot(1, 2, 1)
plt.plot(history.history['loss'], label='Train Loss')
plt.plot(history.history['val_loss'], label='Validation Loss')
plt.title('Loss au fil des epochs')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend()

# Courbe de la précision
plt.subplot(1, 2, 2)
plt.plot(history.history['accuracy'], label='Train Accuracy')
plt.plot(history.history['val_accuracy'], label='Validation Accuracy')
plt.title('Accuracy au fil des epochs')
plt.xlabel('Epoch')
plt.ylabel('Accuracy')
plt.legend()

plt.tight_layout()
plt.savefig("courbes_performance.png")

# Evaluate the model on the training data
loss, accuracy = model.evaluate(X_test, y_test)
print(f"Model accuracy: {accuracy * 100:.2f}%")

# Predict
predictions = model.predict(X_test)
print("Predictions:")
for i in range(len(X_test)):
    print()
    for j in range(4):
        print(f"Predicted Output: {predictions[i][j]:.7f}")

# Print the weight and biases:

weights_biases = model.get_weights()

print("#define _1_OPTIMIZE 0B01000000 // Highly-Recommended Optimization For RAM")
print("#define _2_OPTIMIZE 0B01000000 // NO_BIAS \n")
"""
print('float weights[] = {', end="")
for layer in model.layers:
    weights = layer.get_weights()
    if len(weights) > 0:
        w = weights[0]  # On ne prend que les poids (pas les biais s'ils existent)
        if len(w.shape) == 4:  # Cas Conv2D (kernel_h, kernel_w, input_channels, filters)
            print(f"\n// Layer {layer.name} (Conv2D) weights - {w.shape[3]} filters:")
            for filter_idx in range(w.shape[3]):  # Pour chaque filtre
                print(f"\n// Filter {filter_idx + 1}/{w.shape[3]}:")
                for channel in range(w.shape[2]):  # Pour chaque canal d'entrée (1 dans votre cas)
                    if w.shape[2] > 1:  # Uniquement si plusieurs canaux
                        print(f"// Channel {channel + 1}:")
                    for i in range(w.shape[0]):  # Hauteur du kernel (3)
                        print('    ', end='')
                        for j in range(w.shape[1]):  # Largeur du kernel (3)
                            print(f"{w[i,j,channel,filter_idx]:.8f}, ", end="")
                        print()  # Nouvelle ligne après chaque ligne du kernel
        elif len(w.shape) == 2:  # Cas Dense
            print(f"\n// Layer {layer.name} (Dense) weights:")
            for i in range(w.shape[0]):
                print('    ', end='')
                for j in range(w.shape[1]):
                    print(f"{w[i,j]:.8f}, ", end="")
                print()
print('};')
"""

def print_kernels_c_format(model):
    for layer_idx, layer in enumerate(model.layers):
        weights = layer.get_weights()
        if len(weights) > 0 and len(weights[0].shape) == 4:  # Conv2D layers only
            w = weights[0]
            num_filters = w.shape[3]
            kernel_size = w.shape[0]  # Assuming square kernels

            print(
                f"// Layer {layer_idx + 1}: {layer.name} ({num_filters} filters, {kernel_size}x{kernel_size} kernels)")
            print(f"float kernels{layer_idx + 1}[{num_filters}][{kernel_size}][{kernel_size}] = {{")

            for filter_idx in range(num_filters):
                print("  {")
                for i in range(kernel_size):
                    print("    { ", end="")
                    for j in range(kernel_size):
                        # Handle single or multiple input channels
                        if w.shape[2] == 1:
                            val = w[i, j, 0, filter_idx]
                        else:
                            # For multi-channel, we average across channels
                            val = np.mean(w[i, j, :, filter_idx])

                        print(f"{val:.8f}", end="")
                        if j < kernel_size - 1:
                            print(", ", end="")

                    print(" }", end="")
                    if i < kernel_size - 1:
                        print(",")
                    else:
                        print()

                print("  }", end="")
                if filter_idx < num_filters - 1:
                    print(",")
                else:
                    print()

            print("};\n")


def print_dense_weights_and_biases_c_format(model):
    for layer_idx, layer in enumerate(model.layers):
        weights = layer.get_weights()
        if len(weights) > 0 and len(weights[0].shape) == 2:  # Couches denses seulement
            # Affichage des poids
            w = weights[0]
            print(f"// Couche {layer_idx + 1}: {layer.name} (Dense)")
            print(f"// Dimensions: {w.shape[1]} entrées x {w.shape[0]} sorties")
            print(f"const float dense_weights_{layer_idx + 1}[{w.shape[0]}][{w.shape[1]}] PROGMEM = {{")

            for output_neuron in range(w.shape[1]):
                print("  {", end="")
                for input_neuron in range(w.shape[0]):
                    print(f"{w[input_neuron, output_neuron]:.8f}", end="")
                    if input_neuron < w.shape[0] - 1:
                        print(", ", end="")
                print("}", end="")
                if output_neuron < w.shape[1] - 1:
                    print(",")
                else:
                    print()
            print("};")

            # Affichage des biais s'ils existent
            if len(weights) > 1:  # Si des biais sont présents
                biases = weights[1]
                print(f"\nconst float dense_biases_{layer_idx + 1}[{len(biases)}] PROGMEM = {{")
                for i, bias in enumerate(biases):
                    print(f"  {bias:.8f}", end="")
                    if i < len(biases) - 1:
                        print(",")
                print("\n};\n")
            else:
                print("// Pas de biais pour cette couche\n")


# Exemple d'utilisation :
print_dense_weights_and_biases_c_format(model)
# Usage example:
print_kernels_c_format(model)