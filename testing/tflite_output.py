import numpy as np
from tflite_runtime.interpreter import Interpreter

# Load model
interpreter = Interpreter(model_path="nav_dynamic.tflite")
interpreter.allocate_tensors()

# Get input/output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

print("Input details:", input_details)
print("Output details:", output_details)