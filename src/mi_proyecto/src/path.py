import os

# Ruta del archivo de script actual
script_path = os.path.abspath(__file__)

# Directorio del script actual
script_directory = os.path.dirname(script_path)

# Ruta completa del archivo de imagen relativa al directorio del script
image_path = os.path.join(script_directory, 'imagen.jpg')

# Imprimir la ruta relativa de la imagen
print(image_path)

