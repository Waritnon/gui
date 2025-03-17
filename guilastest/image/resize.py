from PIL import Image
import os

# Input and output directories
input_folder = "ani01"
output_folder = "new"
new_size = (1024, 600)  # Change this to your desired size (width, height)

# Ensure the output folder exists
os.makedirs(output_folder, exist_ok=True)

# Process each image in the folder
for filename in os.listdir(input_folder):
    if filename.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".gif")):
        img_path = os.path.join(input_folder, filename)
        img = Image.open(img_path)
        img_resized = img.resize(new_size)
        
        # Save resized image
        output_path = os.path.join(output_folder, filename)
        img_resized.save(output_path)
        print(f"Resized and saved: {output_path}")

print("All images resized successfully!")
