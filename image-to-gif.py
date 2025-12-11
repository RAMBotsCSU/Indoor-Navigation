from PIL import Image
import os
import re

def make_gif(image_folder, output_path, duration=80):
    """
    Convert sequentially numbered images into a GIF.
    
    image_folder: path to folder containing images
    output_path: output GIF file (e.g. "output.gif")
    duration: time per frame in ms (default: 80ms)
    """
    # Get list of image filenames
    files = os.listdir(image_folder)

    # Only keep image files
    images = [f for f in files if f.lower().endswith(('.png', '.jpg', '.jpeg'))]

    # Sort by numeric sequence if filenames contain numbers
    def extract_number(s):
        match = re.search(r'\d+', s)
        return int(match.group()) if match else -1

    images.sort(key=extract_number)

    # Load images
    frames = [Image.open(os.path.join(image_folder, img)) for img in images]

    # Save as GIF
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=duration,
        loop=0
    )

    print(f"GIF saved to {output_path}")

if __name__ == "__main__":
    make_gif("outputs", "output.gif")  # change "frames" to your folder
