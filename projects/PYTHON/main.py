from PIL import Image

IMG_WIDTH = 160
IMG_HEIGHT = 128
IMG_SIZE = IMG_WIDTH * IMG_HEIGHT * 3

def gen_images():
    """Generate PNG/JPG images from the binary file, that contains RGB data"""

    # open the source data
    with open("C:\\Users\\marek\\Desktop\\NEXT_GEN\\NXP_add\\NXP_NEW0\\FOTKA_FLASH.bin", "rb") as f:
        raw_data = f.read()

    # index to currently processed
    img_index = 0

    # for all images in the source data
    while len(raw_data) >= IMG_SIZE:
        if(img_index == 15):
            print(raw_data[0:3])
            print(raw_data[3:6])
            print(raw_data[6:9])
        # create one image
        img = Image.frombytes("RGB", (IMG_WIDTH, IMG_HEIGHT), raw_data[0:IMG_SIZE])
        # save to the disk
        img_filename = f"image{img_index}.png"
        img.save(img_filename, quality="maximum")
        print(f"Saved {img_filename}")

        # select next image
        raw_data = raw_data[IMG_SIZE:]
        img_index += 1

if __name__ == '__main__':
    gen_images()
