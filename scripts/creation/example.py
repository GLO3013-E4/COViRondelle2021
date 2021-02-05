from Pathfinder import Pathfinder

from PIL import Image, ImageDraw


if __name__ == '__main__':
    obstacles = [
        (175, 27),
        (175, 75),
        (175, 123),
        (175, 170),
        (175, 222),
        (225, 228)
    ]

    start = (21, 24)
    end = (27, 382)

    pucks = [
        (175, 27),
        (175, 75),
        (175, 123),
        (175, 170),
        (175, 222),
        (225, 228)
    ]

    image = Image.open("../../../../Desktop/photo_camera_monde/WIN_20210126_11_29_53_Pro.jpg")

    pathfinder = Pathfinder(start, end, obstacles, pucks, image)
    pathfinder.find_square_matrix_path()
    pathfinder.show()