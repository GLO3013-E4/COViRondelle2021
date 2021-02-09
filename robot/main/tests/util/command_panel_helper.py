from main.src.readers.image_reader import ImageReader

command_panel_images_path = [
    'src/data/images/command_panel_example_1.png',
    'src/data/images/command_panel_example_2.png'
]

command_panel_images = []
image_reader = ImageReader()

for path in command_panel_images_path:
    command_panel_images.append(image_reader.read_image_from_path(path))

command_panel_images_letters = [
    ['A', 'B', 'D', 'C', 'B', 'C', 'A', 'D', 'A'],
    ['C', 'B', 'A', 'B', 'A', 'C', 'D', 'A', 'D']
]
