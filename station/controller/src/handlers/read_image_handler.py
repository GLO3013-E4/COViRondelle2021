from controller.src.handlers.handler import Handler


# TODO : Remove ReadImageHandler
class ReadImageHandler(Handler):
    def __init__(self, image_reader, image_path):
        self.image_reader = image_reader
        self.image_path = image_path

    def handle(self, handled_data=None):
        return self.image_reader.read_image_from_path(self.image_path)
