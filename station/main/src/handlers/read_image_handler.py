from station.main.src.handlers.handler import Handler


# TODO : This is a sample handler, it should be removed when real handlers will be implemented
class ReadImageHandler(Handler):
    def __init__(self, image_reader, image_path):
        self.image_reader = image_reader
        self.image_path = image_path

    def handle(self, handled_data=None):
        return self.image_reader.read_image_from_path(self.image_path)
