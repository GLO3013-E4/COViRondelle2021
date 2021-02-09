from main.src.handlers.handler import Handler


# TODO : This is a sample handler, it should be removed when real handlers will be implemented
class MapLettersHandler(Handler):
    def __init__(self, letter_mapper):
        self.letter_mapper = letter_mapper

    def handle(self, handled_data=None):
        return self.letter_mapper.map_letters_from_image(handled_data)
