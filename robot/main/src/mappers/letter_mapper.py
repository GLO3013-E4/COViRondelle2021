import pytesseract

CUSTOM_CONFIG = r'--oem 3 --psm 11 -c tessedit_char_whitelist=ABCD'
CORRECT_AMOUNT_OF_LETTERS = 9


class LetterMapper:
    @staticmethod
    def map_letters_from_image(image):
        found_letters = pytesseract.image_to_string(image, config=CUSTOM_CONFIG)

        lookup_letters = ['A', 'B', 'C', 'D']
        mapped_letters = list(filter(lambda letter: letter in lookup_letters, found_letters))

        # TODO : Test incorrect amount of letters
        if len(mapped_letters) is not CORRECT_AMOUNT_OF_LETTERS:
            raise Exception(f'Incorrect amount of mapped letters : {len(mapped_letters)}'
                            f'(wanted : {CORRECT_AMOUNT_OF_LETTERS}')

        return mapped_letters
