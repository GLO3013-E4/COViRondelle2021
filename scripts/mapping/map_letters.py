import pytesseract


CUSTOM_CONFIG = r'--oem 3 --psm 11 -c'
CORRECT_AMOUNT_OF_LETTERS = 9


# TODO : Test this
def map_letters(image):
    found_letters = pytesseract.image_to_string(image, config=CUSTOM_CONFIG)

    print(f'Found letters : {found_letters}')
    lookup_letters = ['A', 'B', 'C', 'D']
    mapped_letters = []

    for found_letter in found_letters:
        if found_letter in lookup_letters:
            mapped_letters.append(found_letter)

    print(f'Correct letters : {mapped_letters}')

    if len(mapped_letters) is not CORRECT_AMOUNT_OF_LETTERS:
        raise Exception(f'Incorrect amount of mapped letters : {len(mapped_letters)}'
                        f'(wanted : {CORRECT_AMOUNT_OF_LETTERS}')

    return mapped_letters
