from scripts.src.samples.sample_script import sum_numbers


def test_given_image_then_return_point():
    expected_sum = 0

    actual_sum = sum_numbers([])

    assert actual_sum == expected_sum


def test_given_a_single_number_then_return_that_number():
    numbers = [5]

    actual_sum = sum_numbers(numbers)

    assert actual_sum == numbers[0]


def test_given_many_numbers_then_return_sum():
    expected_sum = 7
    numbers = [3, 4]

    actual_sum = sum_numbers(numbers)

    assert actual_sum == expected_sum
