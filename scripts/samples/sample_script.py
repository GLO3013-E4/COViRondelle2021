#!/usr/bin/python
""" Sample script that serves as a template for scripts """
import argparse


def sum_numbers(numbers):
    """ Sums given integers"""
    total = 0

    for number in numbers:
        total += number

    return total


parser = argparse.ArgumentParser(description='Integers to sum')
parser.add_argument('integers', metavar='N', type=int, nargs='+',
                    help='an integer to add to the sum')

if __name__ == "__main__":
    args, leftover = parser.parse_known_args()

    if args is None:
        print("You need at least one integer as argument")
    else:
        print("Sum : " + str(sum_numbers(args.integers)))
