"""Dump a specific CSV row as key:value pairs for debugging."""
import csv
import sys


def main():
    path = sys.argv[1]
    row_num = int(sys.argv[2])
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    r = rows[row_num - 1]
    for k, v in r.items():
        print(f"{k:40s} = {v}")


if __name__ == "__main__":
    main()
