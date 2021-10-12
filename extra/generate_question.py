import json
import sys

from random import randint


def generate_robots():
    result = []

    for i in range(500 + 1):
        result.append({'id': i, 'capacity': 30})

    return result


def generate_orders():
    def minute_to_oclock(minute):
        return minute // 60 * 100 + minute % 60

    result = []

    for i in range(32767 + 1):
        r_address = (randint(0, 150), randint(0, 150))
        u_address = (randint(0, 150), randint(0, 150))
        start_minute = randint(11 * 60, 12 * 60 + 30)
        end_minute = randint(start_minute + 30, 13 * 60)

        result.append({'id': i, 'r_address': r_address, 'u_address': u_address, 'start_time': minute_to_oclock(start_minute), 'end_time': minute_to_oclock(end_minute)})

    return result


def main():
    json.dump({'robots': generate_robots(), 'orders': generate_orders()}, sys.stdout)


if __name__ == '__main__':
    main()
