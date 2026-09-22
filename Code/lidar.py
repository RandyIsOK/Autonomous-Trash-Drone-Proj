from rplidar import RPLidar

lidar = RPLidar('/dev/ttyUSB0', 460800)

health = False
objectdet = {
    "front": False,
    "back": False,
    "left": False,
    "right": False
}

status, error_code = lidar.get_health()

if status == 'Good':
    print('Lidar is healthy')
    health = True

elif status == 'Warning':
    print('Lidar has warnings')
    health = True

elif status == 'Error':
    print('Lidar is not healthy')
    health = False


if health:
    print('Starting Lidar motor')
    lidar.start_motor()

else:
    print('Lidar is not healthy, cannot start motor')
    lidar.stop_motor()


def get_scan(scan_generator):
    for raw_scan in scan_generator:
        scan_data = []
        for (quality, angle, distance) in raw_scan:
            scan_data.append((quality, angle, distance))
        return scan_data
    return []


try:
    scan_generator = lidar.iter_scans()

    while True:
        scan_data = get_scan(scan_generator)

        objectdet = {
            "front": False,
            "back": False,
            "left": False,
            "right": False
        }

        for (quality, angle, distance) in scan_data:
            if distance < 1000:
                if angle > 340 or angle < 20:
                    objectdet["front"] = True
                elif 80 < angle < 100:
                    objectdet["left"] = True
                elif 260 < angle < 280:
                    objectdet["right"] = True

            if distance < 500:
                if 160 < angle < 200:
                    objectdet["back"] = True

        print(objectdet)

finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()