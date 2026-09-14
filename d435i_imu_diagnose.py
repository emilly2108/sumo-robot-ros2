"""ROS2 없이 연결된 RealSense 장치와 IMU 프로파일을 확인한다.

실행:
    python d435i_imu_diagnose.py
"""

import pyrealsense2 as rs


def info(device, field):
    """지원하지 않는 장치 정보도 오류 없이 출력한다."""
    if device.supports(field):
        return device.get_info(field)
    return '(not reported)'


def main():
    context = rs.context()
    devices = context.query_devices()
    print(f'Connected RealSense devices: {len(devices)}')

    if not devices:
        print('No RealSense device found. Reconnect it directly to a USB 3 port.')
        return

    for number, device in enumerate(devices, start=1):
        print(f'\nDevice {number}')
        print(f'  Name:   {info(device, rs.camera_info.name)}')
        print(f'  Serial: {info(device, rs.camera_info.serial_number)}')
        print(f'  USB:    {info(device, rs.camera_info.usb_type_descriptor)}')

        has_accel = False
        has_gyro = False
        for sensor in device.query_sensors():
            print(f'  Sensor: {info(sensor, rs.camera_info.name)}')
            for profile in sensor.get_stream_profiles():
                stream = profile.stream_type()
                if stream not in (rs.stream.accel, rs.stream.gyro):
                    continue
                motion_profile = profile.as_motion_stream_profile()
                stream_name = 'accel' if stream == rs.stream.accel else 'gyro'
                print(
                    f'    {stream_name}: format={motion_profile.format()}, '
                    f'fps={motion_profile.fps()}'
                )
                has_accel = has_accel or stream == rs.stream.accel
                has_gyro = has_gyro or stream == rs.stream.gyro

        if has_accel and has_gyro:
            print('  Result: IMU available')
        else:
            print('  Result: IMU unavailable — this is not a usable D435i IMU connection.')


if __name__ == '__main__':
    main()

