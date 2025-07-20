import rclpy

from nodes.serial import pub_serial_raw

def main(args=None):
    rclpy.init(args=args)

    params = {
        "node_name": "pub_serial_raw",
        "queue_size": 5,
        "udp_port": 9750,
        "buffer_size": 128,
        "max_hz": 200,
        "num_calib_samples": 1000,
    }
    app = pub_serial_raw.PUB_SERIAL_RAW(params)

    rclpy.spin(app)
    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()