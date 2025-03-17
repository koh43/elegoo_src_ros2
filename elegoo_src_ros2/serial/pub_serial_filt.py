import rclpy

from nodes.serial import pub_serial_filt

def main(args=None):
    rclpy.init(args=args)

    params = {
        "node_name": "pub_arduino_serial",
        "queue_size": 5,
    }
    app = pub_serial_filt.PUB_SERIAL_FILT(params)

    rclpy.spin(app)
    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()