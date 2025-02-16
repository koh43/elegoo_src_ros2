import rclpy

from nodes.serial import pub_serial

def main(args=None):
    rclpy.init(args=args)

    params = {
        "node_name": "pub_arduino_serial",
        "ns" : "/arduino/serial",
        "queue_size": 5,
        "esp_ip": "192.168.1.103",
        "max_hz": 1000
    }
    app = pub_serial.PUB_SERIAL(params)

    rclpy.spin(app)
    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()