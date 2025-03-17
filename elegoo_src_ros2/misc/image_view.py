import rclpy

from nodes.misc import image_view

def main(args=None):
    rclpy.init(args=args)

    params = {
        "topic_name": "/espcam/image_color",
        "img_type"  : "compressed"
    }

    app = image_view.IMAGE_VIEW(params)

    rclpy.spin(app)
    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
