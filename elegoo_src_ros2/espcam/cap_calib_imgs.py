import rclpy

from nodes.espcam import cap_calib_imgs

def main(args=None):
    rclpy.init(args=args)

    params = {
        'node_name'  : 'cap_calib_imgs',
        'queue_size' : 5,
        'cam_topic_name' : '/espcam/image_color',
        'save_path' : '/home/koh/espcam_calib/calib_images',
    }
    app = cap_calib_imgs.CAP_CALIB_IMGS(params)

    rclpy.spin(app)
    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()