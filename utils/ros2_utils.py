from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

def custom_qos_profile(queue_size):
    return QoSProfile(
        history                   = HistoryPolicy.KEEP_LAST,
        depth                     = queue_size,
        reliability               = ReliabilityPolicy.RELIABLE,
        durability                = DurabilityPolicy.VOLATILE
    )

def now(node):
    return node.get_clock().now().to_msg()

def loginfo(node, str, once_flag=False):
    node.get_logger().info(str, once=once_flag)

def to_sec(msg):
    seconds = msg.header.stamp.sec
    nanoseconds = msg.header.stamp.nanosec
    # Convert to float seconds
    return seconds + nanoseconds * 1e-9

def publisher(node, msg_type, topic, qos=10):
    return node.create_publisher(
        msg_type, topic, qos
    )

def subscriber(node, msg_type, topic, callback, qos=10):
    return node.create_subscription(
        msg_type,
        topic,
        callback,
        qos
    )

def timer(node, period, callback):
    return node.create_timer(
        period, callback
    )