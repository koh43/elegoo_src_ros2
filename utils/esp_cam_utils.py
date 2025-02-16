import cv2
import requests

framesize_dict = {
    'FRAMESIZE_96X96'   : 0,  # 96x96
    'FRAMESIZE_QQVGA'   : 1,  # 160x120
    'FRAMESIZE_128X128' : 2,  # 128x128
    'FRAMESIZE_QCIF'    : 3,  # 176x144
    'FRAMESIZE_HQVGA'   : 4,  # 240x176
    'FRAMESIZE_240X240' : 5,  # 240x240
    'FRAMESIZE_QVGA'    : 6,  # 320x240
    'FRAMESIZE_320X320' : 7,  # 320x320
    'FRAMESIZE_CIF'     : 8,  # 400x296
    'FRAMESIZE_HVGA'    : 9,  # 480x320
    'FRAMESIZE_VGA'     : 10, # 640x480
    'FRAMESIZE_SVGA'    : 11, # 800x600
    'FRAMESIZE_XGA'     : 12, # 1024x768
    'FRAMESIZE_HD'      : 13, # 1280x720
    'FRAMESIZE_SXGA'    : 14, # 1280x1024
    'FRAMESIZE_UXGA'    : 15, # 1600x1200
}

def init_camera(params):
    url = f"http://{params["cam_ip"]}:{params["cam_port"]}/stream"
    set_framesize(params['cam_ip'], 'FRAMESIZE_VGA')
    set_quality(params["cam_ip"], 12)
    set_brightness(params["cam_ip"], 0)
    set_contrast(params["cam_ip"], 0)
    set_saturation(params["cam_ip"], 0)
    camera = cv2.VideoCapture(url)
    return camera

def set_params(esp_ip, param, value):
    url = f"http://{esp_ip}/control?var={param}&val={value}"
    response = requests.get(url)
    if response.status_code == 200:
        print(f"Set {param} to {value}")
        return True
    else:
        print(f"Failed to set {param}")
        return False

def set_framesize(esp_ip, framesize):
    if framesize not in framesize_dict:
        print("Framesize should be in one of the available items in this list:")
        print(list(framesize_dict.keys()))
        return False
    return set_params(esp_ip, "framesize", framesize_dict[framesize])

def set_quality(esp_ip, quality):
    if quality > 63 or quality < 4:
        print("JPEG quality should be between [4, 63] (lower the better, default 10)")
        return False
    return set_params(esp_ip, "quality", quality)

def set_brightness(esp_ip, br):
    if br > 2 or br < -2:
        print("Brightness should be between [-2, 2]")
        return False
    return set_params(esp_ip, "brightness", br)

def set_contrast(esp_ip, contrast):
    if contrast > 2 or contrast < -2:
        print("Contrast should be between [-2, 2]")
        return False
    return set_params(esp_ip, "contrast", contrast)

def set_saturation(esp_ip, sat):
    if sat > 2 or sat < -2:
        print("Saturation should be between [-2, 2]")
        return False
    return set_params(esp_ip, "saturation", sat)