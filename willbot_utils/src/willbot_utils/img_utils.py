import numpy as np


def imgmsg_to_array(msg, encoding=None):
    if msg.encoding in ['rgb8', 'bgr8']:
        img_array = np.frombuffer(msg.data, dtype=np.uint8)
        img_array = img_array.reshape((msg.height, msg.width, 3))

        if encoding is not None:
            if encoding not in ['rgb8', 'bgr8']:
                raise Exception('Cannot convert {} image to {}'.format(
                    msg.encoding, encoding))
            if encoding != msg.encoding:
                img_array = np.flip(img_array, axis=2)

        return img_array

    if msg.encoding == '32FC1':
        img_array = np.frombuffer(msg.data, dtype=np.float32)
        img_array = img_array.reshape((msg.height, msg.width, 1))

        if encoding is not None and encoding != '32FC1':
            raise Exception('Cannot convert {} image to {}'.format(
                msg.encoding, encoding))

        return img_array

    raise Exception('Cannot convert {} image'.format(msg.encoding))


def scale_depth(depth, near, far):
    depth = depth.clip(near, far)
    depth[np.isnan(depth)] = far
    depth = (255 * (depth - near) / (far - near)).astype(np.uint8)
    return depth    
