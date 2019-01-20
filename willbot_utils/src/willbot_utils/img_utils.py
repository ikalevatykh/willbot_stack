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

    match = parse_encoding(msg.encoding)
    if match:
        img_array = np.frombuffer(msg.data, dtype=match[0])
        img_array = img_array.reshape((msg.height, msg.width, match[1]))

        if encoding is not None:
            match_to = parse_encoding(encoding)
            if match_to and match_to[1] == match[1]:
                return img_array.astype(match_to[0])
            raise Exception('Cannot convert {} image to {}'.format(
                msg.encoding, encoding))

        return img_array

    raise Exception('Cannot convert {} image'.format(msg.encoding))


def parse_encoding(encoding):
    if encoding.startswith('32F'):
        dtype = np.float32
    elif encoding.startswith('16U'):
        dtype = np.uint16
    elif encoding.startswith('16S'):
        dtype = np.int16
    else:
        return None
    return dtype, int(encoding[-1])


def scale_depth(depth, near, far):
    depth = depth.clip(near, far)
    depth[np.isnan(depth)] = far
    depth = (255 * (depth - near) / (far - near)).astype(np.uint8)
    return depth
