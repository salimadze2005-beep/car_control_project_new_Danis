"""Image decoding and wall-time video assembly, independent of ROS."""
import math
from pathlib import Path

import cv2
import numpy as np


def decode_image(message):
    encoding = message.encoding.lower()
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}
    if encoding not in channels:
        raise ValueError('Unsupported image encoding: ' + encoding)
    count = channels[encoding]
    width, height, step = message.width, message.height, message.step
    if width <= 0 or height <= 0 or step < width * count:
        raise ValueError('Invalid image dimensions/row stride')
    if len(message.data) != height * step:
        raise ValueError('Image buffer length does not match height * step')
    pixels = np.frombuffer(message.data, np.uint8).reshape(height, step)
    pixels = pixels[:, :width * count].reshape(height, width, count)
    conversions = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
                   'bgra8': cv2.COLOR_BGRA2BGR, 'mono8': cv2.COLOR_GRAY2BGR}
    return (cv2.cvtColor(pixels, conversions[encoding]) if encoding in conversions
            else np.ascontiguousarray(pixels))


def assemble_video(output, frames, end_s, fps):
    """Zero-order hold on receipt time; preserve gaps, never invent motion.

    Video time zero is the first accepted frame, not recorder startup. Original
    PNGs and frames.csv remain recoverable if encoding is interrupted.
    """
    if not frames:
        return {'video_created': False, 'video_frames': 0, 'video_duration_s': 0.0}
    output = Path(output)
    first = cv2.imread(str(output / frames[0]['file']))
    if first is None:
        raise RuntimeError('Cannot read first recorded PNG')
    height, width = first.shape[:2]
    count = max(1, math.ceil(max(0., end_s - frames[0]['elapsed_s']) * fps))
    temporary = output / 'camera.partial.mp4'
    writer = cv2.VideoWriter(str(temporary), cv2.VideoWriter_fourcc(*'mp4v'),
                             fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError('Cannot open MP4 encoder; source PNGs are preserved')
    source_index, frame = 0, first
    try:
        for index in range(count):
            timestamp = frames[0]['elapsed_s'] + index / fps
            next_index = source_index
            while next_index + 1 < len(frames) and frames[next_index + 1]['elapsed_s'] <= timestamp:
                next_index += 1
            if next_index != source_index:
                frame = cv2.imread(str(output / frames[next_index]['file']))
                if frame is None:
                    raise RuntimeError('Missing source frame: ' + frames[next_index]['file'])
                if frame.shape[:2] != (height, width):
                    frame = cv2.resize(frame, (width, height))
                source_index = next_index
            writer.write(frame)
    finally:
        writer.release()
    if not temporary.exists() or temporary.stat().st_size == 0:
        raise RuntimeError('Encoder produced no video')
    temporary.replace(output / 'camera.mp4')
    return {'video_created': True, 'video_frames': count,
            'video_duration_s': count / fps,
            'video_start_elapsed_s': frames[0]['elapsed_s'],
            'video_fps': fps, 'video_timing': 'monotonic_receipt_zero_order_hold'}
