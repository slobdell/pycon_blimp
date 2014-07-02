import cv
# from cv import EncodeImage, QueryFrame, CaptureFromCAM
# from cv import CV_BGR2GRAY, CvtColor, CreateImage, GetSize
# from cv import Resize, CV_IMWRITE_JPEG_QUALITY

TARGET_HEIGHT = 60
IMAGE_QUALITY = 40
IMAGE_QUALITY_ARR = [cv.CV_IMWRITE_JPEG_QUALITY, IMAGE_QUALITY, 0]


def _get_resized_image(frame):
    size = cv.GetSize(frame)
    new_width = TARGET_HEIGHT * size[0] / size[1]
    thumbnail = cv.CreateImage((new_width, TARGET_HEIGHT), frame.depth, frame.nChannels)
    cv.Resize(frame, thumbnail)
    return thumbnail


def _get_grayscale_image(frame):
    gray = cv.CreateImage(cv.GetSize(frame), 8, 1)
    cv.CvtColor(frame, gray, cv.CV_BGR2GRAY)
    return gray


class CameraManager(object):
    def __init__(self):
        self.capture = cv.CaptureFromCAM(0)

    def get_jpeg_frame(self):
        ipl_img_frame = cv.QueryFrame(self.capture)
        if ipl_img_frame is None:
            return None
        ipl_img_frame = _get_resized_image(ipl_img_frame)
        # ipl_img_frame = _get_grayscale_image(ipl_img_frame)

        cv_mat = cv.EncodeImage(".jpeg", ipl_img_frame, IMAGE_QUALITY_ARR)
        jpeg_bytes = cv_mat.tostring()
        return jpeg_bytes
