"""
ImageHelper for combining images and drawing multiline texts

Written by Kolin Guo
"""
import re
from pathlib import Path
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont
from typing import Sequence, Tuple, List
FONT_PATH = str(Path(__file__).resolve().parent / "ubuntu-font-family-0.83/UbuntuMono-R.ttf")

class ImageHelper:
    def __init__(self, boundary_padding: int = 20, text_bbox_padding: int = 4,
                 bg_color: List[np.uint8] = [0, 0, 0]):
        """
        :param _image: actual image np.ndarray. [H, W, C]
                       DO NOT edit it directly (will lead to undefined behavior)
        :param image_bboxes: tag sub-image bboxes in _image. [left, top, right, bottom]
        :param text_bboxes: text bboxes in _image. [left, top, right, bottom]
                            When drawing the new text and encounter overlaps,
                            shift the new text bbox so that there's no overlap.
        :param boundary_padding: number of pixels to pad if drawn outside of image shape.
        :param text_bbox_padding: number of pixels to pad around drawn text bboxes.
        :param bg_color: background color used for padding
        """
        self._image = np.zeros((0, 0, 3), dtype=np.uint8)
        self.image_bboxes = {}  # {tag: bbox}
        self.text_bboxes = []
        self.boundary_padding = boundary_padding
        self.text_bbox_padding = text_bbox_padding
        self.bg_color = bg_color


    def _check_image_format(self, image: np.ndarray):
        assert image.dtype == self._image.dtype, \
            f'image must have dtype "{self._image.dtype}", get {image.dtype}'
        assert image.ndim in [3], f'image must be 3 dimensional, get {image.ndim}'
        assert image.shape[2] in [3, 4], \
            f'image must have 3 (RGB) or 4 (RGBA) channels, get {image.shape[2]}'


    def _check_out_of_bounds(self, bbox: Sequence[float]) -> bool:
        """Check if the bbox is out of _image bounds"""
        left, top, right, bottom = bbox
        return left < 0 or top < 0 or right > self.width or bottom > self.height


    @staticmethod
    def _update_bbox(left, top, right, bottom, left_pad, top_pad):
        return [left + left_pad, top + top_pad, right + left_pad, bottom + top_pad]


    def _update_bboxes(self, left_pad, top_pad):
        """Update all bboxes due to padding"""
        for tag, bbox in self.image_bboxes.items():
            self.image_bboxes[tag] = self._update_bbox(*bbox, left_pad, top_pad)

        for i in range(len(self.text_bboxes)):
            self.text_bboxes[i] = self._update_bbox(*self.text_bboxes[i], left_pad, top_pad)


    def _pad_image(self, bbox: Sequence[float], xy: Sequence[float], ret_bbox=False) -> Sequence[float]:
        """Pad image to fit bbox"""
        left, top, right, bottom = bbox
        left_pad, top_pad, right_pad, bottom_pad = 0, 0, 0, 0

        if left < 0:
            left_pad = int(np.ceil(-left)) + self.boundary_padding
            xy[0] += left_pad

        if top < 0:
            top_pad = int(np.ceil(-top)) + self.boundary_padding
            xy[1] += top_pad

        if right > self.width:
            right_pad = int(np.ceil(right-self.width)) + self.boundary_padding

        if bottom > self.height:
            bottom_pad = int(np.ceil(bottom-self.height)) + self.boundary_padding

        # Update bboxes
        self._update_bboxes(left_pad, top_pad)

        # Pad image with zeros
        pad_width = [[top_pad, bottom_pad], [left_pad, right_pad], [0, 0]]
        self._image = np.pad(self._image, pad_width, constant_values=self.pad_values)
        if ret_bbox:
            return xy, self._update_bbox(*bbox, left_pad, top_pad)
        else:
            return xy


    def add_image(self, image: np.ndarray, tag: str, xy: Sequence[float],
                  rel_tag: str = 'whole', show_vis=False):
        """Add an image named tag at xy position where xy is top-left corner
        :param image: image to add. If read by cv2, make sure it's ordered as RGB/RGBA.
        :param tag: the image tag for future reference.
        :param xy: the top-left corner in self._image to add the image.
        :param rel_tag: tag of the relative image top-left corner to use.
                        'whole' means xy is relative to whole image.
                        Otherwise, xy is relative to the tag image top-left corner.
        :param show_vis: whether to show visualization.
        """
        self._check_image_format(image)
        assert len(xy) == 2, f'xy should only contain top-left corner (x, y), get {xy}'
        assert tag != 'whole', 'Tag "whole" is reserved for internal usage'

        height, width, channel = image.shape
        # Update xy to be relative to whole image
        if rel_tag != 'whole':
            xy[0] += self.image_bboxes[rel_tag][0]
            xy[1] += self.image_bboxes[rel_tag][1]
        xy = np.floor(xy).astype(int)

        image_bbox = [xy[0], xy[1], xy[0]+width, xy[1]+height]
        self.image_bboxes[tag] = image_bbox

        if self._check_out_of_bounds(image_bbox):
            xy = self._pad_image(image_bbox, xy)

        image_bbox = self.image_bboxes[tag]
        self._image[image_bbox[1]:image_bbox[3], image_bbox[0]:image_bbox[2]] = image.copy()

        if show_vis:
            self.show_image(window_name=f'Add image "{tag}"')

        return self.image


    def _pad_image_for_text(self, text_bbox: Sequence[float], xy: Sequence[float]):
        """Pad image for drawing text"""
        xy, text_bbox = self._pad_image(text_bbox, xy, ret_bbox=True)
        # Update due to change in self._image
        img = Image.fromarray(self._image).convert('RGBA')
        txt_im = Image.new("RGBA", img.size, (255, 255, 255, 0))
        d = ImageDraw.Draw(txt_im)
        return xy, text_bbox, img, txt_im, d


    def draw_multiline_text(self, xy: Sequence[float], text: str, tag: str = 'whole',
                            fill: Tuple[np.uint8] = None,
                            font_path: str = FONT_PATH, font_size: int = None,
                            anchor: str = None, spacing=4, align='left',
                            text_bbox_overlap_shift='right',
                            draw_text_bbox=False, draw_text_bbox_kwargs={},
                            show_vis=False, ret_bbox=False) -> np.ndarray:
        """Draw a multiline text using PIL. Extend image size when necessary
           When drawing the new text and encounter overlaps,
           shift the new text bbox so that there's no overlap.
        :param xy: The anchor coordinates of the text. (x, y)
        :param text: string to be drawn, can contain multilines.
        :param tag: tag of the image to draw on. 'whole' means xy is relative to whole image.
                    Otherwise, xy is relative to the tag image.
        :param fill: color used for drawing text, (R,G,B) or (R,G,B,A)
        :param font_path: path to a TrueType or OpenType font to use
        :param font_size: the requested font size, in points
        :param anchor: The text anchor alignment. Determines the relative location of
                       the anchor to the text. The default alignment is top left.
                       See :ref:`text-anchors` for valid values.
                       https://pillow.readthedocs.io/en/stable/handbook/text-anchors.html#text-anchors
                       "tb" of anchor[1] are not supported for multiline text
        :param spacing: The number of pixels between lines.
        :param align: "left", "center" or "right". Determines the relative alignment
                      of lines. Use the anchor parameter to specify the alignment to xy.
        :param text_bbox_overlap_shift: "left", "right", "up", "down". How to shift the new text
                                        when there are overlaps.
        :param draw_text_bbox: whether to draw text bbox.
        :param draw_text_bbox_kwargs: kwargs for drawing text bbox: {'fill', 'outline', 'width'}.
        :param show_vis: whether to show visualization.
        :param ret_bbox: whether to return text bbox (should only be used internally).
        :return out_image: output image, always RGBA format [H, W, 4].
        """
        assert tag == 'whole' or tag in self.image_bboxes, \
                f'tag "{tag}" does not exist. Existing tags: {list(self.image_bboxes.keys())}'
        assert len(xy) == 2, f'xy should only contain anchor (x, y), get {xy}'
        assert text_bbox_overlap_shift in ["left", "right", "up", "down"], \
                f'Unknown text_bbox_overlap_shift {text_bbox_overlap_shift}'

        if font_path is None:
            font = ImageFont.load_default()
            if font_size is not None: print('[ Warning ] Default font does not support changing size.')
        else:
            # use a truetype font
            font = ImageFont.truetype(font_path, 10 if font_size is None else font_size)

        # Update xy to be relative to whole image
        if tag != 'whole':
            xy[0] += self.image_bboxes[tag][0]
            xy[1] += self.image_bboxes[tag][1]

        img = Image.fromarray(self._image).convert('RGBA')
        # make a blank image for the text, initialized to transparent text color
        txt_im = Image.new("RGBA", img.size, (255, 255, 255, 0))
        d = ImageDraw.Draw(txt_im)

        def _pad_bbox(bbox: Sequence[float], pad: int) -> Sequence[float]:
            left, top, right, bottom = bbox
            return [left - pad, top - pad, right + pad, bottom + pad]

        # bbox = (left, top, right, bottom)
        text_bbox = d.textbbox(xy, text, font, anchor, spacing, align)
        text_bbox = _pad_bbox(text_bbox, self.text_bbox_padding)
        # Check and pad image for text_bbox
        if self._check_out_of_bounds(text_bbox):
            xy, text_bbox, img, txt_im, d = self._pad_image_for_text(text_bbox, xy)

        # Check if there's overlap with text_bboxes
        if self.text_bboxes:
            left, top, right, bottom = text_bbox
            text_bboxes = np.array(self.text_bboxes)
            overlap_idx = ~((text_bboxes[:, 0] > right) | (text_bboxes[:, 2] < left) \
                          | (text_bboxes[:, 1] > bottom) | (text_bboxes[:, 3] < top))
            if np.any(overlap_idx):  # overlaps exist
                if text_bbox_overlap_shift == 'right':
                    max_right = np.max(text_bboxes[overlap_idx, 2])
                    xy[0] += max_right - left + 1
                else:
                    raise NotImplementedError('text_bbox_overlap_shift other than '
                                              '"right" is not yet implemented')
                text_bbox = d.textbbox(xy, text, font, anchor, spacing, align)
                text_bbox = _pad_bbox(text_bbox, self.text_bbox_padding)
                # Check and pad image for text_bbox
                if self._check_out_of_bounds(text_bbox):
                    xy, text_bbox, img, txt_im, d = self._pad_image_for_text(text_bbox, xy)

        # Actual drawing code
        if draw_text_bbox:
            d.rectangle(text_bbox, **draw_text_bbox_kwargs)
        # Draw the text
        d.text(xy, text, fill, font, anchor, spacing, align)
        out_im = Image.alpha_composite(img, txt_im)

        if show_vis:
            short_text = re.sub('[^ a-zA-Z0-9_]', '', text)[:25]
            self.show_image(out_im, window_name=f'Draw text "{short_text}"')

        if ret_bbox:
            return np.array(out_im), text_bbox
        else:
            return np.array(out_im)


    def add_multiline_text(self, xy: Sequence[float], text: str, **kwargs) -> np.ndarray:
        """Draw and add multiline text
        :param kwargs: kwargs for drawing image bbox: {'fill', 'outline', 'width'}.
                       Can also contain boolean show_vis
        """
        self._image, text_bbox = self.draw_multiline_text(xy, text, ret_bbox=True, **kwargs)
        self.text_bboxes.append(text_bbox)
        return self.image


    def draw_image_bboxes(self, image: np.ndarray = None, show_vis=False, **kwargs) -> np.ndarray:
        """Draw all image bboxes
        :param kwargs: kwargs for drawing image bbox: {'fill', 'outline', 'width'}.
        """
        if image is None:
            image = self._image

        img = Image.fromarray(image).convert('RGBA')
        # make a blank image for bbox, initialized to transparent color
        bbox_im = Image.new("RGBA", img.size, (255, 255, 255, 0))
        d = ImageDraw.Draw(bbox_im)

        for tag, image_bbox in self.image_bboxes.items():
            d.rectangle(image_bbox, **kwargs)
        out_im = Image.alpha_composite(img, bbox_im)

        if show_vis:
            self.show_image(out_im, window_name=f'Draw image bboxes')

        return np.array(out_im)


    def add_image_bboxes(self, **kwargs) -> np.ndarray:
        """Draw and add all image bboxes
        :param kwargs: kwargs for drawing image bbox: {'fill', 'outline', 'width'}.
                       Can also contain boolean show_vis
        """
        assert 'image' not in kwargs, f'image found in kwargs when calling add_image_bboxes()'
        self._image = self.draw_image_bboxes(**kwargs)
        return self.image


    def show_image(self, img: Image = None, use_cv2=True, window_name='Image'):
        if img is None:
            img = Image.fromarray(self._image)

        if not use_cv2:
            img.show(window_name)
        else:
            cv2.imshow(window_name, np.array(img)[..., [2, 1, 0]])
            cv2.waitKey(0)
            cv2.destroyAllWindows()


    def save_image(self, save_path):
        Image.fromarray(self._image).save(save_path)


    @property
    def image(self):
        """Read-only view of _image"""
        image = self._image.view()
        image.flags.writeable = False
        return image

    @property
    def cv2_image(self):
        """cv2 BGR image copy of _image"""
        img = Image.fromarray(self._image).convert('RGB')
        return np.array(img)[..., ::-1]  # RGB to BGR

    @property
    def height(self):
        return self._image.shape[0]

    @property
    def width(self):
        return self._image.shape[1]

    @property
    def channel(self):
        return self._image.shape[2]

    @property
    def shape(self):
        return self._image.shape

    @property
    def dtype(self):
        return self._image.dtype

    @property
    def color_format(self):
        if self.channel == 3:
            return 'RGB'
        elif self.channel == 4:
            return 'RGBA'
        else:
            raise NotImplementedError(f'Image channel {self.channel} is not yet implemented')

    @property
    def pad_values(self):
        # TODO: check if dtype=object causes issues
        if self.color_format == 'RGB':
            return np.asarray([[self.bg_color] * 2] * 2 + [[0, 0]], dtype=object)
        elif self.color_format == 'RGBA':
            return np.asarray([[self.bg_color+[255]] * 2] * 2 + [[0, 0]], dtype=object)
        else:
            raise NotImplementedError(f'Image channel {self.channel} is not yet implemented')

    ### Properties of sub-images contained in image_bboxes ###
    @property
    def sub_images_left(self):
        """Left boundary of all sub-images"""
        if self.image_bboxes:
            return min([bbox[0] for bbox in self.image_bboxes.values()])
        else:
            return 0

    @property
    def sub_images_top(self):
        """Top boundary of all sub-images"""
        if self.image_bboxes:
            return min([bbox[1] for bbox in self.image_bboxes.values()])
        else:
            return 0

    @property
    def sub_images_right(self):
        """Right boundary of all sub-images"""
        if self.image_bboxes:
            return max([bbox[2] for bbox in self.image_bboxes.values()])
        else:
            return self.width

    @property
    def sub_images_bottom(self):
        """Bottom boundary of all sub-images"""
        if self.image_bboxes:
            return max([bbox[3] for bbox in self.image_bboxes.values()])
        else:
            return self.height

    @property
    def sub_images_bbox(self):
        """Enclosed bbox of all sub-images"""
        return [self.sub_images_left, self.sub_images_top, self.sub_images_right, self.sub_images_bottom]
