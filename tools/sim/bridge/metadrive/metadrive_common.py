import numpy as np

from metadrive.component.sensors.rgb_camera import RGBCamera
from panda3d.core import Texture, GraphicsOutput


class CopyRamRGBCamera(RGBCamera):
  """Camera which copies its content into RAM during the render process, for faster image grabbing."""
  def __init__(self, *args, **kwargs):
    """Initialize camera and attach an in-memory render target."""
    super().__init__(*args, **kwargs)
    self.cpu_texture = Texture()
    self.buffer.addRenderTexture(self.cpu_texture, GraphicsOutput.RTMCopyRam)

  def get_rgb_array_cpu(self):
    """Return the latest camera frame as an RGB uint8 image."""
    cpu_texture_image = self.cpu_texture
    rgba_image = np.frombuffer(cpu_texture_image.getRamImage().getData(), dtype=np.uint8)
    rgba_image = rgba_image.reshape((cpu_texture_image.getYSize(), cpu_texture_image.getXSize(), -1))

    # MetaDrive provides RGBA; bridge consumers expect RGB.
    rgb_image = rgba_image[:, :, :3]

    # Match camera orientation expected by downstream image consumers.
    rgb_image = rgb_image[::-1]
    return rgb_image


class RGBCameraWide(CopyRamRGBCamera):
  """Wide-angle RGB camera used for wide road visualization."""
  def __init__(self, *args, **kwargs):
    """Initialize wide FoV camera settings used for the wide stream."""
    super().__init__(*args, **kwargs)
    lens = self.get_lens()
    lens.setFov(120)
    lens.setNear(0.1)


class RGBCameraRoad(CopyRamRGBCamera):
  """Narrow-angle RGB camera used for the primary road stream."""
  def __init__(self, *args, **kwargs):
    """Initialize narrow FoV camera settings used for the road stream."""
    super().__init__(*args, **kwargs)
    lens = self.get_lens()
    lens.setFov(40)
    lens.setNear(0.1)
