import skimage.io
import numpy as np
import matplotlib.pyplot as plt

img = skimage.io.imread('rice_42.png')

fig, ax = plt.subplots(1,2, figsize=(20,10))
ax[0].text(50, 100, 'original image', fontsize=16, bbox={'facecolor': 'white', 'pad': 6})
ax[0].imshow(img)

img = np.flipud(img)
d = np.zeros((img.shape[0], img.shape[1]))

from PIL import Image as PImage
import plotly.graph_objects as go

def create_rgb_surface(rgb_img, depth_img, **kwargs):
    rgb_img = rgb_img.swapaxes(0, 1)[:, ::-1]
    depth_img = depth_img.swapaxes(0, 1)[:, ::-1]
    eight_bit_img = PImage.fromarray(rgb_img).convert('P', palette='WEB', dither=None)
    idx_to_color = np.array(eight_bit_img.getpalette()).reshape((-1, 3))
    colorscale=[[i/255.0, "rgb({}, {}, {})".format(*rgb)] for i, rgb in enumerate(idx_to_color)]
    depth_map = depth_img.copy().astype('float')
    # depth_map[depth_map<depth_cutoff] = np.nan
    return go.Surface(
        z=depth_map,
        surfacecolor=np.array(eight_bit_img),
        cmin=0, 
        cmax=255,
        colorscale=colorscale,
        showscale=False,
        **kwargs
    )

fig = go.Figure(
    data=[create_rgb_surface(img, 
                             d,
                             contours_z=dict(show=True, project_z=True, highlightcolor="limegreen"),
                             opacity=0.5
                            )],
    layout_title_text="3D Surface"
)

fig.update_layout(
    scene = dict(
        xaxis = dict(nticks=4, range=[0,img.shape[0]],),
        yaxis = dict(nticks=4, range=[0,img.shape[1]],),
        zaxis = dict(nticks=4, range=[0,500],),
        ),
    width=700,
    margin=dict(r=20, l=10, b=10, t=10))

fig.write_image("roll_3d_calib.png")
