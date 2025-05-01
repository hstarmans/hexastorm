import os
import matplotlib
matplotlib.use("Webagg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, Rectangle
from matplotlib.ticker import MultipleLocator
from matplotlib.transforms import Affine2D
import numpy as np


# Get the directory where the current script is located
script_directory = os.path.dirname(os.path.abspath(__file__))

# Change the current working directory to the script's directory
os.chdir(script_directory)

print(f"placing results in {script_directory}")


def fix_dimensions(fig, ax, pix_per_mm):
    dpi = 100
    fig.set_dpi(dpi)
    fig.canvas.draw()
    width, height = fig.get_size_inches()
    
    # e.g. 1 unit, 13 pixels, 13/(dpi * 25.24) mm,  
    def dataunit_to_pix_xy():
        # origin
        x_pix = abs(ax.transData.transform((1, 0)) - ax.transData.transform((0, 0)))[0]
        y_pix = abs(ax.transData.transform((0, 1)) - ax.transData.transform((0, 0)))[1]
        return x_pix, y_pix
    x_pix, y_pix = dataunit_to_pix_xy()
    actual_height_mm = (height*dpi)/y_pix
    conv = actual_height_mm / (height*25.4)
    fig.set_size_inches(width * y_pix/x_pix * conv, height * conv)
    fig.set_dpi(round(pix_per_mm*25.4))
    fig.canvas.draw()
    x_pix, y_pix = dataunit_to_pix_xy()
    # test 1 mm in x direction is 1 mm in y direction
    assert round(x_pix,1) == round(y_pix, 1)
    return fig

## Jitter test
##
# create stack of vertical lines

pattern_x_width = 25       # mm
pattern_y_width = 30       # mm
final_linewidth = 0.300    # mm 
linewidth_start = 0.05     # mm
text_size = 4              # mm
pix_per_mm = 200           # pixels per mm
tick_size = 0.2            # mm
points_text = text_size / (1/72*25.4)
points_ticks = tick_size / (1/72*25.4)

plt.rcParams.update({'font.size': points_text,
                     'xtick.major.size': points_ticks,    # Size of major tick marks in points
                     'ytick.major.size': points_ticks,
                     'xtick.minor.size': points_ticks,    # Size of minor tick marks in points
                     'ytick.minor.size': points_ticks,
                     'grid.linewidth': points_ticks})   # Width of grid lines in points 
fig, ax = plt.subplots()
triangles = int(pattern_x_width // ( final_linewidth))
for i in range(1, triangles):
    triangle_x = np.array([-final_linewidth/2, 0, final_linewidth/2]) + i*(final_linewidth)
    if i % 2 == 0:
        triangle_y = np.array([pattern_y_width, 0, pattern_y_width])
        triangle_vertices = np.column_stack((triangle_x, triangle_y))
        if i < 3:
            pass
            #print(triangle_vertices)
        triangle = Polygon(triangle_vertices, closed=True, facecolor='black', edgecolor='none')
        # Add the patch to the axes
        ax.add_patch(triangle)


y_start = (pattern_y_width/final_linewidth)*linewidth_start
plt.xlim(0, pattern_x_width)  
plt.ylim(y_start, pattern_y_width)

y_ticks = np.arange(y_start, pattern_y_width + 1, 5)
y_labels = [f"{round(x*10)}" for x in y_ticks]
ax.grid(axis='y', linestyle='-', color='white', alpha=1)
ax.set_yticks(y_ticks)
ax.set_yticklabels(y_labels)
ax.xaxis.set_major_locator(MultipleLocator(base=10))
ax.set_xlabel("Offset [mm]")
ax.set_ylabel("Width [μm]")
for key, spine in ax.spines.items():
    spine.set_visible(False)
# ax.spines['top'].set_linewidth(1)
# ax.spines['right'].set_linewidth(1)
# ax.spines['bottom'].set_linewidth(1)
# ax.spines['left'].set_linewidth(1)
fix_dimensions(fig, ax, pix_per_mm)
#print(fig.get_size_inches()*25.4)
#print(fig.get_dpi())
print("Creating jittertest")
plt.savefig("jittertest.svg",  bbox_inches='tight', dpi=fig.get_dpi(), pad_inches=0)


## Creates a fan of lines
##
## 

final_linewidth = 0.300 # mm
linewidth_start = 0.05  # mm
pattern_radius = 30     # mm

# Calculate the coordinates of the polygon's corners
triangle_vertices = np.array([[0, 0], [-final_linewidth/2, pattern_radius], [final_linewidth/2, pattern_radius]])

# Create the figure and axes
fig, ax = plt.subplots()
ax.set_aspect('equal') #Ensure the triangle isn't distorted

# Function to create and add a rotated triangle
def add_rotated_triangle(ax, vertices, angle, color='black'):
    """
    Adds a rotated triangle to the given axes.

    Parameters:
        ax (matplotlib.axes.Axes): The axes to add the triangle to.
        vertices (numpy.ndarray): The vertices of the triangle.
        angle (float): The rotation angle in radians.
        color (str, optional): The color of the triangle. Defaults to 'black'.
    """
    # Create a rotation matrix using Affine2D
    rotation_matrix = Affine2D().rotate(angle)

    # Apply the rotation to the triangle vertices
    rotated_vertices = rotation_matrix.transform(vertices)

    # Create a Polygon patch with the rotated vertices
    triangle = Polygon(rotated_vertices, closed=True, facecolor=color, edgecolor=None)
    ax.add_patch(triangle)

angle = np.arctan(final_linewidth/(pattern_radius*2))*2
triangles = int((0.5*np.pi)/(angle*2)+1)

for i in range(1, triangles):
    add_rotated_triangle(ax, triangle_vertices, -2*i*angle)

ax.set_xlim(0, pattern_radius)  
ax.set_ylim(0, pattern_radius)  



r_start = (pattern_radius/final_linewidth)*linewidth_start
# remove lines smaller than 50 micron
circle = Circle((0, 0), r_start, facecolor='white', alpha=1, edgecolor='white')

# Add the circle to the axes
ax.add_patch(circle)

xticks = np.arange(0, pattern_radius + 1, 10)
xlabels = [f"{round(x*10)}" for x in xticks]
yticks = np.arange(0, pattern_radius + 1, 5)
ylabels = [f"{round(x*10)}" for x in yticks]
ax.set_xticks(xticks)
ax.set_xticklabels(xlabels)
ax.set_yticks(yticks)
ax.set_yticklabels(ylabels)
label = "width [μm]"
ax.set_xlabel(label)
ax.set_ylabel(label)

# add grid lines for ticks
for tick in yticks:
    circle = Circle((0, 0), tick, fill=False, edgecolor='white', linewidth=1)
    ax.add_patch(circle)

for key, spine in ax.spines.items():
    spine.set_visible(False)

fix_dimensions(fig, ax, pix_per_mm)
print("Creating fantest")
plt.savefig("fantest.svg",  bbox_inches='tight', dpi=fig.get_dpi(), pad_inches=0)


## Creates a cross scan error test
##
##
pattern_x_width = 25       # mm
horizitonal_lines = 50
final_linewidth = 0.300    # mm 
linewidth_start = 0.025    # mm
stepsize = 0.025
height_line_group = 5      # mm
lines_per_group = 10       # number

plt.rcParams.update({'font.size': points_text,
                     'xtick.major.size': points_ticks,    # Size of major tick marks in points
                     'ytick.major.size': points_ticks,
                     'xtick.minor.size': points_ticks,    # Size of minor tick marks in points
                     'ytick.minor.size': points_ticks,
                     'grid.linewidth': points_ticks})   # Width of grid lines in points 



def thicknes_line(n):
    return n*stepsize+linewidth_start



fig, ax = plt.subplots()

#fix_dimensions(fig, ax, pix_per_mm)
y_ticks = np.arange(5, pattern_y_width + 1, height_line_group)
y_labels = [f"{thicknes_line(idx)*1000:.0f}" for idx, _ in enumerate(y_ticks)]

assert lines_per_group <  int(height_line_group // thicknes_line(len(y_ticks)))

for idx, pos_height in enumerate(y_ticks):
    thickness = thicknes_line(idx)
    start_height = pos_height - lines_per_group * thickness
    for line in range(lines_per_group):
       rect = Rectangle((0, start_height+thickness*2*line), pattern_x_width, thickness, linewidth=0, edgecolor='none', facecolor='black')
       # Add the patch to the axes
       ax.add_patch(rect)

ax.set_yticks(y_ticks)
ax.set_yticklabels(y_labels)
ax.xaxis.set_major_locator(MultipleLocator(base=10))
ax.set_xlabel("Offset [mm]")
ax.set_ylabel("Width [μm]")
for key, spine in ax.spines.items():
    spine.set_visible(False)
plt.xlim(0, pattern_x_width)  
plt.ylim(0, pattern_y_width+0.5*height_line_group)
fix_dimensions(fig, ax, pix_per_mm)
print("Creating crossscantest")
plt.savefig("crosscantest.svg",  bbox_inches='tight', dpi=fig.get_dpi(), pad_inches=0)