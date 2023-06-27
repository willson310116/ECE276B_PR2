import numpy as np
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from utils import command_line_param
from testmap import wrap_test

if __name__=="__main__":
  args = command_line_param()
  wrap_test(args)
  plt.show(block=True)