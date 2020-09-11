import mlfd
import numpy as np
import h5py
import similaritymeasures
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import ja
import lte
import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, './dmp_pastor_2009/')
import perform_dmp as dmp


def get_xyz_demo(filename):
    #open the file
    hf = h5py.File(filename, 'r')
    #navigate to necessary data and store in numpy arrays
    demo = hf.get('demo1')
    tf_info = demo.get('tf_info')
    pos_rot_data = tf_info.get('pos_rot_data')
    pos_rot_data = np.array(pos_rot_data)
    #close out file
    hf.close()
    x = pos_rot_data[0]
    y = pos_rot_data[1]
    z = pos_rot_data[2]
    return [x, y, z]

def main():
    full_name = '../data/reaching_demo_data.h5'
    [x, y, z] = get_xyz_demo(full_name)
    
    print(np.shape(x))
    
    my_mlfd = mlfd.metalfd()
    
    
    #xnew = mlfd.downsample_traj(np.reshape(x, (len(x), 1)))
    #print(xnew)
    #print(np.shape(xnew))
    
    my_mlfd.add_traj(np.hstack((mlfd.downsample_traj(np.reshape(x, (len(x), 1))), mlfd.downsample_traj(np.reshape(y, (len(y), 1))), mlfd.downsample_traj(np.reshape(z, (len(z), 1))))))
    my_mlfd.add_representation(ja.perform_ja_general, 'JA')
    my_mlfd.add_representation(lte.LTE_ND_any_constraints, 'LTE')
    my_mlfd.add_representation(dmp.perform_dmp_general, 'DMP')
    my_mlfd.add_metric(similaritymeasures.frechet_dist, is_dissim=True)
    my_mlfd.create_grid()
    my_mlfd.deform_traj()
    my_mlfd.calc_similarities()
    my_mlfd.save_to_h5(filename='../data/reaching_demo_mlfd.h5')
    #my_mlfd.load_from_h5(filename='../data/reaching_demo_mlfd.h5')
    my_mlfd.get_strongest_sims(0.2)
    #my_mlfd.plot_strongest_sims(mode='show')
    my_mlfd.set_up_classifier()
    #my_mlfd.reproduce_at_point(coords=[[1, 10, 20]], plot=True)
    my_mlfd.plot_classifier_results(filepath='../example_outputs/')
    my_mlfd.plot_cube3D(filepath='../example_outputs/')
    #my_mlfd.show_3d_in_2d_with_slider()
     
if __name__ == '__main__':
  main()