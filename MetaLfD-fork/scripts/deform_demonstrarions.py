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
import os


#fnames = ['21_10', '23_28', '32_45', '36_56', '42_36', '45_33', '49_47', '53_56', '56_47']
fnames = ['32_45']


fname_start = '/home/bhertel/catkin_ws/h5 files/bad preprocessed recorded_demo Fri Jul  3 14:45:33 2020.h5'
fname_end = ''

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

def get_open_house_demo():
        full_name = fname_start
        [x, y, z] = get_xyz_demo(full_name)
        
        print(np.shape(x))
        
        my_mlfd = mlfd.metalfd()
        
        save_fpath = '../example_outputs/45_33/'
        
        my_mlfd.add_traj(np.hstack((np.reshape(x, (len(x), 1)), np.reshape(y, (len(y), 1)), np.reshape(z, (len(z), 1)))))
        my_mlfd.add_representation(ja.perform_ja_general, 'JA')
        my_mlfd.add_representation(lte.LTE_ND_any_constraints, 'LTE')
        my_mlfd.add_representation(dmp.perform_dmp_general, 'DMP')
        my_mlfd.add_metric(similaritymeasures.frechet_dist, is_dissim=True)
        #my_mlfd.create_grid()
        #my_mlfd.deform_traj()
        #my_mlfd.calc_similarities()
        #my_mlfd.save_to_h5(filename=save_fpath + 'mlfd_data.h5')
        my_mlfd.load_from_h5(filename=save_fpath + 'mlfd_data.h5')
        my_mlfd.get_strongest_sims(0.2)
        #my_mlfd.plot_strongest_sims(mode='show')
        my_mlfd.set_up_classifier()
        #my_mlfd.reproduce_at_point(coords=[[1, 10, 20]], plot=True)
        #my_mlfd.plot_classifier_results(filepath=save_fpath)
        #my_mlfd.plot_cube3D(filepath=save_fpath)
        return my_mlfd.show_3d_in_2d_with_slider()

def main():
    for f in fnames:
        full_name = fname_start
        [x, y, z] = get_xyz_demo(full_name)
        
        print(np.shape(x))
        
        my_mlfd = mlfd.metalfd()
        
        save_fpath = '../example_outputs/' + f + '/'
        
        try:
            os.makedirs(save_fpath)
        except OSError:
            print ("Creation of the directory %s failed" % save_fpath)
        else:
            print ("Successfully created the directory %s" % save_fpath)
        
        #xnew = mlfd.downsample_traj(np.reshape(x, (len(x), 1)))
        #print(xnew)
        #print(np.shape(xnew))
        
        my_mlfd.add_traj(np.hstack((mlfd.downsample_traj(np.reshape(x, (len(x), 1))), mlfd.downsample_traj(np.reshape(y, (len(y), 1))), mlfd.downsample_traj(np.reshape(z, (len(z), 1))))))
        my_mlfd.add_representation(ja.perform_ja_general, 'JA')
        my_mlfd.add_representation(lte.LTE_ND_any_constraints, 'LTE')
        my_mlfd.add_representation(dmp.perform_dmp_general, 'DMP')
        my_mlfd.add_metric(similaritymeasures.frechet_dist, is_dissim=True)
        #my_mlfd.create_grid()
        #my_mlfd.deform_traj()
        #my_mlfd.calc_similarities()
        #my_mlfd.save_to_h5(filename=save_fpath + 'mlfd_data.h5')
        my_mlfd.load_from_h5(filename=save_fpath + 'mlfd_data.h5')
        my_mlfd.get_strongest_sims(0.2)
        #my_mlfd.plot_strongest_sims(mode='show')
        my_mlfd.set_up_classifier()
        #my_mlfd.reproduce_at_point(coords=[[1, 10, 20]], plot=True)
        #my_mlfd.plot_classifier_results(filepath=save_fpath)
        #my_mlfd.plot_cube3D(filepath=save_fpath)
        my_mlfd.show_3d_in_2d_with_slider()
     
if __name__ == '__main__':
  main()
