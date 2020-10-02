#ifdef OLD_HEADER_FILENAME
#include <iostream.h>
#else
#include <iostream>
#endif
#include <string>
#include "/home/bhertel/catkin_ws/src/brendan_ur5e/src/hdf5-1.10.4/c++/src/H5Cpp.h"
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <array>
using namespace H5;
const H5std_string  FILE_NAME( "SDS.h5" );
const H5std_string  DATASET_NAME( "IntArray" );
const int   NX = 5;                    // dataset dimensions
const int   NY = 6;
const int   RANK = 2;
int main (void)
{
   /*
    * Data initialization.
    */
   int i, j;
   int data[NX][NY];          // buffer for data to write
   for (j = 0; j < NX; j++)
   {
      for (i = 0; i < NY; i++)
     data[j][i] = i + j;
   }
   /*
    * 0 1 2 3 4 5
    * 1 2 3 4 5 6
    * 2 3 4 5 6 7
    * 3 4 5 6 7 8
    * 4 5 6 7 8 9
    */
   // Try block to detect exceptions raised by any of the calls inside it
   try
   {
      /*
       * Turn off the auto-printing when failure occurs so that we can
       * handle the errors appropriately
       */
      Exception::dontPrint();
      /*
       * Create a new file using H5F_ACC_TRUNC access,
       * default file creation properties, and default file
       * access properties.
       */
      H5File file( FILE_NAME, H5F_ACC_TRUNC );
      /*
       * Define the size of the array and create the data space for fixed
       * size dataset.
       */
      hsize_t     dimsf[2];              // dataset dimensions
      dimsf[0] = NX;
      dimsf[1] = NY;
      DataSpace dataspace( RANK, dimsf );
      /*
       * Define datatype for the data in the file.
       * We will store little endian INT numbers.
       */
      IntType datatype( PredType::NATIVE_INT );
      datatype.setOrder( H5T_ORDER_LE );
      /*
       * Create a new dataset within the file using defined dataspace and
       * datatype and default dataset creation properties.
       */
      DataSet dataset = file.createDataSet( DATASET_NAME, datatype, dataspace );
      /*
       * Write the data to the dataset using default memory space, file
       * space, and transfer properties.
       */
      dataset.write( data, PredType::NATIVE_INT );
   }  // end of try block
   // catch failure caused by the H5File operations
   catch( FileIException error )
   {
      error.printError();
      return -1;
   }
   // catch failure caused by the DataSet operations
   catch( DataSetIException error )
   {
      error.printError();
      return -1;
   }
   // catch failure caused by the DataSpace operations
   catch( DataSpaceIException error )
   {
      error.printError();
      return -1;
   }
   // catch failure caused by the DataSpace operations
   catch( DataTypeIException error )
   {
      error.printError();
      return -1;
   }
   return 0;  // successfully terminated
}
