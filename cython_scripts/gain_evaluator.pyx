# gain_evaluator.pyx

#cdef extern from "eth_mav_msgs/eigen_mav_msgs.h":
#    cdef cppclass eth_mav_msgs:
#        
#cdef extern from "voxblox/core/tsdf_map.h":
#    cdef cppclass voxblox:
#
#cdef extern from "voxblox/utils/camera_model.h":
#    cdef cppclass voxblox:

cdef extern from "gain_evaluator.h":
    cdef cppclass GainEvaluator:
        GainEvaluator()
        void setCameraModelParametersFoV(double, double, double, double)
        void setCameraModelParametersFocalLength(double[2], double, double, double)
        void setCameraExtrinsics(voxblox.Transformation)
        void setTsdfLayer(voxblox.Layer[voxblox.TsdfVoxel]*)
        double evaluateExplorationGainBircher(eth_mav_msgs.EigenTrajectoryPoint, int)
        voxblox.CameraModel& getCameraModel()
        const voxblox.CameraModel& getCameraModel()

# Define a Python wrapper class
cdef class PyGainEvaluator:
    cdef GainEvaluator* thisptr  # Pointer to the C++ object

    def __cinit__(self):
        self.thisptr = new GainEvaluator()

    def __dealloc__(self):
        del self.thisptr

    def setCameraModelParametersFoV(self, double horizontal_fov, double vertical_fov,
                                    double min_distance, double max_distance):
        self.thisptr.setCameraModelParametersFoV(horizontal_fov, vertical_fov, min_distance, max_distance)

    def setCameraModelParametersFocalLength(self, list resolution, double focal_length,
                                            double min_distance, double max_distance):
        cdef double[2] res_arr
        res_arr[0] = resolution[0]
        res_arr[1] = resolution[1]
        self.thisptr.setCameraModelParametersFocalLength(res_arr, focal_length, min_distance, max_distance)

    def setCameraExtrinsics(self, voxblox.Transformation T_C_B):
        self.thisptr.setCameraExtrinsics(T_C_B)

    def setTsdfLayer(self, voxblox.Layer[voxblox.TsdfVoxel]* tsdf_layer):
        self.thisptr.setTsdfLayer(tsdf_layer)

    def evaluateExplorationGainBircher(self, eth_mav_msgs.EigenTrajectoryPoint pose, int modulus):
        return self.thisptr.evaluateExplorationGainBircher(pose, modulus)

    def getCameraModel(self):
        return self.thisptr.getCameraModel()
