
# In this file we saved some data extracted the CAD models of the iCub
# In particular, we extracted the transformations between a 
# R_SOLE_FRAME and the centers of each taxel Triangles placed in an arrangement under 
# the foot. Then, we express the same in the FT sensor frame of the right foot.


import iDynTree
import numpy as np
# random id, just for iDynTree semantics check
r_sole_cad_frame_id = 1
FTsensor_r_foot_frame_id = 2

FTsensor_r_foot_T_r_sole = \
    iDynTree.Transform(iDynTree.Rotation(   1.0,   0, 0, \
                                         0,   -1.0,    0, \
                                            0, 0,    -1.0),\
                       iDynTree.Position(3.5,0.0,7.5));
                       
FTsensor_r_foot_T_r_sole.getSemantics().setPoint(r_sole_cad_frame_id)    
FTsensor_r_foot_T_r_sole.getSemantics().setOrientationFrame(r_sole_cad_frame_id)
FTsensor_r_foot_T_r_sole.getSemantics().setReferencePoint(FTsensor_r_foot_frame_id)    
FTsensor_r_foot_T_r_sole.getSemantics().setReferenceOrientationFrame(FTsensor_r_foot_frame_id)   

# triangle center positions and FT sensor-to-Triangle center transformations for the triangles in FTsensor_r_foot_frame,
# contained in a dictonary where the triangle number is the key
r_sole_T_triangle = {}
ft_r_foot_T_triangle = {}
positionDict = {}
orientation = {}
def addTriangle3DCenter(triangleNumber, x, y, z,rotation,ref_frame_id, positionDict):
    pos = iDynTree.Position(x,y,z);
    pos.getSemantics().setReferencePoint(ref_frame_id);
    pos.getSemantics().setCoordinateFrame(ref_frame_id);
    theta = rotation * 3.14 / 180;
    r_sole_T_triangle[triangleNumber] = iDynTree.Transform(iDynTree.Rotation(np.cos(theta),-np.sin(theta),0,np.sin(theta),np.cos(theta),0,0,0,1),pos)
    ft_r_foot_T_triangle[triangleNumber] = FTsensor_r_foot_T_r_sole * r_sole_T_triangle[triangleNumber]	
    positionDict[triangleNumber] = FTsensor_r_foot_T_r_sole * pos	


addTriangle3DCenter(9,108,26,0,12,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(10,112,7,0,-168,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(11,98,-5,0,12,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(14,102,-24,0,-168,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(15,87,-37,0,12,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(0,69,-31,0,-168,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(1,65,-12,0,12,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(12,79,1,0,-168,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(13,75,19,0,12,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(8,90,32,0,-168,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(4,57,25,0,-168,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(3,43,12,0,12,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(2,47,-6,0,-168,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(21,32,-19,0,12,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(27,0,-26,0,12,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(26,14,-13,0,-168,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(25,10,5,0,12,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(22,24,18,0,-168,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(24,-8,11,0,-168,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(29,-23,-1,0,12,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(28,-19,-20,0,-168,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(17,-33,-33,0,12,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(18,-51,-27,0,-168,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(19,-55,-8,0,12,r_sole_cad_frame_id,positionDict)
addTriangle3DCenter(20,-41,4,0,-168,r_sole_cad_frame_id,positionDict)

