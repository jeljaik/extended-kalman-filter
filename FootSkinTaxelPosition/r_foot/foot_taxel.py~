#!/usr/bin/env python 

import yarp
import numpy as np
import matplotlib.pyplot as plt
import sys
from mpl_toolkits.mplot3d import Axes3D
import scipy.interpolate
import iDynTree


# workaround because ResourceFinder::configure is not properly typemapped 
# in python

# Load yarp ResourceFinder 
rf = yarp.ResourceFinder()
rf.setVerbose();
# Set the same default context of the iCubSkinGui for convenience
rf.setDefaultContext("skinGui/skinGui");

if( "--from" not in sys.argv ):
    print("Usage : skinGui.py --from nameOfSkinGuiConfiguration.ini")
    skinGuiFileName = "right_foot.ini"
else:
    skinGuiFileName = sys.argv[sys.argv.index("--from")+1]

prop = yarp.Property();
prop.fromConfigFile(rf.findFileByName(skinGuiFileName));

print("Reading taxel positions from " + rf.findFileByName(skinGuiFileName))

sens_group = prop.findGroup("SENSORS")


triangles = []
trianglesDict = {}

for i in range(1,sens_group.size()):
    triangle_group = sens_group.get(i).asList();
    triangle = {}
    triangle["type"]   = triangle_group.get(0).asString();
    triangle["number"] = triangle_group.get(1).asInt();
    triangle["u"]      = triangle_group.get(2).asInt();
    triangle["v"]      = triangle_group.get(3).asInt();
    triangle["orient"] = triangle_group.get(4).asInt();
    triangle["gain"]   = triangle_group.get(5).asInt();
    triangle["mirror"] = triangle_group.get(6).asInt();
    
       triangles.append(triangle)
    trianglesDict[triangle["number"]] = triangle
    
# sort the triangle list based on number (id)
triangles.sort(lambda x,y : cmp(x['number'], y['number']))

# taxel positions in triangle frame (expressed in millimeters
taxelsPosInTriangle = []
# taxel 0
taxelsPosInTriangle.append(np.array([6.533, 0.0, 0.0]))
# taxel 1
taxelsPosInTriangle.append(np.array([9.8, -5.66, 0.0]))
# taxel 2
taxelsPosInTriangle.append(np.array([3.267, -5.66, 0.0]))
# taxel 3 
taxelsPosInTriangle.append(np.array([0.0, 0.0, 0.0]))
# taxel 4
taxelsPosInTriangle.append(np.array([-3.267, -5.66, 0.0]))
# taxel 5
taxelsPosInTriangle.append(np.array([-9.8, -5.66, 0.0]))
# taxel 6 (thermal pad!)
taxelsPosInTriangle.append(np.array([-6.533, -3.75, 0.0]))
# taxel 7  
taxelsPosInTriangle.append(np.array([-6.533, 0, 0.0]))
# taxel 8
taxelsPosInTriangle.append(np.array([-3.267, 5.66, 0.0]))
# taxel 9 
taxelsPosInTriangle.append(np.array([0.0, 11.317,0.0]))
# taxel 10 (thermal pad)
taxelsPosInTriangle.append(np.array([0, 7.507, 0.0]))
# taxel 11 
taxelsPosInTriangle.append(np.array([3.267, 5.66, 0.0]))




# generate taxel list, we allocate a list of the total number of triangles
# dummy values (for the foot are 32) and then we overwrite the taxels 
# for the real triangles 
dummy_taxel = {}
dummy_taxel["type"] = "dummy"
dummy_taxel["u"] = 0;
dummy_taxel["v"] = 0;
dummy_taxel["x"] = 0;
dummy_taxel["y"] = 0;
dummy_taxel["z"] = 7.5/1000;
dummy_taxel["triangleNumber"] = None;


# the total number of the triangles is composed by both real triangles
# and dummy triangles, is given by the length of the yarp vector published
# on the port, divided by 12 (for the torso: 384/12 = 32)
total_number_of_triangles = 32

# number of taxels for triangle
taxel_per_triangle = 12

# list of taxels (from 0 to taxel_per_triangle) that are thermal 
thermal_taxels_list = [6,10]

# taxel that is the center of the triangle
center_taxel = 3;


taxels = total_number_of_triangles*taxel_per_triangle*[dummy_taxel]

variables= {}
execfile( "right_footTrianglesCAD.py", variables )


ft_r_foot_T_triangle = variables["ft_r_foot_T_triangle"]
positionDict = variables["positionDict"]
ft_r_foot_T_taxel = {}

taxel_trans = {}
j = 0
for triangle in triangles:
    for i in range(0,taxel_per_triangle):
	
	taxel_trans[i] = iDynTree.Transform(iDynTree.Rotation(1,0,0,0,1,0,0,0,1),iDynTree.Position(taxelsPosInTriangle[i][0],taxelsPosInTriangle[i][1],taxelsPosInTriangle[i][2]))
	
	taxel = {}
        
        # index of the taxel in the skin part YARP port
	taxel["index"] = triangle["number"]*taxel_per_triangle+i;
        taxel["triangleNumber"] = triangle["number"]
	ft_r_foot_T_taxel[taxel["index"]] = ft_r_foot_T_triangle[triangle["number"]] * taxel_trans[i]
	#print ft_r_foot_T_taxel[taxel["index"]]
	#print taxel["index"]

        if( i in thermal_taxels_list ):
            taxel["type"] = "thermal"
            # x,y,z are the coordinates in millimeters of the taxels 
            # with respect to the FT sensor frame on the right foot
                                            
            taxel["x"] = ft_r_foot_T_taxel[taxel["index"]].getPosition()(0)/1000
            taxel["y"] = ft_r_foot_T_taxel[taxel["index"]].getPosition()(1)/1000
            taxel["z"] = ft_r_foot_T_taxel[taxel["index"]].getPosition()(2)/1000

            #u,v coordinates are referred in the icubSkinGui, not relevant for estimation
                      
            taxel["u"] = None
            taxel["v"] = None
	    
 
        else:
            taxel["type"] = "tactile"
           
            taxel["x"] = ft_r_foot_T_taxel[taxel["index"]].getPosition()(0)/1000
            taxel["y"] = ft_r_foot_T_taxel[taxel["index"]].getPosition()(1)/1000
            taxel["z"] = ft_r_foot_T_taxel[taxel["index"]].getPosition()(2)/1000
            
            
            taxel["u"] = None
            taxel["v"] = None

        taxels[taxel["index"]] = taxel







# plot 3d data 
fig = plt.figure()
ax = fig.add_axes([0, 0, 1, 1], projection='3d')
point_x3d = []
point_y3d = []
point_z3d = []

for taxel in taxels:
    point_x3d.append(taxel["x"]);
    point_y3d.append(taxel["y"]);
    point_z3d.append(taxel["z"]);
    

center_x = []
center_y = []
center_z = []

for key in positionDict:    
    label = "ID: " + str(trianglesDict[key]["number"]);
    ax.text(positionDict[key](0)/1000,positionDict[key](1)/1000,positionDict[key](2)/1000,label)
    center_x.append(positionDict[key](0)/1000);
    center_y.append(positionDict[key](1)/1000);
    center_z.append(positionDict[key](2)/1000);
ax.plot(center_x,center_y,center_z,'o',c="red");		
ax.plot(point_x3d,point_y3d,point_z3d,'.',c="blue");
ax.axis('equal')

fig2 = plt.figure()
ax2 = fig2.add_axes([0,0,1,1])
for key in positionDict:    
    label = "ID: " + str(trianglesDict[key]["number"]);
    ax2.text(positionDict[key](0)/1000,positionDict[key](1)/1000,label)

plt.plot(point_x3d,point_y3d,'bo')	

# export the 3d points to a skinManager "positions" compatible file
def exportSkinManagerPositionTxtFile(taxels,posx,posy,posz,normx,normy,normz,name,filename):
    assert(len(taxels) == len(posx))
    assert(len(taxels) == len(posy))
    assert(len(taxels) == len(posz))
    assert(len(taxels) == len(normx))
    assert(len(taxels) == len(normy))
    assert(len(taxels) == len(normz))
    out_file = open(filename,"w");
    out_file.write("name    " + name + "\n");
    out_file.write("spatial_sampling     taxel\n");
    # the convention relative to taxel2repr is that dummy taxels are 
    # mapped to -1, temperature taxel are mapped -2 and tacticle taxels
    # are mapped to the index of the taxel that is the center of the triangle
    out_file.write("taxel2Repr ( ")
    for taxel in taxels:
        if( taxel["type"] == "dummy" ):
            out_file.write(" -1 ");
        elif( taxel["type"] == "thermal"):
            out_file.write(" -2 ");
        elif( taxel["type"] == "tactile"):
            out_file.write(" " + str(taxel["triangleNumber"]*taxel_per_triangle+center_taxel) + " ");
        else:
            assert(false)
            
    out_file.write(" )\n");
    
    # out write 
    out_file.write("[calibration]\n");
    for taxel in taxels:
        if( taxel["type"] == "dummy" ):
            out_file.write("0.0 0.0 0.0 0.0 0.0 0.0 \n");
        elif( taxel["type"] == "thermal" or taxel["type"] == "tactile"):
            taxelIndex = taxel["index"]
            out_file.write(str(posx[taxelIndex]) + " " +  str(posy[taxelIndex]) + " " +  str(posz[taxelIndex]) + \
                           " " + str(normx[taxelIndex]) + " " +  str(normy[taxelIndex]) + " " + str(normz[taxelIndex]) + "\n");
        else:
            assert(false)
    out_file.write("\n")


def exportTaxelPositionforEstimation(taxels,posx,posy,posz,filename):
    assert(len(taxels) == len(posx))
    assert(len(taxels) == len(posy))
    assert(len(taxels) == len(posz))
    out_file = open(filename,"w");

    for key in range(0,len(posx)):
    	out_file.write(str(posx[key]) + " " +  str(posy[key]) + " " +  str(posz[key]) + "\n")
    	
    out_file.close()
    
    		
#normal for now are set to 0
normx = [0]*len(point_x3d)
normy = [0]*len(point_y3d)
normz = [0]*len(point_z3d)

exportSkinManagerPositionTxtFile(taxels,point_x3d,point_y3d,point_z3d,normx,normy,normz,"ft_r_foot","r_foot.txt");

exportTaxelPositionforEstimation(taxels,point_x3d,point_y3d,point_z3d,"r_foot_pos.mat");
plt.show()






