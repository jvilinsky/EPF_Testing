import numpy as np

class EPF_Harness:
    def __init__(self):
        self.uav_uri = "cf5"
        self.ca_threshold2 = 0.5
        self.poses_dict = {}
        self.poses_dict["cf5"] = {
            "x" : -0.2,
            "y" : 0.0,
            "z" : 1.0,
        }
        self.poses_dict["cf6"] = {
            "x" : 0.2,
            "y" : 0.0,
            "z" : 1.0,
        }
        self.wp = Waypoint(0.5,0,1.0)
        self.twist_vx = 1.3
        self.poses = POSE("cf6")

    def epf_ca(self):
        uri = self.uav_uri
        
        ka = 0.3 
        kr = 0.01 
        ng = 2
        do = self.ca_threshold2
        alpha = 0.5
        gam = np.rad2deg(45)

        R_2 = np.array([[np.cos(gam),-np.sin(gam),0],
                        [np.sin(gam),np.cos(gam),0],
                        [0,0,1]])
        R_3 = np.array([[np.cos(gam),0,np.sin(gam)],
                        [0,1,0],
                        [-np.sin(gam),0,np.cos(gam)]])
        R_2_T = R_2.T
        R_3_T = R_3.T

        #distance from uav to goal:
        d_Nr_Nrg = np.sqrt((self.wp.x-self.poses_dict[uri]["x"])**2 + \
                                    (self.wp.y-self.poses_dict[uri]["y"])**2 + \
                                    (self.wp.z-self.poses_dict[uri]["z"])**2)
        #direction from uav to goal:
        C = np.array([self.wp.x,self.wp.y,self.wp.z]) - \
            np.array([self.poses_dict[uri]["x"],self.poses_dict[uri]["y"],self.poses_dict[uri]["z"]])
        dir_Nr_Nrg = C/np.linalg.norm(C)
        #attractive potential field
        fa = ka*d_Nr_Nrg*dir_Nr_Nrg
        #Default repulsive potential field
        fr = 0

        for pose in self.poses:
            uri2 = pose #uri2 = pose.name
            if uri2 != uri:
                #distance from uav to object:
                distance = np.sqrt((self.poses_dict[uri2]["x"]-self.poses_dict[uri]["x"])**2+\
                                    (self.poses_dict[uri2]["y"]-self.poses_dict[uri]["y"])**2+\
                                    (self.poses_dict[uri2]["z"]-self.poses_dict[uri]["z"])**2)
                
                if distance <= self.ca_threshold2:
                    d_Nr_Nro = distance
                    #direction from uav to object:
                    C = np.array([self.poses_dict[uri2]["x"],self.poses_dict[uri2]["y"],self.poses_dict[uri2]["z"]]) - \
                        np.array([self.poses_dict[uri]["x"],self.poses_dict[uri]["y"],self.poses_dict[uri]["z"]])
                    dir_Nr_Nro = C/np.linalg.norm(C)
                    #uav position:
                    Nr = np.array([self.poses_dict[uri]["x"],self.poses_dict[uri]["y"],self.poses_dict[uri]["z"]])
                    #object position:
                    Nro = np.array([self.poses_dict[uri2]["x"],self.poses_dict[uri2]["y"],self.poses_dict[uri2]["z"]])
                    #creating e frame:
                    vx = self.twist_vx #vx = self.msg_odom.twist.twist.linear.x
                    e1_h = np.array([vx,0])
                    e1_v = np.array([0,vx])
                    Horizontal_dir_Nr_Nro = np.array([dir_Nr_Nro[0],dir_Nr_Nro[1]])
                    Vertical_dir_Nr_Nro = np.array([dir_Nr_Nro[2],dir_Nr_Nro[0]])
                    #relative angles
                    a1 = Horizontal_dir_Nr_Nro; b1 = e1_h; a2 = Vertical_dir_Nr_Nro; b2 = e1_v
                    rho_h = np.arccos(np.dot(a1,b1)/(np.abs(a1)*np.abs(b1)))
                    rho_v = np.arccos(np.dot(a2,b2)/(np.abs(a2)*np.abs(b2)))
                    #Rotation matrix determination
                    if np.rad2deg(rho_h) <= 180:
                        R_h = R_3
                    elif np.rad2deg(rho_h) > 180:
                        R_h = R_3_T
                    if np.rad2deg(rho_v) <= 180:
                        R_v = R_2
                    elif np.rad2deg(rho_v) > 180:
                        R_v = R_2_T
                    q_h = R_h*(Nro-Nr)
                    q_v = R_v*(Nro-Nr)
                    q_prime = np.array([alpha*q_h[0],alpha*q_h[1],(1-alpha)*q_v[2]])
                    q_hat = q_prime/np.linalg.norm(q_prime)
                    fr_e = -kr*((d_Nr_Nrg**ng)/(d_Nr_Nro**2))*((1/d_Nr_Nro)-(1/do))*q_hat
                    fr_g = (1/2)*ng*kr*d_Nr_Nrg**(ng-1)*((1/d_Nr_Nro)-(1/do))**2*dir_Nr_Nrg
                    fr = fr_e + fr_g
        ft = fa + fr
        return ft[0],ft[1],ft[2]

class Waypoint:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
class POSE:
    def __init__(self,cf):
        self.name = cf
        self.index = 0
    def __iter__(self):
        return self
    def __next__(self):
        if self.index < 1:
            result = self.name
            self.index += 1
            return result
        else:
            raise StopIteration

def main():
    epf_harness = EPF_Harness()
    Vx,Vy,Vz = epf_harness.epf_ca()
    print(f"Vx ={Vx}, Vy ={Vy}, Vz ={Vz}")

if __name__ == "__main__":
    main()