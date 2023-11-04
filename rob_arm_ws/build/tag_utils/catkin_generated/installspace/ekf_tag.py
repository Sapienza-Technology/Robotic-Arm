import numpy as np
from numpy import sin,cos
from math import pi,atan2
import math
import rospy
#odometry message
from nav_msgs.msg import Odometry
#import POseStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from time import  sleep
class EKF:
    def __init__(self, x0, P0, F, Q, x_i, R,params):
        # x_i: posizione nota dei tag in visibilità, precompilata prima.
        self.x = x0 # Matrice (n, 1) contenente la posizione corrente stimata del rover
        self.P = P0 # Matrice di covarianza dell'errore (n, n) che descrive l'incertezza nella posizione corrente stimata
        self.F = F # Matrice di transizione del modello (n, n) che descrive come la posizione viene stimata durante la predizione
        self.Q = Q # Matrice di covarianza del processo (n, n) che descrive l'incertezza nelle previsioni del modello di transizione
        #self.H = H # Matrice di trasformazione del modello di misura (m, n) che descrive come la posizione misurata dei landmark viene mappata nella stima della posizione corrente
        self.x_i=np.array(x_i) #matrice con 2 righe 15 colonne con dati dei tag in visibilità(dati all'inizio)

        self.tag_gt=self.x_i #posizione globale ground truth dei tag 
        self.R = R # Matrice di covarianza del rumore di misura (m, m) che descrive l'incertezza nella posizione misurata dei landmark
        
        #publisher
        self.pub=rospy.Publisher("/ekf_tag",Odometry,queue_size=10)
        self.pose_pub=rospy.Publisher("/ekf_tag_poseStamped",PoseStamped,queue_size=10)
        self.rate=rospy.Rate(10)


        self.updating=False  #avoid conflict between update and predict

        self.max_visible_range=params["max_visible_range"] #max distance from the rover to the tag to be considered visible
        self.max_visible_angle=params["max_visible_angle"] #max angle from the rover to the tag to be considered visible

        self.num_landmarks=params["num_landmarks"] #number of landmarks
        
        #other pose information, not used, only published
        self.orientation=[0,0,0]
        self.height=0
        self.params=params

    def predict(self):
        #not used if using odometry from other node
        self.x = np.dot(self.F, np.reshape(self.x,(3,1))) # Calcola la posizione stimata durante la predizione utilizzando la matrice di transizione del modello
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q # Aggiorna la matrice di covarianza dell'errore utilizzando la matrice di covarianza del processo

    def fake_predict(self,X,P):
        #get the odometry from other node
        if self.updating:
            return
        self.x=X
        self.P=P

    def compute_tag_in_range(self):
        #from the estimated position of the rover, compute the visible tag in range
        #could be useful to check if the tag detected has a wrong id
        theta=self.x[2]
        ids=[]
        positions=[]
        #print("estimate tag in range")
        for i in range(self.num_landmarks):
            #print("check tag",i+1)
            #x,y tag
            tag=[self.tag_gt[0][i],self.tag_gt[1][i]]

            relative_x=tag[0]-self.x[0]
            relative_y=tag[1]-self.x[1]
            relative_distance=np.sqrt(relative_x**2+relative_y**2)

            #landmark too far
            if relative_distance>self.max_visible_range:
                #print("too far,distance:",relative_distance) 
                continue
                
            
            relative_angle=atan2(relative_y,relative_x)
            angle_diff = abs(self.x[2] - relative_angle)
            
            if angle_diff > pi:
                angle_diff = 2*pi - angle_diff

            #landamrk behind the rover or not in the field of view
            if angle_diff <= self.max_visible_angle / 2 :
                ids.append(i)
            #else: print("not in angle of view, angle:",math.degrees(relative_angle),"angle diff:",math.degrees(angle_diff))

        return ids
    
    def compute_measurement(self,tag_id):
        #compute where I should see the landmark
        tag=[self.tag_gt[0][tag_id],self.tag_gt[1][tag_id]]

        relative_x=tag[0]-self.x[0]
        relative_y=tag[1]-self.x[1]

        theta=self.x[2]

        #transposed rotation matrix
        Rt=np.array([[cos(theta),sin(theta)],
                     [-sin(theta),cos(theta)]])
        
        #print("relative:",relative_x,relative_y)
        #print shapes in single line
        #print("Rt: ",Rt.shape," relative: ",np.array([relative_x,relative_y]).shape)
        z=np.dot(Rt,np.array([relative_x,relative_y]))

        return z

    def publish_state(self):
        odom_msg=Odometry()
        #header
        odom_msg.header.stamp=rospy.Time.now()
        odom_msg.header.frame_id="map"

        odom_msg.pose.pose.position.x=self.x[0]
        odom_msg.pose.pose.position.y=self.x[1]
        odom_msg.pose.pose.position.z=self.height

        #orientation to quaternion
        r=self.orientation[0]
        p=self.orientation[1]
        y=self.x[2]
        q=quaternion_from_euler(r,p,y)
        odom_msg.pose.pose.orientation.x=q[0]
        odom_msg.pose.pose.orientation.y=q[1]
        odom_msg.pose.pose.orientation.z=q[2]
        odom_msg.pose.pose.orientation.w=q[3]

        odom_msg.pose.covariance[0]=self.P[0,0]
        odom_msg.pose.covariance[1]=self.P[0,1]
        odom_msg.pose.covariance[2]=self.P[0,2]
        odom_msg.pose.covariance[6]=self.P[1,0]
        odom_msg.pose.covariance[7]=self.P[1,1]
        odom_msg.pose.covariance[8]=self.P[1,2]
        odom_msg.pose.covariance[12]=self.P[2,0]
        odom_msg.pose.covariance[13]=self.P[2,1]
        odom_msg.pose.covariance[14]=self.P[2,2]
        self.pub.publish(odom_msg)


        #create PoseStamped message
        pose_msg=PoseStamped()
        pose_msg.header.stamp=rospy.Time.now()
        pose_msg.header.frame_id="map"
        pose_msg.pose.position.x=self.x[0]
        pose_msg.pose.position.y=self.x[1]
        pose_msg.pose.position.z=self.height
        pose_msg.pose.orientation.x=q[0]
        pose_msg.pose.orientation.y=q[1]
        pose_msg.pose.orientation.z=q[2]
        pose_msg.pose.orientation.w=q[3]
        self.pose_pub.publish(pose_msg)
        

        #publish transform tree
        br=self.params["broadcaster"]
        roll=self.orientation[0]
        pitch=self.orientation[1]
        yaw=self.x[2]
        #br.sendTransform((self.x[0], self.x[1], self.height),
        #                tf.transformations.quaternion_from_euler(roll,pitch,yaw),
        #                rospy.Time.now(),
        #                "base_link",
        #                "map")
    
    def update(self, z , z_hat):
        #z: actual measurement
        #z_hat: predicted measurement
        previous_state=self.x
        self.updating=True
        if len(z)==0:
            self.updating=False
            return
        H=self._get_H(z_hat) # size 2kx3 k=num landmark detected

        y = z - z_hat # Calcola l'errore di misura (size=2k x 1 k=num landmark detected)

        S = np.dot(np.dot(H, self.P), H.T) + self.R # Calcola la matrice di covarianza dell'errore di misura
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S)) # Calcola la gain di Kalman (size 3x2k k=num landmark detected)
        new_x = np.reshape(self.x,(3,1)) + np.dot(K, y) # Aggiorna la posizione corrente stimata
        self.x=np.reshape(new_x,(3,))
        self.P = self.P - np.dot(np.dot(K, H), self.P) # Aggiorna la matrice di covarianza dell'errore
        
        print("EKF UPDATE:")
        #print state rounding to 3 decimal places
        print("Previous state:\n",np.round(previous_state,4))
        print("new State:\n",np.round(self.x,4))
        print("P:\n",np.round(self.P,4))
        print("\n\n")

        self.publish_state()
        self.updating=False


    def get_state(self):
        return self.x, self.P
    
    def _get_H(self, z_hat): 
        #non_zero_cols = np.where(self.x_i(axis=0))[0] # questo da le non zero columns
        #new_arr = np.array[:,non_zero_cols]
        h=[]
        x=self.x[0]
        y=self.x[1]
        theta=self.x[2]

        s=sin(theta)
        c=cos(theta)
        # transposed rotation matrix
        Rt=np.array([[c,-s],
                     [s,c]])
        
        #derivative of the transposed rotation matrix
        Rtp=np.array([[-s,c],
                      [-c,-s]])

        H=[]
        #print("z_hat",z_hat,"shape",np.shape(z_hat))
        
        for i in range(np.shape(z_hat)[0]//2):
            l_x=z_hat[i,0]
            l_y=z_hat[i*2+1,0]

            h=np.zeros((2,3))
            h[0:2,0:2]=np.array(-Rt)
            q=np.dot(Rtp,np.array([[l_x-x],[l_y-y]]))
            h[0:2,2]=np.reshape(np.dot(Rtp,np.array([[l_x-x],[l_y-y]])),(2,))
            #print("cosa",h)
            H.append(h[0,:])
            H.append(h[1,:])

        return np.array(H)



def odomCallback(msg,ekf):
    X=msg.pose.pose.position
    #turn into 3x1 vector
    
    orientation=msg.pose.pose.orientation
    euler_angles=euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
    theta=euler_angles[2]

    ekf.orientation=euler_angles
    ekf.height=msg.pose.pose.position.z

    X=np.array([X.x,X.y,theta])
    #print
    #print("received state: ",X)

    #get original 6x6 covariance matrix
    P=msg.pose.covariance
    #put needed components into 3x3 matrix
    #we only need the x,y,theta components
    P=np.array([[P[0],P[1],P[5]],[P[6],P[7],P[11]],[P[30],P[31],P[35]]])
    ekf.fake_predict(X, P)
    #sleep(0.1)

def tagCallback(msg,ekf):
    #my data is in the form [x1,y1,cov1,x2,y2,cov2,...]
    #where x1,y1 are the coordinates of the first tag, and cov1 is the covariance of the measurement
    #there are 15 tags in total so the message is 45 elements long
    z_raw=msg.data
    #z is a (2xk) matrix, where k is the number of tags detected
    #turn into 2xk matrix
    z=[]
    z_hat=[]
    covariances=[]

    visible_tags=ekf.compute_tag_in_range()
    print("rover should see tags: ",[t+1 for t in visible_tags])
    for i in range(ekf.num_landmarks):
        
        tag_x=z_raw[i*3]
        tag_y=z_raw[i*3+1]

        #check if tag has been measured
        if tag_x==0 and tag_y==0: continue
        
        #where the rover should see the landmark
        x_hat,y_hat=ekf.compute_measurement(i)


        #print with only 3 decimal places
        print("Measured tag",i+1,"at",np.round(tag_x,3),np.round(tag_y,3),end="")
        print(" should be at",np.round(x_hat,3),np.round(y_hat,3)) 
        
        #check discrepancy
        discrepancy=np.sqrt((tag_x-x_hat)**2+(tag_y-y_hat)**2)
        e=np.sqrt((tag_x-x_hat)**2+(tag_y-y_hat)**2)
        print(f"\tError:",np.round(e,3),"cov:",np.round(z_raw[i*3+2],3),"discrepancy:",np.round(discrepancy,3))
        print()
        
        #if i not in visible_tags:
        #    print(f"tag {i+1} wrongly detected, ignoring...")
        #    continue


        tag_id=i
        max_discrepancy=5
        if discrepancy>max_discrepancy:
            print("WARNING: possible wrong ID..",end="")
            #find closest tag
            min_discrepancy=1e4
            min_tag=-1
            for j in range(ekf.num_landmarks):
                x_hat,y_hat=ekf.compute_measurement(j)
                discrepancy=np.sqrt((tag_x-x_hat)**2+(tag_y-y_hat)**2)
                if discrepancy<min_discrepancy:
                    min_discrepancy=discrepancy
                    min_tag=j
            if min_discrepancy>max_discrepancy:
                print("WARNING: no feasible tag found, ignoring measurement")
                continue
            tag_id=min_tag
            print("maybe correct tag is",min_tag+1,"with discrepancy",np.round(min_discrepancy,3))
        
        x_hat,y_hat=ekf.compute_measurement(tag_id)


        print("\n")
        #measurement
        z.append([tag_x])
        z.append([tag_y])
        #predicted measurement
        z_hat.append([x_hat])
        z_hat.append([y_hat])
        
        #save covariances for later
        covariances.append(z_raw[i*3+2])
    
    #create covariance matrix
    R=np.zeros((len(covariances)*2,len(covariances)*2))
    for i in range(len(covariances)):
        cov=covariances[i]
        #fill diagonal
        R[i*2,i*2]=cov 
        R[i*2+1,i*2+1]=cov
    
    #update covariance of measurement 
    ekf.R=R

    z=np.array(z)
    z_hat=np.array(z_hat)
    #print("z shape",z.shape)
    #print("z_hat shape",z_hat.shape)
    ekf.update(z,z_hat)
    sleep(0.1)

def main():
    #ros node 
    rospy.init_node("EKF_tag", anonymous=True)
    
    # Inizializza la posizione stimata iniziale x0, la matrice di covarianza dell'errore P0, la matrice di transizione del modello F,
    # la matrice di covarianza del processo Q, la matrice di trasformazione del modello di misura H, e la matrice di covarianza del rumore di misura R

    x0 = np.array([0,0,0]) # Matrice (3, 1) contenente lamatrice iniziale della posizione stimata del rover
    P0_i=1e-2
    P0 = np.array([[P0_i, 0, 0], [0, P0_i, 0], [0, 0, P0_i]]) # Matrice (3, 3) di covarianza iniziale dell'errore
    
    F = np.array([[1, 0, 1], [0, 1, 0], [0, 0, 1]]) # Matrice (3, 3) di transizione del modello
    Q = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]) # Matrice (3, 3) di covarianza del processo
    
    landmark_positions=[
        [10,-0],
        [10,-10],
        [28.35,-0.04],
        [21.83,-2.80],
        [18.71,17.19],
        [26.95,-7.44],
        [15.97,7.57],
        [17.87,-7.57],
        [10,10],
        [29.26,-14.52],
        [18.41,-25.83],
        [23.34,-14.11],
        [8.18,-18.63],
        [1e3,1e3],
        [2.27,-16.84]
    ]   

    x_i=np.array(np.transpose(landmark_positions)) #posizioni dei tag in coordinate globali (2 righe x 15 colonne)

    num_landmarks=x_i.shape[1] #numero di tag

    R = np.eye(num_landmarks) # Matrice (2, 2) di covarianza del rumore di misura

    #transform broadcaster
    br = tf.TransformBroadcaster()

    params={
        "max_visible_range": 15,
        "max_visible_angle": math.radians(120),
        "num_landmarks": num_landmarks,
        "broadcaster": br,

    }

    ekf = EKF(x0, P0, F, Q, x_i, R,params) # Crea un'istanza della classe EKF

    # Esegui la predizione e l'aggiornamento per un numero fissato di step


    #subscriber
    rospy.Subscriber("odom", Odometry, lambda msg: odomCallback(msg,ekf))

    rospy.Subscriber("tag_topic", Float32MultiArray, lambda msg: tagCallback(msg,ekf))


    rospy.spin()

main()