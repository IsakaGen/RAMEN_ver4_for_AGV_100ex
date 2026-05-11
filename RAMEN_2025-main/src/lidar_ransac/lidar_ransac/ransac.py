import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random
import numpy as np

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_sample_node')
        self.ranges =[]
        self.points =[]
        self.plot_x =[]
        self.plot_y =[]
        self.lidar_sub = self.create_subscription(
            LaserScan,'/scan',self.lidar_cb,10
        )
        self.xnew =[]
        self.ynew =[]
        self.state = False
        
    def get_param(self,p_1,p_2):
        a = p_2[1]-p_1[1]
        b = p_1[0]-p_2[0]
        c = -1*p_1[0]*b-p_1[0]*a
        return a,b,c
    
    def euclid(self,params,point):
        a = params[0]
        b = params[1]
        c = params[2]
        x = point[0]
        y = point[1]
        noum =  math.sqrt(a**2+b**2)
        dist = math.fabs(a*x+b*y+c)/noum
        return dist
    
    def model(self,params,x):
        a = params[0]
        b = params[1]
        c = params[2]
        grad = -a/b
        intercept = -c/b
        return grad*x+intercept

    def ransac(self,max_loop,threshold,points):
        best_p_cnt = 0
        best_param =[]
        for i in range(max_loop):
            ran_num_1 = random.randint(0,len(points)-1)
            ran_num_2 = random.randint(0,len(points)-1)
            if ran_num_1 == ran_num_2:
                continue
            point_1 = points[ran_num_1]
            point_2 = points[ran_num_2]
            
            a,b,c = self.get_param(point_1,point_2)
            param =[]
            param.append(a)
            param.append(b)
            param.append(c)
            p_cnt =0
            for j in range(0,len(points)):
                p = points[j]
                dist = self.euclid(param,p)
                if dist <= threshold:
                    p_cnt+=1
            if p_cnt>best_p_cnt:
                best_p_cnt = p_cnt
                best_param = param
            
            #print(best_p_cnt)
                
        if best_p_cnt <=50:
            print("No line ")
            self.state = False
        else:
            self.state = True
            print("Line detected")
            #self.xnew = np.linspace(-2,2,10)
            #self.ynew =[]
            #for k in range(0,len(self.xnew)):
            #    self.ynew.append(self.model(params=best_param,x=self.xnew[k]))
    
    def lidar_cb(self,msg):
        self.ranges =msg.ranges
        self.points =[]
        self.plot_x =[]
        self.plot_y =[]
        rad_start = math.radians(45)
        rad_end = math.radians(135)
        idx_start = int((rad_start - msg.angle_min) / msg.angle_increment)
        idx_end = int((rad_end - msg.angle_min) / msg.angle_increment)
        indices = range(idx_start, idx_end + 1)
        vals = [msg.ranges[i] for i in indices if 0 <= i < len(msg.ranges) and not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i])]
        for i in indices:
            point =[0,0]
            theta = math.pi*(225-0.2497*i)/180
            length = self.ranges[i]
            point[0]=length*math.cos(theta)
            point[1]=length*math.sin(theta)
            self.points.append(point)
            self.plot_x.append(point[0])
            self.plot_y.append(point[1])
        self.ransac(max_loop=300,threshold=0.001,points=self.points)    
        
        
        
        
def main():
    rclpy.init()
    node = LidarNode()
    print('lidar_sample_node is started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__=='__main__':
    main()
