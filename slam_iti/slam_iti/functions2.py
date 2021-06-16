import math

def menger_curvature(self, point_1_x, point_1_y, point_2_x, point_2_y, point_3_x, point_3_y):
        triangle_area = 0.5 * abs( (point_1_x*point_2_y) + (point_2_x*point_3_y) + (point_3_x*point_1_y) - (point_2_x*point_1_y) - (point_3_x*point_2_y) - (point_1_x*point_3_y))#Shoelace formula 
            
        curvature = (4*triangle_area) / (math.sqrt(pow(point_1_x - point_2_x,2)+pow(point_1_y - point_2_y,2)) * math.sqrt(pow(point_2_x - point_3_x,2)+pow(point_2_y - point_3_y,2)) * math.sqrt(pow(point_3_x - point_1_x,2)+pow(point_3_y - point_1_y,2)))#Menger curvature 
        return curvature
  
