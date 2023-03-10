import numpy as np


P1 = np.array([[309, 304, 1], 
            [194, 310, 1], 
            [257, 317, 1],
            [236, 332, 1],
            [198, 430, 1],
            [316, 421, 1]])

P2 = np.array([[1.24925325, -0.84324552, 1],    
                [-1.16865626, -0.84324552, 1],                
                [0.17846475, -0.71515759, 1],
                [-0.33965872, -0.39493777,1],
                [-1.16865626, 1.3982932,1],
                [1.24925325, 1.3982932,1]])
'''
P2 = np.array([[255, 0, 1],    
                [0, 0, 1],                
                [145, 18, 1],
                [92, 54,1],
                [0, 255,1],
                [255, 255,1]])
'''

H = np.array([[1.740549, 0.009007, 254.633637], 
            [0.001351, 1.567363, 365.946805], 
            [0.000297, -0.000278, 0.997941]])

'''
H = np.array([[0.464673, 0.000907, 194.312117], 
            [-0.018674, 0.427163, 313.184939], 
            [0.000029, -0.000090, 1.005162]])
'''



for i in range(6):
    xyl=np.dot(H, P1[i])
    print(xyl)
    x = int(xyl[0]/xyl[2])
    y = int(xyl[1]/xyl[2])


    ex = P2[i][0] - x
    ey = P2[i][1] - y

    print("x : " +str(x)+ " y: "+ str(y))

    print(" Error x : " +str(ex)+ " Error y: "+ str(ey))

K = np.array([[510.96, 0, -326.80], 
            [0, 510.39, -263.16], 
            [0, 0, -1]])
print("K:")
print(K)
Kinv = np.linalg.inv(K)
print("Kinv:")
print(Kinv)
