#!/usr/bin/env python
# -*- coding: utf-8 -*-
#$Id: conf.py 42 2010-08-28 03:44:55Z kejia $


valid_time = 2.0    # two seconds
max_speed = 0.5     # 0.5 m/s

# ROS related
odom_link = "/odom"
base_link = "/base_link"
arm_link = "/armbase_link"

cam1394_node_name = "/camera1394_nodelet"
vision_node_name = "/we_vision_node"
vfh_node_name   = "/we_local_vfh"
vfh_goal_name = "/we_local_vfh/goal"

use_local_vfh = False

""" TODO: clean up the configs """
header = '$'
test = False
debug = False
avoid = False
speed = 1.0
oldarm = True # set this false if using katana

half_wheel = 0.2
wheel_width = 0.4
ewiw_elevator_height = 0.6

chinese_version = False

do_log = True
logger = None
show_opt = True

wrist_length = 0.09#0.1
elevator_height_max = 0.73
elevator_height_min = 0.02#0.25

armASpeed = 15000
shoulderzSpeed = 1000000
shoulderySpeed = 500000
elbowSpeed = 500000
wristySpeed = 2000000
wristzSpeed = 500000
pawSpeed = 800000
arm_init_pose = [0.10,0,0.5]
arm_put_pose = [0.2, 0, 0.5]
arm_give_pose = [0.15, 0, 0.55]
pre_length = -0.01
hold_length = -0.08
fetch_offset = [-0.01,0.00,0.00]

fetch_obj_dist_max = 4.0
fetch_obj_height_max = 1.4
fetch_obj_height_min = 0.0
fetch_obj_offset = [20,0,0]
fetch_obj_sync_max =  20
search_obj_pitch = -30
arm_reach_length_min = 0.40 + wrist_length
arm_reach_length_max = 0.44 + wrist_length

eye2hand = [-0.2, 0.0, -0.26] #[-0.22, 0.0, -0.26] # ev = 0.0

laser_dist_error = 0.1#0.05

no_log_prefix = ['v', 'm']

entry = '_entry' #enter point
house = '_house' #the whole scenario

location = "ISC"

if location == "ISC":
    room = ""

    ##slam

    run_slam = False
    #run_slam = True

    slam_pose_sequence = [
    [4.5,0.5,1.57], # start

    [4.5,2,1.57],
    [3.5,4,3.14],
    [6.75,4,1.57],
    [3.5,6,3.14],
    [3.5,6,3.14],
    [5.2,8.5,0],
    [5.2,8.5,0],
    [9,9,0],
    [9,6.5,0],
    [9,3,-1.57],
    [3,1,3.14],

    [4.5,-0.5,-1.57], # end
    ]

 #   fetch_accept_objs = ["fanta", "water", "milk", "pepsi", "coca", "amsa", "sprite", "barbecue_chips"]
    fetch_accept_objs = [{*foreach $a as $value*}"{*$value*}",{*/foreach*}]

    ##restaurant & shopping
    shopping_dict = {
#                     "green_tea": "green_tea", \
#                  "yohurt": "yohurt", \
                  "fanta": "fanta", \
                  "water": "water", \
                  "milk":"milk", \
                  "pepsi":"pepsi", \
                  "coca":"coca",\
                  "amsa":"amsa",\
                  "sprite":"sprite",\

                  "barbecue_chips":"barbecue_chips"

#                  "cotton_buds":"cotton_buds", \
#                  "collar_cleaner":"collar_cleaner", \
#                  "red_paint":"red_paint", \

#                  "coffee": "coffee", \
#                  "plain_chips": "plain_chips", \
#                  "barbecue_chips": "barbecue_chips", \
#                  "fruit_drink": "fruit_drink", \
#                  "rice_cereal": "rice_cereal", \

#                  "tomato_sauce":"tomato_sauce", \
#                  "candle":"candle"
                  }

    restaurant_table_list = ["one", "two", "three"]

    ##clean up & go get it

    cleanup_start_pos = [2.465,-0.139,-1.690]

    obj_category_list = ["drinks", "snacks", "bathroom_stuff", "food", "unknown"]

    obj_category_dict = {
                    "fanta": "drinks", \
                  "water": "drinks", \
                  "milk":"drinks", \
                  "pepsi":"drinks", \
                  "coca":"drinks",\
                  "amsa":"drinks",\
                  "sprite":"drinks",\

                  "barbecue_chips":"snacks", \
                      "green_tea": "drink", \
                      "yohurt": "drink", \
                      "fanta": "drink", \
                      "water": "drink", \
                      "milk":"drink", \
                      "pepsi":"drink", \

                      "cotton_buds":"clean", \
                      "collar_clean":"clean", \
                      "red_paint":"clean", \

                      "coffee": "snack", \
                      "plain_chips": "snack", \
                      "barbecue_chips": "snack", \
                      "fruit_drink": "snack", \
                      "rice_cereal": "snack", \

                      "tomato_sauce":"kitchen", \
                      "candle":"kitchen"
                      }

    obj_category_put_pose = {"drinks":[4.213,0.141,-1.601, 0.5], #[x,y,r,height]
                    "snacks":[4.213,0.141,-1.601, 0.5],
                    "bathroom_stuff":[4.213,0.141,-1.601,0.5],
                    "food":[4.213,0.141,-1.601, 0.5],
                    "unknown":[4.213,0.141,-1.601, 0.5]}

    room_floor_pose = {"dining_room":[[2.894,-2.374,-0.754], [7.334,-2.088,-2.168], [7.484,-4.319,2.414]], \
                       "living_room":[[4.63, 0.61, 1.30],[4.21, 1.24, -3.00],[4.19, 2.89, 1.56]], \
                       "bedroom": [[5,6.5,3.0], [5.30, 7.75, 1.36]], \
                       "washroom": [],\
                       "passage": []}

    # update from we_gslam/datas/isc_entities.txt
    fetch_room_rect = {
        "none":[1.5,0,10,10],
        "living_room":[1.200,   -1.350,    6.250,    3.250],
        "dining_room":[1.200,   -5.450,    9.150,   -1.350],
        "bedroom":[6.250,   -1.350,   11.250,    1.600],
        "washroom":[9.150,   -5.450,   11.200,   -1.350],
    }

    room_rect = fetch_room_rect

    room_entrance = {
        "living_room": [1.91, 6.33, 0.30],
        "dining_room": [2.38, 3.79, -1.09],
        "bedroom": [7.61, 5.51, 1.59]
    }

    fetch_scan_pose = {
        "none":
        [
        [8,2.25,1.57],
        ],
        "dining_room":
        [
        #[[7.243,6.883,1.798],[0]],# oven_desk



        #[[8.9,3.122,-3.115],[0,-35,35]],#dining_table +x

        [[7.749,5.183,-1.563],[0,-35,35]],# dining_table +y
        [[9.062,3.9,0.021],[0]],#cupboard
        [[7.885,2.666,1.570],[0,-35,35]],# dining_table -y
        [[9.410,6.973,1.147],[0]],# fridge
        [[7.243,6.883,1.798],[0]],# oven_desk
        #[[6.892,3.368,-0.053],[0,-35,35]],# dining_table -x

        #[[8.214,6.668,1.530],[0,30,-30]],# shuichi

        #[[7.635,0.603,-1.586],[0]]# shujia
        ],
        "living_room":
        [
        [3.5, 1.75, -1.57],
        [3.9, 3.5, 1.57],
        ],
        "bedroom":
        [
        [5, 8, 1.57],
        ],
        "washroom":
        [
        [8.5, 8.5, 1.57],
        ],
        "fridge":
        [
        [[9.410,6.973,1.147],[0]]# fridge
        ],
        "dining_table":
        [
        [[8.05,3.4,3.1],[0,-35,35]],#dining_table +x
        [[7.124,4.490,-1.613],[0,-35,35]]# dining_table +y
        ],
        "cupboard":
        [
        [[9.062,3.990,0.021],[0]]
        ],
        "tv":
        [
        [[3.986,3.94,1.57],[0]]
        ]

    }

    ## who is who
    # for  test

    wiw_run_cali = True
    # wiw_run_cali = False

    wiw_door_conf = "door_b1"
    wiw_room_conf = "dining_room"

    wiw_face_zrange = [0.8,1.9,1.45]

    if wiw_door_conf == "door_b1":
        wiw_pose_start = [4.8,6.4,0]
        wiw_pose_end   = [7,6.5,-3.14]
        wiw_pose_exit  = [5,6.5,-3.14]
        wiw_door_enter = "door_b1"
        wiw_path_enter = [
        # [1,6.5,0.3],
        [3, -90, -30]
        ]
    elif wiw_door_conf == "door_rb1":
        wiw_pose_start = [7,6.5,3.14]
        wiw_pose_end   = [5,6.5,0]
        wiw_pose_exit  = [7,6.5,0]
        wiw_door_enter = "door_b1"
        wiw_path_enter = [
        [1,5.5,0.3],
        ]
    elif wiw_door_conf == "entry":
        wiw_pose_start = [4.5,-0.5,1.57]
        wiw_pose_end   = [4.5,1.0,-1.57]
        wiw_pose_exit  = [4.5,-0.5,-1.57]
        wiw_door_enter = "entry"
        wiw_path_enter = [
        [2,1,0.3],
        ]

    if wiw_room_conf == "living_room":
        wiw_roomid = "living_room"
        wiw_face_range = [1.6,0.1,5.9,4.9]
        wiw_scan_poses = [
        [5,1,2.1],
        [3.5,4,-1.57],
        ]
    elif wiw_room_conf == "dining_room":
        wiw_roomid = "dining_room"
        wiw_face_range = [6.1,0.1,9.9,7.9]
        wiw_scan_poses = [
        [8.8,2.0,2],
        [8,6.2,-1.57],
        ]
    elif wiw_room_conf == "bedroom":
        wiw_roomid = "bedroom"
        wiw_face_range = [3.1,5.1,6.9,9.9]
        wiw_scan_poses = [
        [5.3,6.5,-3.14],
        [5.5,8.5,0.7],
        ]

    wiw_seek_tracks = [
    [
    [-30,-10,10,10],
    [500,1000],
    #[-60,-15,10,10],
    #[500,1000],
    #[-90,-15,10,10],
    #[500,1000],
    [-120,-10,10,10],
    [500,1000],
    [-140,-10,10,10],
    [500,1000],
    [-140,0,10,10],
    [500,1000],
    #[-120,0,10,10],
    #[500,1000],
    #[-60,-0,10,10],
    #[500,1000],
    #[-30,-0,10,10],
    #[500,1000],
    [0,-0,10,10],
    [500,1000],
    [30,-0,10,10],
    [500,1000],
    [60,-0,10,10],
    [500,1000],
    [90,-0,10,10],
    [500,1000],
    [120,-0,10,10],
    [500,1000],
    [150,-0,10,10],
    [500,500],
    [90,-0,10,10],
    [500,1000],
    [30,-0,10,10],
    [500,1000],
    [0,-0,30,30],
    ],
    [
    [500, 1000],
    [-60,-15,10,10],
    [500,1000],
    [-120,-15,10,10],
    [500,1000],
    [-120,-0,10,10],
    [500,1000],
    [-60,-0,10,10],
    [500,1000],
    [0,-0,10,10],
    [500,1000],
    [100,-0,10,10],
    [500,500],
    [0,-0,30,30],
    ],
    ]

    ## enhanced who is who

    stand_height_min=1.4
    people_dist=0.75

    stand_num= 3
    wiw_cali_num = 3
    recog_dist=0.75
    recog_overtime=600
    ##common conf
    # for  test
    ewiw_pri_name = ['john','michael','joshua']
    ewiw_pri_name2obj = {
    'me':'herbal_tea',
    'john':'green_tea',
    'joshua':'plum_juice',
    }
    ewiw_pri_name = []
    ewiw_pri_name2obj = {}

    ewiw_fetch_locname = "dining_table"
    ewiw_fetch_name2loc = {
    "dining_table":[4.321,2.794,-0.013, 0.2],
    "sink":[8,6.5,1.57, 0.2],
    "desk":[6.2,8.9,1.57, 0.2],
    "shelf":[3.7, 1, -1.57, 0.2],
    "tv":[4.989,7.811,1.57],
    "goods":[8.75, 2, 3.14, 0.2],
    "fridge":[9.410,6.973,1.147, 0.2],
    "xiju":[9.25, 9.25, 0.75, 0.2],
    }

    ewiw_door_conf = "door_b1"
    ewiw_room_conf = "living_room"
    ewiw_face_zrange = [0.8,1.9,1.45]

    if ewiw_door_conf == "door_b1":
        ewiw_pose_start = [7.168,3.401,0.0]
        ewiw_pose_end   = [0.232,6.235,3.097]
        ewiw_pose_exit  = [0.232,6.235,3.097]
        ewiw_door_enter = "door_b1"
        ewiw_path_enter = [
        [1,6.5,0.3],
        ]
    elif ewiw_door_conf == "door_rb1":
        ewiw_pose_start = [7.5,6.8,0]
        ewiw_pose_end   = [5,6.5,3.14]
        ewiw_pose_exit  = [7,6.5,3.14]
        ewiw_door_enter = "door_b1"
        ewiw_path_enter = [
        [1,5.5,0.3],
        ]
    elif ewiw_door_conf == "entry":
        ewiw_pose_start = [2.924,6.260,1.568]
        ewiw_pose_end   = [7.612,6.383,1.570]
        ewiw_pose_exit  = [7.612,6.383,1.570]
        ewiw_door_enter = "entry"
        ewiw_path_enter = [
        [2,1,0.3],
        ]

    if ewiw_room_conf == "living_room":
        ewiw_roomid = "living_room"
        ewiw_face_range = [1.6,0.1,5.9,4.9]
        ewiw_scan_poses = [
#        [5,1,3.14],
        [2.489,6.225,0.750],
        ]
    elif ewiw_room_conf == "dining_room":
        ewiw_roomid = "dining_room"
        ewiw_face_range = [6.1,0.1,9.9,7.9]
        ewiw_scan_poses = [
       # [8.5,6,-2.3],
        [6.027,1.813,1.558],
        ]
    elif ewiw_room_conf == "bedroom":
        ewiw_roomid = "bedroom"
        ewiw_face_range = [3.1,5.1,6.9,9.9]
        ewiw_scan_poses = [
        [5.3,6.5,-3.14],
        [5.3,8,1.57],
        ]


    ewiw_scan_tracks = [
    [
    [-100,400,5,5],
    [500,300],
    [100,400,5,5],
    [500,300],
    [0,400,5,5],
    ],
    [
    [-100,400,5,5],
    [500,300],
    [100,400,5,5],
    [500,300],
    [0,400,5,5],
    ],
    ]

    ewiw_seek_tracks = [
    [
    [-100,-15,30,30],
    [500,300],
    [100,5,30,30],
    [500,300],
    [0,0,30,30],
    ],
    [
    [-100,-15,30,30],
    [500,300],
    [100,5,30,30],
    [500,300],
    [0,0,30,30],
    ],
    ]

    enterdoor = "entry"

elif location == "MEXICA":
    room = ""

    ##slam

    run_slam = False
    #run_slam = True

    slam_pose_sequence = [
    [4.5,0.5,1.57], # start

    [4.5,2,1.57],
    [3.5,4,3.14],
    [6.75,4,1.57],
    [3.5,6,3.14],
    [3.5,6,3.14],
    [5.2,8.5,0],
    [5.2,8.5,0],
    [9,9,0],
    [9,6.5,0],
    [9,3,-1.57],
    [3,1,3.14],

    [4.5,-0.5,-1.57], # end
    ]

    fetch_accept_objs = ["coke", "seven_up", "lemon_tea", "pepsi", "mineral_water", "gatorade", "orange_juice", "apple_juice", \
                         "wafer", "candy", \
                         "chili", "dough", "tomato_sauce", "tofu", "tea_box", "cotton", \
                         "disinfactant", "hair_gel", "soap", "tooth_paste", "pad"]

    ##restaurant & shopping
    shopping_dict = {"coke": "coke", \
                  "seven_up": "seven_up", \
                  "lemon_tea": "lemon_tea", \
                  "mineral_water": "mineral_water", \
                  "gatorade":"gatorade", \
                  "pepsi":"pepsi", \
                  "orange_juice":"orange_juice", \
                  "apple_juice":"apple_juice", \

                  "chili":"chili", \
                  "dough":"dough", \
                  "tomato_sauce":"tomato_sauce", \
                  "soy_sauce": "soy_sauce", \
                  "tofu": "tofu", \
                  "tea_box": "tea_box"
                  }

    restaurant_table_list = ["one", "two", "three"]

    ##clean up & go get it
    obj_category_list = ["drinks", "snacks", "food", "bathroom_stuff", "unknown"]

    obj_category_dict = {
                      "coke": "drinks", \
                      "seven_up": "drinks", \
                      "lemon_tea": "drinks", \
                      "pepsi": "drinks", \
                      "mineral_water":"drinks", \
                      "gatorade":"drinks", \
                      "orange_juice":"drinks", \
                      "apple_juice":"drinks", \

                      "crackers": "snacks", \
                      "wafer": "snacks", \
                      "candy": "snacks", \
                      "cookies": "snacks", \

                      "chili":"food", \
                      "dough":"food", \
                      "tomato_sauce":"food", \
                      "soy_sauce":"food", \
                      "tofu":"food", \
                      "tea_box":"food", \

                      "cotton":"bathroom_stuff", \

                      "disinfactant":"bathroom_stuff", \
                      "hair_gel":"bathroom_stuff", \
                      "soap":"bathroom_stuff", \
                      "tooth_paste":"bathroom_stuff", \
                      "toliet_paper":"bathroom_stuff", \
                      "shampoo":"bathroom_stuff", \
                      "pad":"bathroom_stuff"
                      }

#    cleanup_start_pos = [8.143,3.539,0.736]
    cleanup_start_pos = [6.50, 2.90, 1.56]
#    obj_category_put_pose = { #ArenaA
#                    "drinks":[2.612,7.358,-1.522, 0.6], #[x,y,r,height] bar
#                    "snacks":[7.236,3.978,3.041, 0.62], # sideboard
#                    "food":[3.513,7.571,-0.019, 0.55], # kitchen table
#                    "bathroom_stuff":[3.137,2.517,1.613, 0.55], # bed
#                    "unknown":[9.63, 4.11, -0.78, 0.5] #trash bin
#                    }

    obj_category_put_pose = { #ArenaB
                    "drinks":[11.84, 7.72, -1.59, 0.6], #[x,y,r,height] bar
                    "snacks":[7.43, 3.91, -0.00, 0.62], # sideboard
                    "food":[10.79, 8.38, 3.13, 0.55], # kitchen table
                    "bathroom_stuff":[3.137,2.517,1.613, 0.55], # bed
                    "unknown":[4.72, 4.71, -2.68, 0.5] #trash bin
                    }
#    room_floor_pose = {"living_room":[[9.70, 5.97, 2.69],[5.55, 9.82, -0.67]], #,[5.30, 5.76, 0.67]], \ #ArenaA
#                       "lobby":[[6.65, 1.98, 0.72], [9.65, 5.01, -2.26]],
#                       "bedroom": [[4.33, 2.12, 2.60], [1.96, 2.32, 1.56]],
#                       "kitchen": [[4.18, 9.72, -2.38], [1.73, 9.54, -0.94]]}

    room_floor_pose = { #ArenaB
                       "living_room":[[4.945,6.449,0.768], [9.24, 9.95, -2.31]],
                       "lobby":[[6.59, 2.53, 1.88]],
                       "bedroom": [[10.25, 2.40, 0.83]],
                       "kitchen": [[10.22, 10.07, -0.88]]}

    # update from we_gslam/datas/isc_entities.txt
#    fetch_room_rect = { #ArenaA
#        "living_room":[4.625,5.320,10.606,10.317],
#        "lobby":[4.641,1.354,10.606,5.336],
#        "bedroom":[0.627,1.291,4.641,5.019],
#        "kitchen":[0.611,6.208,4.609,10.285]
#    }

    fetch_room_rect = { #ArenaB
        "living_room":[3.659,5.719,9.807,10.637],
        "lobby":[3.640,1.694,8.319,5.719],
        "bedroom":[9.569,1.733,13.674,5.481],
        "kitchen":[9.787,6.671,13.693,10.657]
    }

    room_rect = fetch_room_rect

    room_entrance = {
        "living_room": [1.91, 6.33, 0.30],
        "dining_room": [2.38, 3.79, -1.09],
        "bedroom": [7.61, 5.51, 1.59]
    }

    fetch_scan_pose = {
        "none":
        [
        [8,2.25,1.57],
        ],
        "dining_room":
        [
        #[[7.243,6.883,1.798],[0]],# oven_desk



        #[[8.9,3.122,-3.115],[0,-35,35]],#dining_table +x

        [[7.749,5.183,-1.563],[0,-35,35]],# dining_table +y
        [[9.062,3.9,0.021],[0]],#cupboard
        [[7.885,2.666,1.570],[0,-35,35]],# dining_table -y
        [[9.410,6.973,1.147],[0]],# fridge
        [[7.243,6.883,1.798],[0]],# oven_desk
        #[[6.892,3.368,-0.053],[0,-35,35]],# dining_table -x

        #[[8.214,6.668,1.530],[0,30,-30]],# shuichi

        #[[7.635,0.603,-1.586],[0]]# shujia
        ],
        "living_room":
        [
        [3.5, 1.75, -1.57],
        [3.9, 3.5, 1.57],
        ],
        "bedroom":
        [
        [5, 8, 1.57],
        ],
        "washroom":
        [
        [8.5, 8.5, 1.57],
        ],
        "fridge":
        [
        [[9.410,6.973,1.147],[0]]# fridge
        ],
        "dining_table":
        [
        [[8.05,3.4,3.1],[0,-35,35]],#dining_table +x
        [[7.124,4.490,-1.613],[0,-35,35]]# dining_table +y
        ],
        "cupboard":
        [
        [[9.062,3.990,0.021],[0]]
        ],
        "tv":
        [
        [[3.986,3.94,1.57],[0]]
        ]

    }

    ## who is who
    # for  test

    wiw_run_cali = True
    # wiw_run_cali = False

    wiw_door_conf = "door_b1"
    wiw_room_conf = "dining_room"

    wiw_face_zrange = [0.8,1.9,1.45]

    if wiw_door_conf == "door_b1":
        wiw_pose_start = [4.8,6.4,0]
        wiw_pose_end   = [7,6.5,-3.14]
        wiw_pose_exit  = [5,6.5,-3.14]
        wiw_door_enter = "door_b1"
        wiw_path_enter = [
        # [1,6.5,0.3],
        [3, -90, -30]
        ]
    elif wiw_door_conf == "door_rb1":
        wiw_pose_start = [7,6.5,3.14]
        wiw_pose_end   = [5,6.5,0]
        wiw_pose_exit  = [7,6.5,0]
        wiw_door_enter = "door_b1"
        wiw_path_enter = [
        [1,5.5,0.3],
        ]
    elif wiw_door_conf == "entry":
        wiw_pose_start = [4.5,-0.5,1.57]
        wiw_pose_end   = [4.5,1.0,-1.57]
        wiw_pose_exit  = [4.5,-0.5,-1.57]
        wiw_door_enter = "entry"
        wiw_path_enter = [
        [2,1,0.3],
        ]

    if wiw_room_conf == "living_room":
        wiw_roomid = "living_room"
        wiw_face_range = [1.6,0.1,5.9,4.9]
        wiw_scan_poses = [
        [5,1,2.1],
        [3.5,4,-1.57],
        ]
    elif wiw_room_conf == "dining_room":
        wiw_roomid = "dining_room"
        wiw_face_range = [6.1,0.1,9.9,7.9]
        wiw_scan_poses = [
        [8.8,2.0,2],
        [8,6.2,-1.57],
        ]
    elif wiw_room_conf == "bedroom":
        wiw_roomid = "bedroom"
        wiw_face_range = [3.1,5.1,6.9,9.9]
        wiw_scan_poses = [
        [5.3,6.5,-3.14],
        [5.5,8.5,0.7],
        ]

    wiw_seek_tracks = [
    [
    [-30,-10,10,10],
    [500,1000],
    #[-60,-15,10,10],
    #[500,1000],
    #[-90,-15,10,10],
    #[500,1000],
    [-120,-10,10,10],
    [500,1000],
    [-140,-10,10,10],
    [500,1000],
    [-140,0,10,10],
    [500,1000],
    #[-120,0,10,10],
    #[500,1000],
    #[-60,-0,10,10],
    #[500,1000],
    #[-30,-0,10,10],
    #[500,1000],
    [0,-0,10,10],
    [500,1000],
    [30,-0,10,10],
    [500,1000],
    [60,-0,10,10],
    [500,1000],
    [90,-0,10,10],
    [500,1000],
    [120,-0,10,10],
    [500,1000],
    [150,-0,10,10],
    [500,500],
    [90,-0,10,10],
    [500,1000],
    [30,-0,10,10],
    [500,1000],
    [0,-0,30,30],
    ],
    [
    [500, 1000],
    [-60,-15,10,10],
    [500,1000],
    [-120,-15,10,10],
    [500,1000],
    [-120,-0,10,10],
    [500,1000],
    [-60,-0,10,10],
    [500,1000],
    [0,-0,10,10],
    [500,1000],
    [100,-0,10,10],
    [500,500],
    [0,-0,30,30],
    ],
    ]

    ## enhanced who is who

    stand_height_min=1.4
    people_dist=0.75

    stand_num= 3
    wiw_cali_num = 3
    recog_dist=0.75
    recog_overtime=300
    ##common conf
    # for  test
    ewiw_pri_name = ['john','michael','joshua']
    ewiw_pri_name2obj = {
    'me':'herbal_tea',
    'john':'green_tea',
    'joshua':'plum_juice',
    }
    ewiw_pri_name = []
    ewiw_pri_name2obj = {}

    ewiw_fetch_locname = "kitchen_table"
    ewiw_fetch_name2loc = {
#    "kitchen_table":[2.625,7.632,-1.57, 0.2],
    "kitchen_table":[2.596,7.509,-1.57],
    "sink":[8,6.5,1.57, 0.2],
    "desk":[6.2,8.9,1.57, 0.2],
    "shelf":[3.7, 1, -1.57, 0.2],
    "tv":[4.989,7.811,1.57],
    "goods":[8.75, 2, 3.14, 0.2],
    "fridge":[9.410,6.973,1.147, 0.2],
    "xiju":[9.25, 9.25, 0.75, 0.2],
    }

    ewiw_door_conf = "entry"
    ewiw_room_conf = "living_room"
    ewiw_face_zrange = [0.8,1.9,1.45]

    if ewiw_door_conf == "door_b1":
        ewiw_pose_start = [7.168,3.401,0.0]
        ewiw_pose_end   = [0.232,6.235,3.097]
        ewiw_pose_exit  = [0.232,6.235,3.097]
        ewiw_door_enter = "door_b1"
        ewiw_path_enter = [
        [1,6.5,0.3],
        ]
    elif ewiw_door_conf == "door_rb1":
        ewiw_pose_start = [7.5,6.8,0]
        ewiw_pose_end   = [5,6.5,3.14]
        ewiw_pose_exit  = [7,6.5,3.14]
        ewiw_door_enter = "door_b1"
        ewiw_path_enter = [
        [1,5.5,0.3],
        ]
    elif ewiw_door_conf == "entry":
        ewiw_pose_start = [8.142,3.383,1.57]
        ewiw_pose_end   = [5.734,11.266,1.57]
        ewiw_pose_exit  = [5.734,11.266,1.57]
        ewiw_door_enter = "entry"
        ewiw_path_enter = [
        [2,1,0.3],
        ]

    if ewiw_room_conf == "living_room":
        ewiw_roomid = "living_room"
        ewiw_face_range = [1.6,0.1,5.9,4.9]
        ewiw_scan_poses = [
#        [5,1,3.14],
        [9.015,6.475,2.336],
        [7.279,8.404,-1.57],
        ]
    elif ewiw_room_conf == "dining_room":
        ewiw_roomid = "dining_room"
        ewiw_face_range = [6.1,0.1,9.9,7.9]
        ewiw_scan_poses = [
       # [8.5,6,-2.3],
        [6.027,1.813,1.558],
        ]
    elif ewiw_room_conf == "bedroom":
        ewiw_roomid = "bedroom"
        ewiw_face_range = [3.1,5.1,6.9,9.9]
        ewiw_scan_poses = [
        [5.3,6.5,-3.14],
        [5.3,8,1.57],
        ]


    ewiw_scan_tracks = [
    [
    [-100,400,5,5],
    [500,300],
    [100,400,5,5],
    [500,300],
    [0,400,5,5],
    ],
    [
    [-100,400,5,5],
    [500,300],
    [100,400,5,5],
    [500,300],
    [0,400,5,5],
    ],
    ]

    ewiw_seek_tracks = [
    [
    [-100,-15,30,30],
    [500,300],
    [100,5,30,30],
    [500,300],
    [0,0,30,30],
    ],
    [
    [-100,-15,30,30],
    [500,300],
    [100,5,30,30],
    [500,300],
    [0,0,30,30],
    ],
    ]

    enterdoor = "entry"

