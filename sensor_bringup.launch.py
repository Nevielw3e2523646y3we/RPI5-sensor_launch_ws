from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

#    gscam_node = Node(
#        package='gscam2',
#        executable='gscam_main',
#        name='gscam_publisher',
#        namespace='pi_cam',
#        parameters=[{
#####       	    #'gscam_config': 'udpsrc port=5050 do-timestamp=true buffer-size=1048576 ! h264parse ! avdec_h264 ! videoconvert',
#	    'gscam_config': 'udpsrc port=5050 caps="video/mpegts, systemstream=true, packetsize=188" ! tsdemux ! h264parse ! avdec_h264 ! videoconvert ! queue max-size-buffers=1 leaky=downstream',
#            'image_encoding': 'rgb8',
#            'frame_id': 'pi_cam_optical_frame',
#	    'camera_info_url': 'file:///root/sensor_launch_ws/src/launch_all/launch/rpi_hq_1280x720.yaml',
#      	    'camera_name': 'rpi_hq_1280x720',
#            'use_gst_timestamps': True,
#            'sync_sink': False,
#            'preroll': False,
#        }],
#        remappings=[
#            ('/image_raw', '/pi_cam/image_raw'),
#            ('/camera_info', '/pi_cam/camera_info'),
#        ],
#        output='screen'
#    )


    # IMU + Mag fusion
    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_ahrs',
        remappings=[
            ('imu/data_raw', '/imu/data'),
            ('imu/data',      '/imu/data_ahrs')
        ],
        parameters=[{
            'use_mag': True,
            'use_magnetic_field_msg': True,
            'world_frame': 'enu',
            'mag_bias_x': 0.0,
            'mag_bias_y': 0.0,
            'mag_bias_z': 0.0,
            'publish_tf': False,
            'stateless': False,
	    'publish_tf': False,
            'orientation_stddev': 0.02,
	    'gain': 0.03,
	    'zeta': 0.010
        }],
        arguments=['--ros-args', '--log-level', 'imu_filter_madgwick:=DEBUG'],
        output='screen'
    )

    return LaunchDescription([
        # LiDAR
	Node(
	    package='ydlidar_ros2_driver',
	    executable='ydlidar_ros2_driver_node',
	    name='lidar',
	    parameters=[{
	        'port': '/dev/ttyAMA1',
	        'frame_id': 'laser_frame',
	        'baudrate': 230400,
	        'lidar_type': 1,
	        'device_type': 0,
	        'sample_rate': 4,
	        'intensity_bit': 8,
	        'abnormal_check_count': 4,
	        'fixed_resolution': True,
	        'reversion': False,
	        'inverted': False,
	        'auto_reconnect': True,
	        'isSingleChannel': False,
	        'intensity': True,
	        'support_motor_dtr': False,
	        'angle_max': 180.0,
	        'angle_min': -180.0,
	        'range_max': 12.0,
	        'range_min': 0.03,
	        'frequency': 10.0,
	        'invalid_range_is_inf': True
	    }]
	),
        # GPS
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps',
            parameters=[{
		'port': '/dev/ttyAMA0',
	 	'baud': 9600,
		'frame_id': 'gps',
		'use_gps_time': False,
		'nmea_sentence': True
		}]
        ),
        # Magnetometer
	Node(
    	    package='mag_tools',
            executable='hmc5883l_pub',
            name='hmc5883l',
            parameters=[{
                'bus': 1,
                'address': 0x1E,
                'rate': 20.0,
                'frame_id': 'imu_link',
                'topic': '/imu/mag',
                'gain_lsb_per_gauss': 1090.0,
                'bx': -6.651376146788991e-06,
                'by': -2.7477064220183487e-05,
                'bz':  1.880733944954129e-05,
            }],
        output='screen'
        ),

	# PiCamera
#	gscam_node,

	# IMU+Mag node
	madgwick,

	# Static transform

	# LiDAR
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_lidar',
	  arguments=['--x','0.02531','--y','0.00000','--z','0.15414',
	             '--roll','0','--pitch','0','--yaw','3.141592653589793',
	             '--frame-id','base_link','--child-frame-id','laser_frame']
	),

	# GPS
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_gps',
	  arguments=['--x','-0.02070','--y','0.00000','--z','0.14343',
	             '--roll','0','--pitch','0','--yaw','0',
	             '--frame-id','base_link','--child-frame-id','gps']
	),

	# Pi Camera
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_pi_cam',
	  arguments=['--x','0.10071','--y','0.02737','--z','0.10019',
	             '--roll','0','--pitch','0','--yaw','0',
	             '--frame-id','base_link','--child-frame-id','pi_cam_optical_frame']
	),

	# INMP441 microphones
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_inmp441_l',
	  arguments=['--x','-0.04360','--y','0.07332','--z','0.09980',
	             '--roll','0','--pitch','0','--yaw','1.5708',
	             '--frame-id','base_link','--child-frame-id','inmp441_l']
	),
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_inmp441_r',
	  arguments=['--x','-0.04360','--y','-0.07332','--z','0.09980',
	             '--roll','0','--pitch','0','--yaw','-1.5708',
	             '--frame-id','base_link','--child-frame-id','inmp441_r']
	),
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_inmp441_f',
	  arguments=['--x','0.06627','--y','0.00000','--z','0.10593',
	             '--roll','0','--pitch','0','--yaw','0',
	             '--frame-id','base_link','--child-frame-id','inmp441_f']
	),

	# AHT10 sensors
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_aht10_l',
	  arguments=['--x','0.00000','--y','0.07452','--z','0.10154',
	             '--roll','0','--pitch','0','--yaw','1.5708',
	             '--frame-id','base_link','--child-frame-id','aht10_l']
	),
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_aht10_r',
	  arguments=['--x','0.00000','--y','-0.07452','--z','0.10134',
	             '--roll','0','--pitch','0','--yaw','-1.5708',
	             '--frame-id','base_link','--child-frame-id','aht10_r']
	),
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_aht10_f',
	  arguments=['--x','0.06486','--y','0.0000','--z','0.08945',
	             '--roll','0','--pitch','0','--yaw','0',
	             '--frame-id','base_link','--child-frame-id','aht10_f']
	),

	# MQ2
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_mq2',
	  arguments=['--x','0.07897','--y','0.02610','--z','0.06953',
	             '--roll','0','--pitch','0','--yaw','3.141592653589793',
	             '--frame-id','base_link','--child-frame-id','mq2_gas']
	),

	# Ultrasonic
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_ultrasonic',
	  arguments=['--x','0.06780','--y','0.01483','--z','0.07594',
	             '--roll','0','--pitch','0','--yaw','3.141592653589793',
	             '--frame-id','base_link','--child-frame-id','ultrasonic']
	),

	# Motors
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_motor_front',
	  arguments=['--x','0.06907','--y','0.02926','--z','0.02809',
	             '--roll','0','--pitch','0','--yaw','-1.5708',
	             '--frame-id','base_link','--child-frame-id','motor_front']
	),
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_motor_back',
	  arguments=['--x','-0.06907','--y','0.02926','--z','0.02809',
	             '--roll','0','--pitch','0','--yaw','1.5708',
	             '--frame-id','base_link','--child-frame-id','motor_back']
	),

	# IMU
	Node(
	  package='tf2_ros', executable='static_transform_publisher', name='static_tf_imu',
	  arguments=['--x','0.04684','--y','0.00115','--z','0.05332',
	             '--roll','-0.0319','--pitch','0.0779','--yaw','0',
	             '--frame-id','base_link','--child-frame-id','imu_link']
	),
        # Encoder counts -> /wheel/odometry
        ExecuteProcess(
            cmd=[
                'python3',
                '/root/sensor_launch_ws/src/launch_all/scripts/encoder_odom.py',
                '--ticks_per_rev',  '885',
                '--wheel_diameter', '0.0492',
                '--track_width',    '0.1845',
                '--left_scale',	    '1.135784',
                '--right_scale',    '1.050518',
                '--left_topic',     '/encoders/left',
                '--right_topic',    '/encoders/right',
                '--odom_topic',     '/wheel/odometry',
                '--odom_frame',     'odom',
                '--base_link_frame','base_link',
            ],
            output='screen'
        ),

	#  Local EKF Obom + IMU
	Node(
	    package='robot_localization',
	    executable='ekf_node',
	    name='ekf_local_node',
	    output='screen',
	    parameters=['/root/sensor_launch_ws/src/launch_all/config/fusion/ekf_local.yaml'],
	    remappings=[
	        ('odometry/filtered', 'odometry/local'),
	    ],
	    arguments=['--ros-args', '--log-level', 'robot_localization:=DEBUG'],
	)

#	Node(
#	    package='robot_localization',
#	    executable='ekf_node',
#	    name='ekf_local_node',
#	    output='screen',
#	    parameters=['/root/sensor_launch_ws/src/launch_all/config/fusion/ekf_local.yaml'],
#	    remappings=[
#	        ('/odometry/filtered', '/odometry/local'),
#	    ],
#	),

 #       # Navsat: GPS + /odometry/local + IMU
 #       Node(
 #           package='robot_localization',
 #           executable='navsat_transform_node',
 #           name='navsat_transform_node',
 #           output='screen',
 #           parameters=['/root/sensor_launch_ws/src/launch_all/config/fusion/navsat.yaml'],
 #           remappings=[
 #               ('/imu', 'data_ahrs'),
 #               ('/gps/fix', '/fix'),
 #               ('/odometry/filtered', '/odometry/local'),
 #               ('/odometry/gps', '/odometry/gps'),
 #           ]
 #       ),

#        # Global EKF: GPS odom + wheel + IMU
#        Node(
#            package='robot_localization',
#            executable='ekf_node',
#            name='ekf_global_node',
#            output='screen',
#            parameters=['/root/sensor_launch_ws/src/launch_all/config/fusion/ekf_global.yaml'],
#            remappings=[
#                ('/odometry/filtered', '/odometry/global'),
#            ]
#        ),
    ])
