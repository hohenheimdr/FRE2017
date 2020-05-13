<!---->
<launch>
<!-- Erstellung und  Vorfilterung der PointClouds in relevante Bereiche für RANSAC -->

  <node pkg="pointcloud_plantdetector" type="laser_row_filter" name="left_row_filter">
		<param name="cloud_in" value="/front_cloud_filtered" />
		<param name="frame_id" value="/base_link" />
		<param name="filtered_cloud" value="/left_row_test" />
		<param name="plants_pub" value="/left_row" />
		
		<param name="use_box_filter" value="true" />
		<param name="x_min" value="-0.3" />
		<param name="y_min" value="0.0" />
		<param name="z_min" value="-1.5" />
		<param name="x_max" value="0.6" />
		<param name="y_max" value="0.8" />
		<param name="z_max" value="1" />
		<param name="remove_plane" value="false" />
		<param name="use_radius_outlier_filter" value="false" />
		<param name="neighbour_nr" value="1" />
		<param name="use_statistical_outlier_filter" value="false" />
		<param name="push_to_plane" value="false" />
		<param name="use_eucledian_cluster" value="false" />
		<param name="cluster_distance" value="0.5" />
		
		<param name="use_mincluster" value="false" />		
</node>

 <node pkg="pointcloud_plantdetector" type="laser_row_filter" name="right_row_filter">
		<param name="cloud_in" value="/front_cloud_filtered" />
		<param name="frame_id" value="/base_link" />
		<param name="filtered_cloud" value="/right_row_test" />
		<param name="plants_pub" value="/right_row" />

		<param name="use_box_filter" value="true" />
		<param name="x_min" value="-0.3" />
		<param name="y_min" value="-0.8" />
		<param name="z_min" value="-1" />
		<param name="x_max" value="0.6" />
		<param name="y_max" value="0.0" />
		<param name="z_max" value="1" />	
		<param name="remove_plane" value="false" />
		<param name="use_radius_outlier_filter" value="false" />
		<param name="neighbour_nr" value="3" />
		<param name="use_statistical_outlier_filter" value="false" />
		<param name="push_to_plane" value="false" />
		<param name="use_eucledian_cluster" value="false" />
		<param name="cluster_distance" value="0.5" />
		<param name="use_mincluster" value="false" />		
</node>

<!-- RANSAC: Erstellung von Geradengleichung und Pose zur Reihenerkennung -->

  <node pkg="laser_row_detection" type="ransac_line_generator" name="line_generator_left" >
    <param name="cloud_in" value="/left_row"/>
    <param name="row_out" value="/line_pose_left"/>    
    <param name="frame_id" value="/base_link"/>
    <param name="mean_value" value="1"/>
    <param name="iterations" value="1000"/>
    <param name="add_points" value="1"/>
    <param name="distance" value="0.01"/>
    <param name="left_row" value="true"/>
    
  </node>
  
  <node pkg="laser_row_detection" type="ransac_line_generator" name="line_generator_right" >
    <param name="cloud_in" value="/right_row"/>
    <param name="row_out" value="/line_pose_right"/>    
    <param name="frame_id" value="/base_link"/>
    <param name="mean_value" value="1"/>
    <param name="iterations" value="1000"/>
    <param name="add_points" value="1"/>
    <param name="distance" value="0.01"/>
    <param name="left_row" value="false"/>

  </node>

<!-- Ermittlung des Korrekturwertes zur Navigation zwischen zwei Pflanzenreihen -->
##get out a speed value

 <node pkg="laser_row_detection" type="tf_row_publisher"  name="cmdvel_laser_correction">
    <param name="pose_right" value="/line_pose_right"/>
    <param name="pose_left" value="/line_pose_left"/>
    <param name="tf_name" value="row_point"/>
    <param name="frequency" value="5"/>
    <param name="offset_x" value="0.2"/>
    <param name="offset_y" value="0.02"/>
    <param name="robot_width" value="0.5"/>
    <param name="headland_out" value="/headland_detected"/>
  </node>

	
	<node pkg="basic_navigation" name="row_follower" type="point_follower">
		    <param name="cmd_vel_topic_id" value="/cmd_vel"/>
		    <param name="rabbit_frame_id" value="row_point"/>
		    <param name="vehicle_frame_id" value="base_link"/>
		    <param name="goal_reached" value="/movement_finished_row"/>
		   ##variables floor indoors... need a change for outdoor condition
 		   
		###defines when the algorithm tries to change the angle difference
 		    <param name="angle_react_thresh" value="0.03"/>
		## start angle when the vehicle starts moving... to not get a extreme turn....
		    <param name="start_moving_angle" value="0.1"/>
		## when goal is not reachable for 0.2 seconds skip goal
		    <param name="time_offset" value="0.15"/>
		
		 ###distance acceptable for goalpoint
		<param name="dist_thresh" value="0.1"/>
		###biggest acceptable error for angle difference
 		<param name="angle_thresh" value="0.03"/>
		 
		<param name="speed_min" value="0.1"/>	
 		### what value always should get started with... (start moving condition) 
		<param name="angle_speed_min" value="0.2"/>	
		<param name="speed_max" value="0.2"/>	<!--0.7-->
		<param name="angle_speed_max" value="0.8"/>		<!--2.0-->
		<param name="use_PID_for_turn" value="false"/>
		<param name="P" value="2.0"/>
		<param name="I" value="0.0"/>
		<param name="D" value="0.0"/>
		<param name="max_integral" value="0"/>
		  
	</node>

</launch>
