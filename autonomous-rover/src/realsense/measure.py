import pyrealsense2 as rs
pipe = rs.pipeline()
profile = pipe.start()
try:
  for i in range(0, 100):
    frames = pipe.wait_for_frames()
    for f in frames:
      #print(f.__dir__())
      print(f.get_data.__dir__())
finally:
    pipe.stop()
#    float width = depth.get_width();
#    float height = depth.get_height();
#    rs2::depth_frame depth = frames.get_depth_frame();
#    float dist_to_center = depth.get_distance(width / 2, height / 2);
#    std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";



